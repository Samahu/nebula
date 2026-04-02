// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nebula_ouster/decoder_wrapper.hpp"

#include <nebula_core_ros/point_cloud_conversions.hpp>

#include <algorithm>
#include <memory>
#include <tuple>
#include <utility>

namespace nebula::ros
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

OusterDecoderWrapper::OusterDecoderWrapper(
  rclcpp::Node * const parent_node,
  const std::shared_ptr<nebula::drivers::OusterHwInterface> & hw_interface,
  const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & config,
  const std::shared_ptr<const nebula::drivers::OusterCalibrationConfiguration> & calibration)
: status_(nebula::drivers::Status::NOT_INITIALIZED),
  logger_(parent_node->get_logger().get_child("DecoderWrapper")),
  hw_interface_(hw_interface),
  sensor_cfg_(config),
  calibration_cfg_(calibration),
  driver_ptr_(new drivers::OusterDriver(config, calibration))
{
  status_ = driver_ptr_->get_status();

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Publish raw packets only when HW interface is active
  if (hw_interface_) {
    current_scan_msg_ = std::make_unique<ouster_msgs::msg::OusterScan>();
    packets_pub_ = parent_node->create_publisher<ouster_msgs::msg::OusterScan>(
      "ouster_packets", rclcpp::SensorDataQoS());
  }

  auto qos_profile = rmw_qos_profile_sensor_data;
  auto pointcloud_qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

  nebula_points_pub_ = parent_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "ouster_points", pointcloud_qos);
  aw_points_base_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points", pointcloud_qos);
  aw_points_ex_pub_ =
    parent_node->create_publisher<sensor_msgs::msg::PointCloud2>("aw_points_ex", pointcloud_qos);

  cloud_watchdog_ =
    std::make_shared<WatchdogTimer>(*parent_node, 100'000us, [this, parent_node](bool ok) {
      if (ok) return;
      RCLCPP_WARN_THROTTLE(
        logger_, *parent_node->get_clock(), 5000, "Missed pointcloud output deadline");
    });

  RCLCPP_INFO_STREAM(logger_, "Initialized OusterDecoderWrapper. Status=" << status_);
}

void OusterDecoderWrapper::process_cloud_packet(
  std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg)
{
  // Accumulate packets for raw scan publishing
  if (
    hw_interface_ && (packets_pub_->get_subscription_count() > 0 ||
                      packets_pub_->get_intra_process_subscription_count() > 0)) {
    if (current_scan_msg_->packets.empty()) {
      current_scan_msg_->header.stamp = packet_msg->stamp;
    }
    ouster_msgs::msg::OusterPacket ouster_pkt{};
    ouster_pkt.stamp = packet_msg->stamp;
    ouster_pkt.data = packet_msg->data;
    current_scan_msg_->packets.emplace_back(std::move(ouster_pkt));
  }

  std::tuple<nebula::drivers::NebulaPointCloudPtr, double> pointcloud_ts{};
  nebula::drivers::NebulaPointCloudPtr pointcloud = nullptr;

  {
    std::lock_guard lock(mtx_driver_ptr_);
    pointcloud_ts = driver_ptr_->parse_cloud_packet(packet_msg->data);
    pointcloud = std::get<0>(pointcloud_ts);
  }

  if (pointcloud == nullptr) return;

  cloud_watchdog_->update();

  if (current_scan_msg_ && !current_scan_msg_->packets.empty()) {
    packets_pub_->publish(std::move(current_scan_msg_));
    current_scan_msg_ = std::make_unique<ouster_msgs::msg::OusterScan>();
  }

  if (
    nebula_points_pub_->get_subscription_count() > 0 ||
    nebula_points_pub_->get_intra_process_subscription_count() > 0) {
    auto ros_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    *ros_pc = nebula::ros::to_ros_msg(*pointcloud);
    ros_pc->header.stamp =
      rclcpp::Time(to_nanos(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc), nebula_points_pub_);
  }
  if (
    aw_points_base_pub_->get_subscription_count() > 0 ||
    aw_points_base_pub_->get_intra_process_subscription_count() > 0) {
    auto cloud_xyzi =
      nebula::drivers::convert_point_xyzircaedt_to_point_xyzir(*pointcloud);
    auto ros_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    *ros_pc = nebula::ros::to_ros_msg(cloud_xyzi);
    ros_pc->header.stamp =
      rclcpp::Time(to_nanos(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc), aw_points_base_pub_);
  }
  if (
    aw_points_ex_pub_->get_subscription_count() > 0 ||
    aw_points_ex_pub_->get_intra_process_subscription_count() > 0) {
    auto cloud_ex = nebula::drivers::convert_point_xyzircaedt_to_point_xyziradt(
      *pointcloud, std::get<1>(pointcloud_ts));
    auto ros_pc = std::make_unique<sensor_msgs::msg::PointCloud2>();
    *ros_pc = nebula::ros::to_ros_msg(cloud_ex);
    ros_pc->header.stamp =
      rclcpp::Time(to_nanos(std::get<1>(pointcloud_ts)).count());
    publish_cloud(std::move(ros_pc), aw_points_ex_pub_);
  }
}

void OusterDecoderWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_driver_ptr_);
  driver_ptr_ = std::make_shared<drivers::OusterDriver>(new_config, calibration_cfg_);
  sensor_cfg_ = new_config;
}

nebula::drivers::Status OusterDecoderWrapper::status()
{
  return status_;
}

void OusterDecoderWrapper::publish_cloud(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher)
{
  if (pointcloud->header.stamp.sec < 0) {
    RCLCPP_WARN_STREAM(logger_, "Timestamp error, verify clock source.");
    return;
  }
  pointcloud->header.frame_id = sensor_cfg_->frame_id;
  publisher->publish(std::move(pointcloud));
}

}  // namespace nebula::ros
