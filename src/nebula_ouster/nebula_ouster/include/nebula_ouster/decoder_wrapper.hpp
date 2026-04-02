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

#pragma once

#include "nebula_core_ros/watchdog_timer.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_ouster_common/ouster_common.hpp>
#include <nebula_ouster_decoders/ouster_driver.hpp>
#include <nebula_ouster_hw_interfaces/ouster_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <ouster_msgs/msg/ouster_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <memory>
#include <mutex>

namespace nebula::ros
{

/// @brief Decoder wrapper for Ouster driver
class OusterDecoderWrapper
{
public:
  explicit OusterDecoderWrapper(
    rclcpp::Node * const parent_node,
    const std::shared_ptr<nebula::drivers::OusterHwInterface> & hw_interface,
    const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & config,
    const std::shared_ptr<const nebula::drivers::OusterCalibrationConfiguration> & calibration);

  void process_cloud_packet(std::unique_ptr<nebula_msgs::msg::NebulaPacket> packet_msg);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & new_config);

  nebula::drivers::Status status();

private:
  void publish_cloud(
    std::unique_ptr<sensor_msgs::msg::PointCloud2> pointcloud,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher);

  static std::chrono::nanoseconds to_nanos(double seconds)
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(seconds));
  }

  nebula::drivers::Status status_;
  rclcpp::Logger logger_;

  std::shared_ptr<nebula::drivers::OusterHwInterface> hw_interface_;
  std::shared_ptr<const drivers::OusterSensorConfiguration> sensor_cfg_;
  std::shared_ptr<const drivers::OusterCalibrationConfiguration> calibration_cfg_;

  std::shared_ptr<drivers::OusterDriver> driver_ptr_;
  std::mutex mtx_driver_ptr_;

  rclcpp::Publisher<ouster_msgs::msg::OusterScan>::SharedPtr packets_pub_{};
  ouster_msgs::msg::OusterScan::UniquePtr current_scan_msg_{};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nebula_points_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_ex_pub_{};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aw_points_base_pub_{};

  std::shared_ptr<WatchdogTimer> cloud_watchdog_;
};

}  // namespace nebula::ros
