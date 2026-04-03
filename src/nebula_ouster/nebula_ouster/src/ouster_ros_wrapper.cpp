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

#include "nebula_ouster/ouster_ros_wrapper.hpp"

#include "nebula_core_ros/parameter_descriptors.hpp"

#include <nebula_core_common/util/string_conversions.hpp>
#include <rclcpp/qos.hpp>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace nebula::ros
{

OusterRosWrapper::OusterRosWrapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("ouster_ros_wrapper", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  wrapper_status_(Status::NOT_INITIALIZED),
  sensor_cfg_ptr_(nullptr)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  wrapper_status_ = declare_and_get_sensor_config_params();

  if (wrapper_status_ != Status::OK) {
    throw std::runtime_error(
      "Sensor configuration invalid: " + util::to_string(wrapper_status_));
  }

  RCLCPP_INFO_STREAM(get_logger(), "Sensor Configuration: " << *sensor_cfg_ptr_);

  launch_hw_ = declare_parameter<bool>("launch_hw", param_read_only());

  if (launch_hw_) {
    hw_interface_wrapper_.emplace(this, sensor_cfg_ptr_);
    hw_monitor_wrapper_.emplace(this, sensor_cfg_ptr_);
  }

  decoder_wrapper_.emplace(
    this,
    hw_interface_wrapper_ ? hw_interface_wrapper_->hw_interface() : nullptr,
    sensor_cfg_ptr_,
    calibration_cfg_ptr_);

  RCLCPP_INFO_STREAM(get_logger(), "Decoder wrapper status: " << decoder_wrapper_->status());

  if (launch_hw_) {
    hw_interface_wrapper_->hw_interface()->register_scan_callback(
      std::bind(&OusterRosWrapper::receive_cloud_packet_callback, this, std::placeholders::_1));
    stream_start();
  } else {
    packets_sub_ = create_subscription<ouster_msgs::msg::OusterScan>(
      "ouster_packets", rclcpp::SensorDataQoS(),
      std::bind(
        &OusterRosWrapper::receive_scan_message_callback, this, std::placeholders::_1));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Hardware connection disabled, listening for packets on "
        << packets_sub_->get_topic_name());
  }

  parameter_event_cb_ = add_on_set_parameters_callback(
    std::bind(&OusterRosWrapper::on_parameter_change, this, std::placeholders::_1));
}

Status OusterRosWrapper::declare_and_get_sensor_config_params()
{
  nebula::drivers::OusterSensorConfiguration config;

  auto _sensor_model = declare_parameter<std::string>("sensor_model", param_read_only());
  config.sensor_model = drivers::sensor_model_from_string(_sensor_model);

  auto _return_mode = declare_parameter<std::string>("return_mode", param_read_write());
  config.return_mode = drivers::return_mode_from_string_ouster(_return_mode);

  config.host_ip = declare_parameter<std::string>("host_ip", param_read_only());
  config.sensor_ip = declare_parameter<std::string>("sensor_ip", param_read_only());
  config.data_port = declare_parameter<uint16_t>("data_port", param_read_only());
  config.imu_port = declare_parameter<uint16_t>("imu_port", param_read_only());
  config.frame_id = declare_parameter<std::string>("frame_id", param_read_write());
  config.packet_mtu_size = declare_parameter<uint16_t>("packet_mtu_size", param_read_only());

  {
    rcl_interfaces::msg::ParameterDescriptor desc = param_read_write();
    desc.additional_constraints = "Scan start angle [0., 360.] degrees";
    desc.floating_point_range = float_range(0, 360, 0.01);
    config.scan_phase = declare_parameter<double>("scan_phase", desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc = param_read_write();
    desc.additional_constraints = "Minimum range [0.1, 10.0] m";
    desc.floating_point_range = float_range(0.1, 10.0, 0.01);
    config.min_range = declare_parameter<double>("min_range", desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor desc = param_read_write();
    desc.additional_constraints = "Maximum range [1.0, 400.0] m";
    desc.floating_point_range = float_range(1.0, 400.0, 0.1);
    config.max_range = declare_parameter<double>("max_range", desc);
  }

  // Columns per frame: 512, 1024, or 2048
  config.columns_per_frame =
    static_cast<uint16_t>(declare_parameter<int>("columns_per_frame", param_read_only()));

  // Load calibration
  auto calibration_file =
    declare_parameter<std::string>("calibration_file", param_read_only());

  auto calib = std::make_shared<nebula::drivers::OusterCalibrationConfiguration>();
  calib->calibration_file = calibration_file;

  if (!calibration_file.empty()) {
    auto status = calib->load_from_file(calibration_file);
    if (status != nebula::Status::OK) {
      RCLCPP_WARN_STREAM(
        get_logger(),
        "Could not load calibration file '" << calibration_file
          << "': " << util::to_string(status)
          << ". Using default approximate angles.");
      calib->set_default_for_model(config.sensor_model);
    }
  } else {
    RCLCPP_WARN(
      get_logger(), "No calibration file specified. Using default approximate angles.");
    calib->set_default_for_model(config.sensor_model);
  }

  calibration_cfg_ptr_ = calib;

  auto new_cfg_ptr =
    std::make_shared<const nebula::drivers::OusterSensorConfiguration>(config);
  return validate_and_set_config(new_cfg_ptr);
}

Status OusterRosWrapper::validate_and_set_config(
  std::shared_ptr<const drivers::OusterSensorConfiguration> & new_config)
{
  if (!drivers::is_ouster_model(new_config->sensor_model)) {
    return Status::INVALID_SENSOR_MODEL;
  }
  if (new_config->return_mode == nebula::drivers::ReturnMode::UNKNOWN) {
    return Status::INVALID_ECHO_MODE;
  }
  if (new_config->frame_id.empty()) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  if (hw_interface_wrapper_) {
    hw_interface_wrapper_->on_config_change(new_config);
  }
  if (hw_monitor_wrapper_) {
    hw_monitor_wrapper_->on_config_change(new_config);
  }
  if (decoder_wrapper_) {
    decoder_wrapper_->on_config_change(new_config);
  }

  sensor_cfg_ptr_ = new_config;
  return Status::OK;
}

void OusterRosWrapper::receive_scan_message_callback(
  std::unique_ptr<ouster_msgs::msg::OusterScan> scan_msg)
{
  if (hw_interface_wrapper_) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Ignoring received OusterScan. Launch with launch_hw:=false to enable packet replay.");
    return;
  }
  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }
  for (auto & pkt : scan_msg->packets) {
    auto nebula_pkt = std::make_unique<nebula_msgs::msg::NebulaPacket>();
    nebula_pkt->stamp = pkt.stamp;
    nebula_pkt->data = std::move(pkt.data);
    decoder_wrapper_->process_cloud_packet(std::move(nebula_pkt));
  }
}

Status OusterRosWrapper::get_status()
{
  return wrapper_status_;
}

Status OusterRosWrapper::stream_start()
{
  if (!hw_interface_wrapper_) {
    return Status::UDP_CONNECTION_ERROR;
  }
  if (hw_interface_wrapper_->status() != Status::OK) {
    return hw_interface_wrapper_->status();
  }
  return hw_interface_wrapper_->hw_interface()->sensor_interface_start();
}

rcl_interfaces::msg::SetParametersResult OusterRosWrapper::on_parameter_change(
  const std::vector<rclcpp::Parameter> & p)
{
  using rcl_interfaces::msg::SetParametersResult;

  if (p.empty()) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  std::scoped_lock lock(mtx_config_);

  drivers::OusterSensorConfiguration new_cfg(*sensor_cfg_ptr_);
  std::string _return_mode;

  bool got_any =
    get_param(p, "return_mode", _return_mode) |
    get_param(p, "frame_id", new_cfg.frame_id) |
    get_param(p, "scan_phase", new_cfg.scan_phase) |
    get_param(p, "min_range", new_cfg.min_range) |
    get_param(p, "max_range", new_cfg.max_range);

  if (!got_any) {
    return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
  }

  if (!_return_mode.empty()) {
    new_cfg.return_mode = drivers::return_mode_from_string_ouster(_return_mode);
  }

  auto new_cfg_ptr =
    std::make_shared<const nebula::drivers::OusterSensorConfiguration>(new_cfg);
  auto status = validate_and_set_config(new_cfg_ptr);

  if (status != Status::OK) {
    RCLCPP_WARN_STREAM(get_logger(), "OnParameterChange aborted: " << status);
    auto result = SetParametersResult();
    result.successful = false;
    result.reason = "Invalid configuration: " + util::to_string(status);
    return result;
  }

  return rcl_interfaces::build<SetParametersResult>().successful(true).reason("");
}

void OusterRosWrapper::receive_cloud_packet_callback(std::vector<uint8_t> & packet)
{
  if (!decoder_wrapper_ || decoder_wrapper_->status() != Status::OK) {
    return;
  }

  const auto now = std::chrono::high_resolution_clock::now();
  const auto timestamp_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

  auto msg_ptr = std::make_unique<nebula_msgs::msg::NebulaPacket>();
  msg_ptr->stamp.sec = static_cast<int>(timestamp_ns / 1'000'000'000);
  msg_ptr->stamp.nanosec = static_cast<int>(timestamp_ns % 1'000'000'000);
  msg_ptr->data.swap(packet);

  decoder_wrapper_->process_cloud_packet(std::move(msg_ptr));
}

RCLCPP_COMPONENTS_REGISTER_NODE(OusterRosWrapper)
}  // namespace nebula::ros
