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

#include "nebula_ouster/decoder_wrapper.hpp"
#include "nebula_ouster/hw_interface_wrapper.hpp"
#include "nebula_ouster/hw_monitor_wrapper.hpp"

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_core_common/nebula_status.hpp>
#include <nebula_ouster_common/ouster_common.hpp>
#include <nebula_ouster_hw_interfaces/ouster_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <nebula_msgs/msg/nebula_packet.hpp>
#include <ouster_msgs/msg/ouster_scan.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <vector>

namespace nebula::ros
{

/// @brief ROS 2 component node for Ouster LiDAR
class OusterRosWrapper final : public rclcpp::Node
{
public:
  explicit OusterRosWrapper(const rclcpp::NodeOptions & options);
  ~OusterRosWrapper() noexcept = default;

  drivers::Status get_status();
  drivers::Status stream_start();

private:
  void receive_cloud_packet_callback(std::vector<uint8_t> & packet);
  void receive_scan_message_callback(std::unique_ptr<ouster_msgs::msg::OusterScan> scan_msg);

  nebula::Status declare_and_get_sensor_config_params();

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & p);

  nebula::Status validate_and_set_config(
    std::shared_ptr<const drivers::OusterSensorConfiguration> & new_config);

  nebula::drivers::Status wrapper_status_;

  std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> sensor_cfg_ptr_;
  std::shared_ptr<const nebula::drivers::OusterCalibrationConfiguration> calibration_cfg_ptr_;

  rclcpp::Subscription<ouster_msgs::msg::OusterScan>::SharedPtr packets_sub_;
  bool launch_hw_;

  std::optional<OusterHwInterfaceWrapper> hw_interface_wrapper_;
  std::optional<OusterHwMonitorWrapper> hw_monitor_wrapper_;
  std::optional<OusterDecoderWrapper> decoder_wrapper_;

  std::mutex mtx_config_;
  OnSetParametersCallbackHandle::SharedPtr parameter_event_cb_;
};

}  // namespace nebula::ros
