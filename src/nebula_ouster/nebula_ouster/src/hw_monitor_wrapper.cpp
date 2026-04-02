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

#include "nebula_ouster/hw_monitor_wrapper.hpp"

#include "nebula_core_ros/parameter_descriptors.hpp"

#include <nebula_core_common/util/string_conversions.hpp>

namespace nebula::ros
{

OusterHwMonitorWrapper::OusterHwMonitorWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & config)
: parent_(parent_node),
  logger_(parent_->get_logger().get_child("HwMonitorWrapper")),
  diagnostics_updater_(parent_node),
  sensor_cfg_(config)
{
  auto descriptor = param_read_only().set__additional_constraints("milliseconds");
  diag_span_ = parent_->declare_parameter<uint16_t>("diag_span", descriptor);

  auto hw_id =
    nebula::drivers::sensor_model_to_string(sensor_cfg_->sensor_model) + ": " +
    sensor_cfg_->sensor_ip;
  diagnostics_updater_.setHardwareID(hw_id);
  diagnostics_updater_.add("ouster_status", this, &OusterHwMonitorWrapper::check_status);

  diagnostics_update_timer_ = parent_->create_wall_timer(
    std::chrono::milliseconds(diag_span_),
    [this] { diagnostics_updater_.force_update(); });
}

void OusterHwMonitorWrapper::check_status(
  diagnostic_updater::DiagnosticStatusWrapper & diagnostics)
{
  std::lock_guard lock(mtx_config_);
  diagnostics.add(
    "sensor_model",
    nebula::drivers::sensor_model_to_string(sensor_cfg_->sensor_model));
  diagnostics.add("sensor_ip", sensor_cfg_->sensor_ip);
  diagnostics.add("host_ip", sensor_cfg_->host_ip);
  diagnostics.add("data_port", std::to_string(sensor_cfg_->data_port));
  diagnostics.add("imu_port", std::to_string(sensor_cfg_->imu_port));
  diagnostics.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
}

void OusterHwMonitorWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & new_config)
{
  std::lock_guard lock(mtx_config_);
  if (!new_config) {
    throw std::invalid_argument("Config is not nullable");
  }
  if (new_config->sensor_model != sensor_cfg_->sensor_model) {
    throw std::invalid_argument("Sensor model is read-only during runtime");
  }
  sensor_cfg_ = new_config;
}

}  // namespace nebula::ros
