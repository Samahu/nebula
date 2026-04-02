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

#include "nebula_ouster/hw_interface_wrapper.hpp"

#include "nebula_core_ros/rclcpp_logger.hpp"

#include <nebula_core_common/util/string_conversions.hpp>

namespace nebula::ros
{

OusterHwInterfaceWrapper::OusterHwInterfaceWrapper(
  rclcpp::Node * const parent_node,
  std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & config)
: hw_interface_(
    std::make_shared<drivers::OusterHwInterface>(
      drivers::loggers::RclcppLogger(parent_node->get_logger()).child("HwInterface"))),
  logger_(parent_node->get_logger().get_child("HwInterfaceWrapper")),
  status_(nebula::drivers::Status::NOT_INITIALIZED)
{
  status_ = hw_interface_->set_sensor_configuration(config);
  if (drivers::Status::OK != status_) {
    throw std::runtime_error("Sensor configuration invalid: " + util::to_string(status_));
  }
}

void OusterHwInterfaceWrapper::on_config_change(
  const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & new_config)
{
  hw_interface_->set_sensor_configuration(new_config);
}

nebula::drivers::Status OusterHwInterfaceWrapper::status()
{
  return status_;
}

std::shared_ptr<drivers::OusterHwInterface> OusterHwInterfaceWrapper::hw_interface() const
{
  return hw_interface_;
}

}  // namespace nebula::ros
