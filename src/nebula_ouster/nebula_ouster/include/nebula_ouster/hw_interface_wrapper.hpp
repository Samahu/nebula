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

#include <nebula_core_common/nebula_common.hpp>
#include <nebula_ouster_common/ouster_common.hpp>
#include <nebula_ouster_hw_interfaces/ouster_hw_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace nebula::ros
{

class OusterHwInterfaceWrapper
{
public:
  explicit OusterHwInterfaceWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & new_config);

  nebula::drivers::Status status();

  std::shared_ptr<drivers::OusterHwInterface> hw_interface() const;

private:
  std::shared_ptr<drivers::OusterHwInterface> hw_interface_;
  rclcpp::Logger logger_;
  nebula::drivers::Status status_;
};

}  // namespace nebula::ros
