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

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <nebula_core_common/nebula_common.hpp>
#include <nebula_ouster_common/ouster_common.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>

namespace nebula::ros
{

/// @brief Hardware monitor wrapper for Ouster driver
class OusterHwMonitorWrapper
{
public:
  explicit OusterHwMonitorWrapper(
    rclcpp::Node * const parent_node,
    std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & config);

  void on_config_change(
    const std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> & new_config);

private:
  void check_status(diagnostic_updater::DiagnosticStatusWrapper & diagnostics);

  rclcpp::Node * parent_;
  rclcpp::Logger logger_;
  diagnostic_updater::Updater diagnostics_updater_;
  std::shared_ptr<const nebula::drivers::OusterSensorConfiguration> sensor_cfg_;
  std::mutex mtx_config_;
  rclcpp::TimerBase::SharedPtr diagnostics_update_timer_;
  uint16_t diag_span_{1000};
};

}  // namespace nebula::ros
