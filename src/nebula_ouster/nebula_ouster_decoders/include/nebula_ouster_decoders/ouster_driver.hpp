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

#include "nebula_core_common/nebula_common.hpp"
#include "nebula_core_common/nebula_status.hpp"
#include "nebula_core_common/point_types.hpp"
#include "nebula_core_decoders/nebula_driver_base.hpp"
#include "nebula_ouster_common/ouster_common.hpp"
#include "nebula_ouster_decoders/decoders/ouster_scan_decoder.hpp"

#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{

/// @brief Driver for Ouster LiDAR sensors
class OusterDriver : NebulaDriverBase
{
public:
  OusterDriver() = delete;

  /// @brief Constructor
  explicit OusterDriver(
    const std::shared_ptr<const OusterSensorConfiguration> & sensor_cfg,
    const std::shared_ptr<const OusterCalibrationConfiguration> & calib);

  /// @brief Get current driver status
  Status get_status();

  /// @brief Set calibration configuration (not supported at runtime)
  Status set_calibration_configuration(
    const CalibrationConfigurationBase & calibration_configuration) override;

  /// @brief Parse a raw UDP packet. Returns a point cloud when a full scan is completed.
  /// @param packet Raw packet bytes
  /// @return Tuple of (point cloud or nullptr, timestamp in seconds)
  std::tuple<drivers::NebulaPointCloudPtr, double> parse_cloud_packet(
    const std::vector<uint8_t> & packet);

private:
  Status driver_status_{Status::NOT_INITIALIZED};
  std::shared_ptr<OusterScanDecoder> scan_decoder_;
};

}  // namespace nebula::drivers
