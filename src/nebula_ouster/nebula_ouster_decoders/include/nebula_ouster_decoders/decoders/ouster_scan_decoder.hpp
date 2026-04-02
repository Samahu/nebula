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

#include "nebula_core_common/point_types.hpp"

#include <tuple>
#include <vector>

namespace nebula::drivers
{

/// @brief Base class for Ouster LiDAR scan decoder
class OusterScanDecoder
{
public:
  OusterScanDecoder(OusterScanDecoder &&) = delete;
  OusterScanDecoder & operator=(OusterScanDecoder &&) = delete;
  OusterScanDecoder(const OusterScanDecoder &) = delete;
  OusterScanDecoder & operator=(const OusterScanDecoder &) = delete;

  virtual ~OusterScanDecoder() = default;
  OusterScanDecoder() = default;

  /// @brief Parse a raw UDP lidar packet and accumulate points
  /// @param packet Raw packet bytes
  /// @return Last measurement_id processed, or -1 on error
  virtual int unpack(const std::vector<uint8_t> & packet) = 0;

  /// @brief Check if a full 360-degree scan is ready
  virtual bool has_scanned() = 0;

  /// @brief Retrieve the completed point cloud and its start timestamp
  /// @return Tuple of (point cloud, timestamp in seconds)
  virtual std::tuple<drivers::NebulaPointCloudPtr, double> get_pointcloud() = 0;
};

}  // namespace nebula::drivers
