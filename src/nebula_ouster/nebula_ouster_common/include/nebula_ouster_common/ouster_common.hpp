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

#include <nlohmann/json.hpp>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace nebula
{
namespace drivers
{

/// @brief Returns the number of channels (pixels per column) for a given Ouster sensor model
/// @param model Sensor model enum
/// @return Number of channels, or 0 if unknown
inline uint16_t ouster_pixels_per_column(SensorModel model)
{
  switch (model) {
    case SensorModel::OUSTER_OS0_32:
    case SensorModel::OUSTER_OS1_32:
    case SensorModel::OUSTER_OS2_32:
      return 32;
    case SensorModel::OUSTER_OS0_64:
    case SensorModel::OUSTER_OS1_64:
    case SensorModel::OUSTER_OS2_64:
      return 64;
    case SensorModel::OUSTER_OS0_128:
    case SensorModel::OUSTER_OS1_128:
    case SensorModel::OUSTER_OS2_128:
      return 128;
    default:
      return 0;
  }
}

/// @brief Returns true if the model is an Ouster sensor
inline bool is_ouster_model(SensorModel model)
{
  return ouster_pixels_per_column(model) != 0;
}

/// @brief Struct for Ouster sensor configuration
struct OusterSensorConfiguration : LidarConfigurationBase
{
  uint16_t imu_port{7503};
  uint16_t columns_per_frame{1024};  ///< Horizontal resolution: 512, 1024, or 2048
  double scan_phase{0.0};            ///< Start angle of a scan (degrees)
};

/// @brief Convert OusterSensorConfiguration to string (Overloading the << operator)
inline std::ostream & operator<<(std::ostream & os, OusterSensorConfiguration const & arg)
{
  os << "Ouster Sensor Configuration:" << '\n';
  os << static_cast<const LidarConfigurationBase &>(arg) << '\n';
  os << "IMU Port: " << arg.imu_port << '\n';
  os << "Columns per Frame: " << arg.columns_per_frame << '\n';
  os << "Scan Phase: " << arg.scan_phase;
  return os;
}

/// @brief Convert return mode name to ReturnMode enum for Ouster
inline ReturnMode return_mode_from_string_ouster(const std::string & return_mode)
{
  if (return_mode == "SingleStrongest" || return_mode == "Strongest" || return_mode == "STRONGEST")
    return ReturnMode::SINGLE_STRONGEST;
  if (return_mode == "SingleLast" || return_mode == "Last" || return_mode == "LAST")
    return ReturnMode::SINGLE_LAST;
  if (return_mode == "Dual" || return_mode == "DualReturn" || return_mode == "DUAL")
    return ReturnMode::DUAL;
  return ReturnMode::UNKNOWN;
}

/// @brief Struct for Ouster calibration configuration
/// Ouster sensors provide beam intrinsics via JSON metadata
struct OusterCalibrationConfiguration : CalibrationConfigurationBase
{
  std::vector<double> beam_altitude_angles;          ///< Per-channel elevation angles (degrees)
  std::vector<double> beam_azimuth_angles;           ///< Per-channel azimuth offsets (degrees)
  double lidar_origin_to_beam_origin_mm{27.67};      ///< Beam offset from lidar origin (mm)
  std::vector<int> pixel_shift_by_row;               ///< Per-channel column shift for de-staggering

  /// @brief Returns the number of channels derived from beam angle arrays
  [[nodiscard]] uint16_t get_pixels_per_column() const
  {
    return static_cast<uint16_t>(beam_altitude_angles.size());
  }

  /// @brief Load calibration from a JSON string (Ouster sensor_info format)
  nebula::Status load_from_string(const std::string & content)
  {
    try {
      auto j = nlohmann::json::parse(content);

      if (j.contains("beam_altitude_angles")) {
        beam_altitude_angles = j["beam_altitude_angles"].get<std::vector<double>>();
      }
      if (j.contains("beam_azimuth_angles")) {
        beam_azimuth_angles = j["beam_azimuth_angles"].get<std::vector<double>>();
      }
      if (j.contains("lidar_origin_to_beam_origin_mm")) {
        lidar_origin_to_beam_origin_mm =
          j["lidar_origin_to_beam_origin_mm"].get<double>();
      }
      if (j.contains("pixel_shift_by_row")) {
        pixel_shift_by_row = j["pixel_shift_by_row"].get<std::vector<int>>();
      }

      if (beam_altitude_angles.empty() || beam_azimuth_angles.empty()) {
        return Status::INVALID_CALIBRATION_FILE;
      }
      if (beam_altitude_angles.size() != beam_azimuth_angles.size()) {
        return Status::INVALID_CALIBRATION_FILE;
      }
    } catch (const std::exception & e) {
      std::cerr << "Failed to parse Ouster calibration JSON: " << e.what() << std::endl;
      return Status::INVALID_CALIBRATION_FILE;
    }
    return Status::OK;
  }

  /// @brief Load calibration from a JSON file
  nebula::Status load_from_file(const std::string & calibration_file)
  {
    std::ifstream ifs(calibration_file);
    if (!ifs) {
      return Status::INVALID_CALIBRATION_FILE;
    }
    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());
    ifs.close();
    return load_from_string(content);
  }

  /// @brief Save calibration to a JSON file
  nebula::Status save_file(const std::string & calibration_file) const
  {
    std::ofstream ofs(calibration_file);
    if (!ofs) {
      return Status::CANNOT_SAVE_FILE;
    }
    nlohmann::json j;
    j["beam_altitude_angles"] = beam_altitude_angles;
    j["beam_azimuth_angles"] = beam_azimuth_angles;
    j["lidar_origin_to_beam_origin_mm"] = lidar_origin_to_beam_origin_mm;
    j["pixel_shift_by_row"] = pixel_shift_by_row;
    ofs << j.dump(2) << std::endl;
    ofs.close();
    return Status::OK;
  }

  /// @brief Generate default calibration for a given sensor model (approximate values)
  /// @param model Sensor model
  void set_default_for_model(SensorModel model)
  {
    const uint16_t n = ouster_pixels_per_column(model);
    if (n == 0) return;

    beam_altitude_angles.resize(n);
    beam_azimuth_angles.resize(n);
    pixel_shift_by_row.resize(n, 0);

    // Generate approximate uniform altitude angles
    // OS1-64: range roughly -22.5 to +22.5 degrees
    // OS1-32: range roughly -22.5 to +22.5 degrees
    // OS1-128: range roughly -22.5 to +22.5 degrees
    double fov_degrees = 45.0;
    double step = fov_degrees / static_cast<double>(n - 1);
    double start = fov_degrees / 2.0;
    for (uint16_t i = 0; i < n; ++i) {
      beam_altitude_angles[i] = start - i * step;
      beam_azimuth_angles[i] = 0.0;
    }
    lidar_origin_to_beam_origin_mm = 27.67;
  }
};

}  // namespace drivers
}  // namespace nebula
