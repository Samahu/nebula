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

#include "nebula_ouster_decoders/ouster_driver.hpp"

#include "nebula_ouster_decoders/decoders/ouster_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <stdexcept>

namespace nebula::drivers
{

OusterDriver::OusterDriver(
  const std::shared_ptr<const OusterSensorConfiguration> & sensor_cfg,
  const std::shared_ptr<const OusterCalibrationConfiguration> & calib)
{
  if (!is_ouster_model(sensor_cfg->sensor_model)) {
    driver_status_ = Status::INVALID_SENSOR_MODEL;
    throw std::runtime_error(
      "OusterDriver: unsupported sensor model " +
      sensor_model_to_string(sensor_cfg->sensor_model));
  }

  scan_decoder_ = std::make_shared<OusterDecoder>(sensor_cfg, calib);
  driver_status_ = Status::OK;
}

Status OusterDriver::get_status()
{
  return driver_status_;
}

Status OusterDriver::set_calibration_configuration(
  const CalibrationConfigurationBase & /*calibration_configuration*/)
{
  throw std::runtime_error("OusterDriver::set_calibration_configuration not implemented");
}

std::tuple<drivers::NebulaPointCloudPtr, double> OusterDriver::parse_cloud_packet(
  const std::vector<uint8_t> & packet)
{
  auto logger = rclcpp::get_logger("OusterDriver");

  if (driver_status_ != Status::OK) {
    RCLCPP_ERROR(logger, "Driver not OK.");
    return {nullptr, 0.0};
  }

  scan_decoder_->unpack(packet);
  if (scan_decoder_->has_scanned()) {
    return scan_decoder_->get_pointcloud();
  }
  return {nullptr, 0.0};
}

}  // namespace nebula::drivers
