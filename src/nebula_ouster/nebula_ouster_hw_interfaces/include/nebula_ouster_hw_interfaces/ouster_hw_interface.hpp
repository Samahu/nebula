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

#include "nebula_core_hw_interfaces/connections/udp.hpp"

#include <nebula_core_common/loggers/logger.hpp>
#include <nebula_ouster_common/ouster_common.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace nebula::drivers
{

/// @brief Default Ouster lidar data UDP port
static constexpr uint16_t OUSTER_DEFAULT_DATA_PORT = 7502;
/// @brief Default Ouster IMU UDP port
static constexpr uint16_t OUSTER_DEFAULT_IMU_PORT = 7503;

/// @brief Hardware interface for Ouster LiDARs
class OusterHwInterface
{
public:
  explicit OusterHwInterface(const std::shared_ptr<loggers::Logger> & logger);

  /// @brief Callback invoked when a lidar data UDP packet is received
  void receive_sensor_packet_callback(std::vector<uint8_t> & buffer);

  /// @brief Start listening for lidar data UDP packets
  Status sensor_interface_start();

  /// @brief Stop the UDP listener
  Status sensor_interface_stop();

  /// @brief Set sensor configuration (must be called before sensor_interface_start)
  Status set_sensor_configuration(
    std::shared_ptr<const OusterSensorConfiguration> sensor_configuration);

  /// @brief Register a callback for received lidar data packets
  Status register_scan_callback(std::function<void(std::vector<uint8_t> &)> scan_callback);

private:
  std::shared_ptr<loggers::Logger> logger_;
  std::unique_ptr<connections::UdpSocket> cloud_udp_socket_;
  std::shared_ptr<const OusterSensorConfiguration> sensor_configuration_;
  std::function<void(std::vector<uint8_t> &)> scan_reception_callback_;
};

}  // namespace nebula::drivers
