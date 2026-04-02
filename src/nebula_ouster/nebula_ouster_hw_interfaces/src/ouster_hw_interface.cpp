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

#include "nebula_ouster_hw_interfaces/ouster_hw_interface.hpp"

#include <nebula_core_common/util/string_conversions.hpp>

namespace nebula::drivers
{

OusterHwInterface::OusterHwInterface(const std::shared_ptr<loggers::Logger> & logger)
: logger_(logger)
{
}

void OusterHwInterface::receive_sensor_packet_callback(std::vector<uint8_t> & buffer)
{
  if (scan_reception_callback_) {
    scan_reception_callback_(buffer);
  }
}

Status OusterHwInterface::sensor_interface_start()
{
  if (!sensor_configuration_) {
    return Status::SENSOR_CONFIG_ERROR;
  }

  try {
    logger_->info(
      "Starting UDP server for Ouster data on: " + sensor_configuration_->sensor_ip + ":" +
      std::to_string(sensor_configuration_->data_port));

    cloud_udp_socket_ = std::make_unique<connections::UdpSocket>(
      connections::UdpSocket::Builder(
        sensor_configuration_->host_ip, sensor_configuration_->data_port)
        .limit_to_sender(sensor_configuration_->sensor_ip, sensor_configuration_->data_port)
        .bind());

    cloud_udp_socket_->subscribe(
      [this](std::vector<uint8_t> & buffer, const connections::UdpSocket::RxMetadata &) {
        this->receive_sensor_packet_callback(buffer);
      });
  } catch (const std::exception & ex) {
    Status status = Status::UDP_CONNECTION_ERROR;
    logger_->error(
      util::to_string(status) + " " + sensor_configuration_->sensor_ip + "," +
      std::to_string(sensor_configuration_->data_port) + ": " + ex.what());
    return status;
  }
  return Status::OK;
}

Status OusterHwInterface::sensor_interface_stop()
{
  cloud_udp_socket_.reset();
  return Status::OK;
}

Status OusterHwInterface::set_sensor_configuration(
  std::shared_ptr<const OusterSensorConfiguration> sensor_configuration)
{
  if (!sensor_configuration) {
    return Status::SENSOR_CONFIG_ERROR;
  }
  if (!is_ouster_model(sensor_configuration->sensor_model)) {
    return Status::INVALID_SENSOR_MODEL;
  }
  sensor_configuration_ = sensor_configuration;
  return Status::OK;
}

Status OusterHwInterface::register_scan_callback(
  std::function<void(std::vector<uint8_t> &)> scan_callback)
{
  scan_reception_callback_ = std::move(scan_callback);
  return Status::OK;
}

}  // namespace nebula::drivers
