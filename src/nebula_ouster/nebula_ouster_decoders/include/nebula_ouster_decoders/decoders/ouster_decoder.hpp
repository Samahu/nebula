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

#include "nebula_ouster_common/ouster_common.hpp"
#include "nebula_ouster_decoders/decoders/ouster_packet.hpp"
#include "nebula_ouster_decoders/decoders/ouster_scan_decoder.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

namespace nebula::drivers
{

/// @brief Ouster lidar scan decoder supporting all OS0/OS1/OS2 models
class OusterDecoder : public OusterScanDecoder
{
public:
  OusterDecoder(
    const std::shared_ptr<const OusterSensorConfiguration> & sensor_cfg,
    const std::shared_ptr<const OusterCalibrationConfiguration> & calib)
  : sensor_cfg_(sensor_cfg), calib_(calib), logger_(rclcpp::get_logger("OusterDecoder"))
  {
    const auto n = calib_->get_pixels_per_column();

    sin_alt_.resize(n);
    cos_alt_.resize(n);
    az_offset_rad_.resize(n);

    for (uint16_t i = 0; i < n; ++i) {
      const double alt_rad = deg2rad(calib_->beam_altitude_angles[i]);
      sin_alt_[i] = std::sin(alt_rad);
      cos_alt_[i] = std::cos(alt_rad);
      az_offset_rad_[i] = deg2rad(calib_->beam_azimuth_angles[i]);
    }
    beam_offset_m_ = calib_->lidar_origin_to_beam_origin_mm / 1000.0;

    decode_pc_ = std::make_shared<NebulaPointCloud>();
    output_pc_ = std::make_shared<NebulaPointCloud>();
    decode_pc_->reserve(n * sensor_cfg_->columns_per_frame);
  }

  int unpack(const std::vector<uint8_t> & packet) override
  {
    const uint16_t n = calib_->get_pixels_per_column();
    const size_t expected = ouster_packet::packet_size(n);
    if (packet.size() < expected) {
      RCLCPP_ERROR_STREAM(
        logger_, "Packet too small: " << packet.size() << " < " << expected);
      return -1;
    }

    const uint8_t * buf = packet.data();
    int last_mid = -1;

    for (uint16_t col = 0; col < ouster_packet::COLUMNS_PER_PACKET; ++col) {
      const auto * hdr = ouster_packet::column_header(buf, col, n);

      if (ouster_packet::column_status(hdr, n) != ouster_packet::COLUMN_VALID) {
        continue;
      }

      const int fid = static_cast<int>(hdr->frame_id);

      // Detect frame boundary: when frame_id changes, emit the completed scan
      if (last_frame_id_ >= 0 && fid != last_frame_id_) {
        output_pc_ = decode_pc_;
        output_ts_ns_ = decode_start_ns_;
        decode_pc_ = std::make_shared<NebulaPointCloud>();
        decode_pc_->reserve(n * sensor_cfg_->columns_per_frame);
        has_scanned_ = true;
        decode_start_ns_ = hdr->timestamp_ns;
      }

      last_frame_id_ = fid;
      if (decode_pc_->empty()) {
        decode_start_ns_ = hdr->timestamp_ns;
      }

      // Azimuth from encoder (0..ENCODER_TICKS_PER_REV → 0..2π)
      const double enc_rad =
        2.0 * M_PI *
        static_cast<double>(hdr->encoder_count) /
        static_cast<double>(ouster_packet::ENCODER_TICKS_PER_REV);

      for (uint16_t ch = 0; ch < n; ++ch) {
        const auto * cd = ouster_packet::channel_data(hdr, ch);
        if (cd->range_mm == 0) continue;

        const double range_m = static_cast<double>(cd->range_mm) / 1000.0;
        if (range_m < sensor_cfg_->min_range || range_m > sensor_cfg_->max_range) continue;

        const double az_rad = enc_rad + az_offset_rad_[ch];
        const double cos_az = std::cos(az_rad);
        const double sin_az = std::sin(az_rad);
        const double r_cos_alt = range_m * cos_alt_[ch];

        NebulaPoint p;
        p.x = static_cast<float>(r_cos_alt * cos_az + beam_offset_m_ * cos_az);
        p.y = static_cast<float>(-(r_cos_alt * sin_az + beam_offset_m_ * sin_az));
        p.z = static_cast<float>(range_m * sin_alt_[ch]);
        p.intensity = static_cast<uint8_t>(std::min<uint16_t>(255u, cd->reflectivity));
        p.channel = ch;
        p.azimuth = static_cast<float>(az_rad);
        p.elevation = static_cast<float>(deg2rad(calib_->beam_altitude_angles[ch]));
        p.distance = static_cast<float>(range_m);
        p.return_type = static_cast<uint8_t>(ReturnType::STRONGEST);
        p.time_stamp = (hdr->timestamp_ns >= decode_start_ns_)
          ? static_cast<uint32_t>(hdr->timestamp_ns - decode_start_ns_)
          : 0u;

        decode_pc_->emplace_back(p);
      }

      last_mid = static_cast<int>(hdr->measurement_id);
    }

    return last_mid;
  }

  bool has_scanned() override { return has_scanned_; }

  std::tuple<NebulaPointCloudPtr, double> get_pointcloud() override
  {
    has_scanned_ = false;
    return {output_pc_, static_cast<double>(output_ts_ns_) * 1e-9};
  }

private:
  std::shared_ptr<const OusterSensorConfiguration> sensor_cfg_;
  std::shared_ptr<const OusterCalibrationConfiguration> calib_;
  rclcpp::Logger logger_;

  std::vector<double> sin_alt_;
  std::vector<double> cos_alt_;
  std::vector<double> az_offset_rad_;
  double beam_offset_m_{0.0};

  NebulaPointCloudPtr decode_pc_;
  NebulaPointCloudPtr output_pc_;

  uint64_t decode_start_ns_{0};
  uint64_t output_ts_ns_{0};
  int last_frame_id_{-1};
  bool has_scanned_{false};
};

}  // namespace nebula::drivers
