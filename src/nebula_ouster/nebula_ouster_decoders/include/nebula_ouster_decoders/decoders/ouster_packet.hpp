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

#include <cstdint>
#include <cstring>

namespace nebula::drivers::ouster_packet
{

/// @brief Number of measurement columns per UDP packet (Ouster legacy format)
static constexpr uint16_t COLUMNS_PER_PACKET = 16;
/// @brief Ouster encoder max value (360 degrees = 90112 counts)
static constexpr uint32_t ENCODER_TICKS_PER_REV = 90112;
/// @brief Column status indicating a valid measurement
static constexpr uint32_t COLUMN_VALID = 0xFFFFFFFF;

#pragma pack(push, 1)

/// @brief Per-channel measurement data (12 bytes)
struct ChannelData
{
  uint32_t range_mm;      ///< Distance in millimeters (0 = no return)
  uint16_t reflectivity;  ///< Calibrated reflectivity
  uint16_t signal;        ///< Signal photon count
  uint16_t near_ir;       ///< Near-IR ambient photon count
  uint16_t unused;
};

static_assert(sizeof(ChannelData) == 12, "ChannelData must be 12 bytes");

/// @brief Column (measurement block) header (16 bytes)
struct ColumnHeader
{
  uint64_t timestamp_ns;   ///< Timestamp in nanoseconds
  uint16_t measurement_id; ///< Column index within the frame
  uint16_t frame_id;       ///< Frame counter (wraps at 65535)
  uint32_t encoder_count;  ///< Encoder position (0..ENCODER_TICKS_PER_REV-1)
};

static_assert(sizeof(ColumnHeader) == 16, "ColumnHeader must be 16 bytes");

/// @brief Column footer (4 bytes)
struct ColumnFooter
{
  uint32_t status; ///< 0xFFFFFFFF = valid, all other values = invalid
};

static_assert(sizeof(ColumnFooter) == 4, "ColumnFooter must be 4 bytes");

#pragma pack(pop)

/// @brief Compute expected UDP packet size for a given channel count
inline size_t packet_size(uint16_t pixels_per_column)
{
  const size_t col_size =
    sizeof(ColumnHeader) + pixels_per_column * sizeof(ChannelData) + sizeof(ColumnFooter);
  return COLUMNS_PER_PACKET * col_size;
}

/// @brief Get pointer to a column header within a raw packet buffer
inline const ColumnHeader * column_header(
  const uint8_t * buf, uint16_t col_idx, uint16_t pixels_per_column)
{
  const size_t col_size =
    sizeof(ColumnHeader) + pixels_per_column * sizeof(ChannelData) + sizeof(ColumnFooter);
  return reinterpret_cast<const ColumnHeader *>(buf + col_idx * col_size);
}

/// @brief Get pointer to channel data within a column
inline const ChannelData * channel_data(const ColumnHeader * hdr, uint16_t channel_idx)
{
  const uint8_t * base = reinterpret_cast<const uint8_t *>(hdr) + sizeof(ColumnHeader);
  return reinterpret_cast<const ChannelData *>(base + channel_idx * sizeof(ChannelData));
}

/// @brief Get the status word of a column
inline uint32_t column_status(const ColumnHeader * hdr, uint16_t pixels_per_column)
{
  const uint8_t * base = reinterpret_cast<const uint8_t *>(hdr) + sizeof(ColumnHeader) +
                          pixels_per_column * sizeof(ChannelData);
  return reinterpret_cast<const ColumnFooter *>(base)->status;
}

}  // namespace nebula::drivers::ouster_packet
