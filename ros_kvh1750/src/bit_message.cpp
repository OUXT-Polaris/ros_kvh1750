// Copyright 2015 Jason Ziglar.
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * Built In Test messages.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02-21-2015
 */
#include "ros_kvh1750/bit_message.h"

#include <cstdint>

namespace kvh
{
/**
 * Default Constructor
 */
BITMessage::BITMessage() : _status(NumStatusBytes * BitsUsedPerByte, false) {}

/**
 * Constructor from raw message. Tests checksum
 */
BITMessage::BITMessage(const BITRawMessage & raw) : _status(NumStatusBytes * BitsUsedPerByte, false)
{
  from_raw(raw);
}

/**
 * Default Destructor
 */
BITMessage::~BITMessage() {}

/**
 * Extract flags from raw message, and test checksum.
 * \param[out] Flag indicating if checksum was valid
 */
bool BITMessage::from_raw(const BITRawMessage & raw)
{
  uint8_t computed_checksum = raw.header[0] + raw.header[1] + raw.header[2] + raw.header[3];

  for (size_t ii = 0; ii < NumStatusBytes; ++ii) {
    for (size_t jj = 0; jj < BitsUsedPerByte; ++jj) {
      _status[ii * NumStatusBytes + jj] = raw.status[ii] & (1 << jj);
    }
    computed_checksum += raw.status[ii];
  }

  bool result = computed_checksum != raw.checksum;
  //bad message, reject data
  if (!result) {
    _status.clear();
    _status.resize(BitsUsedPerByte, false);
  }

  return result;
}

//The following commands are all status flags defined in the KVH
//manual
bool BITMessage::gyro_x_sld() const { return _status[0]; }

bool BITMessage::gyro_x_moddac() const { return _status[1]; }

bool BITMessage::gyro_x_phase() const { return _status[2]; }

bool BITMessage::gyro_x_flash() const { return _status[3]; }

bool BITMessage::gyro_x_pzt_temp() const { return _status[4]; }

bool BITMessage::gyro_x_sld_temp() const { return _status[5]; }

bool BITMessage::gyro_y_sld() const { return _status[6]; }

bool BITMessage::gyro_y_moddac() const { return _status[7]; }

bool BITMessage::gyro_y_phase() const { return _status[8]; }

bool BITMessage::gyro_y_flash() const { return _status[9]; }

bool BITMessage::gyro_y_pzt_temp() const { return _status[10]; }

bool BITMessage::gyro_y_sld_temp() const { return _status[11]; }

bool BITMessage::gyro_z_sld() const { return _status[12]; }

bool BITMessage::gyro_z_moddac() const { return _status[13]; }

bool BITMessage::gyro_z_phase() const { return _status[14]; }

bool BITMessage::gyro_z_flash() const { return _status[15]; }

bool BITMessage::gyro_z_pzt_temp() const { return _status[16]; }

bool BITMessage::gyro_z_sld_temp() const { return _status[17]; }

bool BITMessage::accel_x() const { return _status[18]; }

bool BITMessage::accel_x_temp() const { return _status[19]; }

bool BITMessage::accel_y() const { return _status[20]; }

bool BITMessage::accel_y_temp() const { return _status[21]; }

bool BITMessage::accel_z() const { return _status[22]; }

bool BITMessage::accel_z_temp() const { return _status[23]; }

bool BITMessage::gcb_temp() const { return _status[24]; }

bool BITMessage::imu_temp() const { return _status[25]; }

bool BITMessage::gcb_dsp_flash() const { return _status[26]; }

bool BITMessage::gcb_fpga_flash() const { return _status[27]; }

bool BITMessage::imu_dsp_flash() const { return _status[28]; }

bool BITMessage::imu_fpga_flash() const { return _status[29]; }

bool BITMessage::gcb_1_2() const { return _status[30]; }

bool BITMessage::gcb_3_3() const { return _status[31]; }

bool BITMessage::gcb_5() const { return _status[32]; }

bool BITMessage::imu_1_2() const { return _status[33]; }

bool BITMessage::imu_3_3() const { return _status[34]; }

bool BITMessage::imu_5() const { return _status[35]; }

bool BITMessage::imu_15() const { return _status[36]; }

bool BITMessage::gcb_fpga() const { return _status[37]; }

bool BITMessage::imu_fpga() const { return _status[38]; }

bool BITMessage::hispeed_sport() const { return _status[39]; }

bool BITMessage::aux_sport() const { return _status[40]; }

bool BITMessage::software() const { return _status[41]; }

}  // namespace kvh