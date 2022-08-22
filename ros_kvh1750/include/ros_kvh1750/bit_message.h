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
 * Built In Test Message from KVH 1750
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/21/2015
 */

#ifndef _KVH1750_BIT_MESSAGE_H_
#define _KVH1750_BIT_MESSAGE_H_

#include <cstring>
#include <vector>

namespace kvh
{
//! Number of status bytes in the message
const size_t NumStatusBytes = 6;
//! Number of bits used in each byte
const size_t BitsUsedPerByte = 7;

#pragma pack(push, 1)

/**
 * Raw format of the KVH Built In Test message, used for
 * accessing straight from a byte array.
 */
struct BITRawMessage
{
  char header[4];
  char status[NumStatusBytes];
  char checksum;
};

#pragma pack(pop)

/**
 * Higher level form of BIT message
 */
class BITMessage
{
public:
  BITMessage();
  BITMessage(const BITRawMessage & raw);
  ~BITMessage();

  bool from_raw(const BITRawMessage & raw);

  //gyro
  bool gyro_x_sld() const;
  bool gyro_x_moddac() const;
  bool gyro_x_phase() const;
  bool gyro_x_flash() const;
  bool gyro_x_pzt_temp() const;
  bool gyro_x_sld_temp() const;
  bool gyro_y_sld() const;
  bool gyro_y_moddac() const;
  bool gyro_y_phase() const;
  bool gyro_y_flash() const;
  bool gyro_y_pzt_temp() const;
  bool gyro_y_sld_temp() const;
  bool gyro_z_sld() const;
  bool gyro_z_moddac() const;
  bool gyro_z_phase() const;
  bool gyro_z_flash() const;
  bool gyro_z_pzt_temp() const;
  bool gyro_z_sld_temp() const;

  //accelerometers
  bool accel_x() const;
  bool accel_x_temp() const;
  bool accel_y() const;
  bool accel_y_temp() const;
  bool accel_z() const;
  bool accel_z_temp() const;

  //other
  bool gcb_temp() const;
  bool imu_temp() const;
  bool gcb_dsp_flash() const;
  bool gcb_fpga_flash() const;
  bool imu_dsp_flash() const;
  bool imu_fpga_flash() const;
  bool gcb_1_2() const;
  bool gcb_3_3() const;
  bool gcb_5() const;
  bool imu_1_2() const;
  bool imu_3_3() const;
  bool imu_5() const;
  bool imu_15() const;
  bool gcb_fpga() const;
  bool imu_fpga() const;
  bool hispeed_sport() const;
  bool aux_sport() const;
  bool software() const;

private:
  std::vector<bool> _status;
};

}  // namespace kvh

#endif