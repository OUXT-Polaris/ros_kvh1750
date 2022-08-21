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
 * Base interface for KVH IMU
 * \author Jason Ziglar <jpz@vt.edu>
 * \date 02/15/2015
 * Copyright 2015, Virginia Tech. All Rights Reserved.
 */
#ifndef _KVH_1750_IMU_BASE_h_
#define _KVH_1750_IMU_BASE_h_

#include <memory>

#include "kvh1750/types.h"

namespace kvh
{
/**
 * Base class for handling logic of interfacing with the KVH IMU. Delegates
 * actual reading to a derived class, so that alternate interfaces can be
 * considered.
 */
class IMU1750
{
public:
  /**
   * Reasons why a processing attempt might fail
   */
  typedef enum { VALID = 0, BAD_READ, FATAL_ERROR, BAD_CRC, PARTIAL_READ, OVER_TEMP } ParseResults;
  static const std::vector<int> DataRates;  //! Supported IMU Data Rates
public:
  IMU1750(std::shared_ptr<IOModule> mod);
  ~IMU1750();

  void set_temp_limit(int max_temp_c);

  ParseResults read(kvh::Message & msg);

  int imu_rate() const;
  bool is_celsius() const;
  bool is_delta_angle() const;
  bool is_configuring() const;
  bool query_temp_units(bool & is_celsius);
  bool query_angle_units(bool & is_da);
  bool query_data_rate(int & rate_hz);
  bool set_temp_units(bool is_celius);
  bool set_data_rate(int rate_hz);
  bool set_angle_units(bool is_da);
  //TODO: Add configuration commands
protected:
  void set_mode(bool is_config);
  bool parse_angle_units(std::vector<uint8_t>::iterator match);
  bool parse_temp_units(std::vector<uint8_t>::iterator match);
  bool parse_data_rate(std::vector<uint8_t>::iterator match, int & rate);
  bool base_read(bool use_tov = true);
  bool cmd_read();
  bool cmd_write(const std::string & cmd);
  bool find_header(bool is_imu, std::vector<uint8_t>::iterator & match);
  void reset_buffer();
  void reset_partial_buffer(const std::vector<uint8_t>::iterator & match);
  size_t bytes_remaining() const;
  void set_buffer_size(size_t len);

  std::string build_command(const std::string & type, const std::string & val, bool is_query);
  bool find_response(const std::string & type, std::vector<uint8_t>::iterator & match);

protected:
  std::shared_ptr<IOModule> _io;      //! IO interface
  std::vector<uint8_t> _buff;         //! Internal buffer
  std::vector<uint8_t> _backup_buff;  //! Backup buffer for swapping
  size_t _bytes_read;                 //! Number of bytes read into the buffer
  int _rate;                          //! Last read date rate of IMU
  int _max_temp;                      //! Max temperature of IMU allowed
  bool _is_config;                    //! In config mode
  bool _is_c;                         //! Last read temperature units (true if Celsius)
  bool _is_da;                        //! Last read angle units (true if delta angle)
};

}  // namespace kvh

#endif