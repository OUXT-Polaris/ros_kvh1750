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
 * Interfacing with KVH 1750 over RSS-422.
 * \author Jason Ziglar <jpz@vt.edu>, based on code form Eric L. Hahn <erichahn@vt.edu>
 * \date 02/15/2015
 * Copyright 2015, Virginia Tech. All Rights Reserved.
 */
#ifndef __KVH_1750_IMU_h__
#define __KVH_1750_IMU_h__

#include <serial/serial.h>

#include <chrono>
#include <memory>
#include <vector>

#include "imu.h"

namespace kvh
{
/**
 * Interface to a KVH1750 over RS-422, with support for the TOV signal
 * via a file descriptor.
 */
class TOVFile : public IOModule
{
public:
  TOVFile(
    const std::string & addr = "/dev/ttyS4", uint32_t baud = 921600, uint32_t tm = 1,
    const std::string & tov_addr = "");
  virtual ~TOVFile();

  virtual bool read(uint8_t * buff, size_t max_bytes, size_t & bytes, bool tov);
  virtual bool write(const uint8_t * buff, size_t bytes);
  virtual void flush_buffers();
  virtual void time(uint32_t & secs, uint32_t & nsecs);
  virtual void reset_time();

protected:
  typedef std::chrono::high_resolution_clock::duration duration_t;
  std::shared_ptr<serial::Serial> _data;
  std::shared_ptr<serial::Serial> _tov;
  duration_t _tm;  //! Time when message was read
  bool _valid_tm;  //! Flag indicating if time is valid
};

}  // namespace kvh

#endif