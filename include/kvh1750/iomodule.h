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
 * Module defining IO interface to an IMU.
 * \author: Jason Ziglar <jpz@vt.edu>
 * \date: 02/22/2015
 */
#ifndef _IOMODULE_h_
#define _IOMODULE_h_

#include <stdint.h>

#include <cstring>

namespace kvh
{
/**
 * Interface to IO for reading IMU data.
 */
class IOModule
{
public:
  IOModule();
  virtual ~IOModule();

  virtual bool read(uint8_t * buff, size_t max_bytes, size_t & bytes, bool tov = true) = 0;
  virtual bool write(const uint8_t * buff, size_t bytes) = 0;
  virtual void flush_buffers() = 0;
  virtual void time(uint32_t & secs, uint32_t & nsecs) = 0;
  virtual void reset_time() = 0;
};

}  // namespace kvh

#endif
