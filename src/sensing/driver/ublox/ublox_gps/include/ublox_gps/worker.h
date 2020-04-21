//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_GPS_WORKER_H
#define UBLOX_GPS_WORKER_H

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/function.hpp>

namespace ublox_gps {

/**
 * @brief Handles I/O reading and writing.
 */
class Worker {
 public:
  typedef boost::function<void(unsigned char*, std::size_t&)> Callback;
  virtual ~Worker() {}

  /**
   * @brief Set the callback function for received messages.
   * @param callback the callback function which process messages in the buffer
   */
  virtual void setCallback(const Callback& callback) = 0;

  /**
   * @brief Set the callback function which handles raw data.
   * @param callback the write callback which handles raw data
   */
  virtual void setRawDataCallback(const Callback& callback) = 0;

  /**
   * @brief Send the data in the buffer.
   * @param data the bytes to send
   * @param size the size of the buffer
   */
  virtual bool send(const unsigned char* data, const unsigned int size) = 0;

  /**
   * @brief Wait for an incoming message.
   * @param timeout the maximum time to wait.
   */
  virtual void wait(const boost::posix_time::time_duration& timeout) = 0;

  /**
   * @brief Whether or not the I/O stream is open.
   */
  virtual bool isOpen() const = 0;
};

}  // namespace ublox_gps

#endif  // UBLOX_GPS_WORKER_H
