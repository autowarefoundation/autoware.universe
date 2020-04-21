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

#ifndef UBLOX_MSGS_CHECKSUM_H
#define UBLOX_MSGS_CHECKSUM_H

#include <stdint.h>

namespace ublox {

/**
 * @brief calculate the checksum of a u-blox_message
 * @param data the start of the u-blox message
 * @param data the size of the u-blox message
 * @param ck_a the checksum a output
 * @param ck_b the checksum b output
 */
static inline void calculateChecksum(const uint8_t* data, uint32_t size, uint8_t& ck_a, uint8_t& ck_b) {
  ck_a = 0;
  ck_b = 0;
  for (uint32_t i = 0; i < size; ++i) {
    ck_a = ck_a + data[i];
    ck_b = ck_b + ck_a;
  }
}

/**
 * @brief calculate the checksum of a u-blox_message.
 * @param data the start of the u-blox message
 * @param data the size of the u-blox message
 * @param checksum the checksum output
 * @return the checksum
 */
static inline uint16_t calculateChecksum(const uint8_t* data, uint32_t size, uint16_t& checksum) {
  uint8_t* byte = reinterpret_cast<uint8_t*>(&checksum);
  calculateChecksum(data, size, byte[0], byte[1]);
  return checksum;
}

}  // namespace ublox

#endif  // UBLOX_MSGS_CHECKSUM_H
