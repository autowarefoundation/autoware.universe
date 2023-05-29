// Copyright 2020 F1TENTH Foundation
//
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

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_driver/vesc_packet.hpp"
#include "vesc_driver/vesc_packet_factory.hpp"

#include <cassert>
#include <iterator>
#include <memory>
#include <string>

namespace vesc_driver
{

/** Construct map on first use */
VescPacketFactory::FactoryMap * VescPacketFactory::getMap()
{
  static FactoryMap m;
  return &m;
}

void VescPacketFactory::registerPacketType(int payload_id, CreateFn fn)
{
  FactoryMap * p_map(getMap());
  assert(0 == p_map->count(payload_id));
  (*p_map)[payload_id] = fn;
}

/** Helper function for when createPacket can not create a packet */
VescPacketPtr createFailed(
  int * p_num_bytes_needed, std::string * p_what,
  const std::string & what, int num_bytes_needed = 0)
{
  if (p_num_bytes_needed != NULL) {*p_num_bytes_needed = num_bytes_needed;}
  if (p_what != NULL) {*p_what = what;}
  return VescPacketPtr();
}

VescPacketPtr VescPacketFactory::createPacket(
  const Buffer::const_iterator & begin,
  const Buffer::const_iterator & end,
  int * num_bytes_needed, std::string * what)
{
  // initialize output variables
  if (num_bytes_needed != NULL) {*num_bytes_needed = 0;}
  if (what != NULL) {what->clear();}

  // need at least VESC_MIN_FRAME_SIZE bytes in buffer
  int buffer_size(std::distance(begin, end));
  if (buffer_size < VescFrame::VESC_MIN_FRAME_SIZE) {
    return createFailed(
      num_bytes_needed, what, "Buffer does not contain a complete frame",
      VescFrame::VESC_MIN_FRAME_SIZE - buffer_size);
  }

  // buffer must begin with a start-of-frame
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME != *begin &&
    VescFrame::VESC_SOF_VAL_LARGE_FRAME != *begin)
  {
    return createFailed(num_bytes_needed, what, "Buffer must begin with start-of-frame character");
  }

  // get a view of the payload
  BufferRangeConst view_payload;
  if (VescFrame::VESC_SOF_VAL_SMALL_FRAME == *begin) {
    // payload size field is one byte
    view_payload.first = begin + 2;
    view_payload.second = view_payload.first + *(begin + 1);
  } else {
    assert(VescFrame::VESC_SOF_VAL_LARGE_FRAME == *begin);
    // payload size field is two bytes
    view_payload.first = begin + 3;
    view_payload.second = view_payload.first + (*(begin + 1) << 8) + *(begin + 2);
  }

  // check length
  if (std::distance(view_payload.first, view_payload.second) > VescFrame::VESC_MAX_PAYLOAD_SIZE) {
    return createFailed(num_bytes_needed, what, "Invalid payload length");
  }

  // get iterators to crc field, end-of-frame field, and a view of the whole frame
  Buffer::const_iterator iter_crc(view_payload.second);
  Buffer::const_iterator iter_eof(iter_crc + 2);
  BufferRangeConst view_frame(begin, iter_eof + 1);

  // do we have enough data in the buffer to complete the frame?
  int frame_size = std::distance(view_frame.first, view_frame.second);
  if (buffer_size < frame_size) {
    return createFailed(
      num_bytes_needed, what, "Buffer does not contain a complete frame",
      frame_size - buffer_size);
  }

  // is the end-of-frame character valid?
  if (VescFrame::VESC_EOF_VAL != *iter_eof) {
    return createFailed(num_bytes_needed, what, "Invalid end-of-frame character");
  }

  // is the crc valid?
  uint16_t crc = (static_cast<uint16_t>(*iter_crc) << 8) + *(iter_crc + 1);
  if (crc != CRC::Calculate(
      &(*view_payload.first), std::distance(view_payload.first, view_payload.second),
      VescFrame::CRC_TYPE))
  {
    return createFailed(num_bytes_needed, what, "Invalid checksum");
  }

  // frame looks good, construct the raw frame
  std::shared_ptr<VescFrame> raw_frame(new VescFrame(view_frame, view_payload));

  // if the packet has a payload, construct the corresponding subclass
  if (std::distance(view_payload.first, view_payload.second) > 0) {
    // get constructor function from payload id
    FactoryMap * p_map(getMap());
    FactoryMap::const_iterator search(p_map->find(*view_payload.first));
    if (search != p_map->end()) {
      return search->second(raw_frame);
    } else {
      // no subclass constructor for this packet
      return createFailed(num_bytes_needed, what, "Unkown payload type.");
    }
  } else {
    // no payload
    return createFailed(num_bytes_needed, what, "Frame does not have a payload");
  }
}

}  // namespace vesc_driver
