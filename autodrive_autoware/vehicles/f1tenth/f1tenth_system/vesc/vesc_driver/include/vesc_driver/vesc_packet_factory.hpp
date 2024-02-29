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

#ifndef VESC_DRIVER__VESC_PACKET_FACTORY_HPP_
#define VESC_DRIVER__VESC_PACKET_FACTORY_HPP_

#include "vesc_driver/vesc_packet.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace vesc_driver
{

/**
 * Class for creating VESC packets from raw data.
 */
class VescPacketFactory
{
public:
  /** Return the global factory object */
  static VescPacketFactory * getFactory();

  /**
   * Create a VescPacket from a buffer (factory function). Packet must start (start of frame
   * character) at @p begin and complete (end of frame character) before *p end. The buffer element
   * at @p end is not examined, i.e. it can be the past-the-end element. Only returns a packet if
   * the packet is valid, i.e. valid size, matching checksum, complete etc. An empty pointer is
   * returned if a packet cannot be found or if it is invalid. If a valid packet is not found,
   * optional output parameter @what is set to a string providing a reason why a packet was not
   * found. If a packet was not found because additional bytes are needed on the buffer, optional
   * output parameter @p num_bytes_needed will contain the number of bytes needed to either
   * determine the size of the packet or complete the packet. Output parameters @p num_bytes_needed
   * and @p what will be set to 0 and empty if a valid packet is found.
   *
   * @param begin[in] Iterator to a buffer at the start-of-frame character
   * @param end[in] Iterator to the buffer past-the-end element.
   * @param num_bytes_needed[out] Number of bytes needed to determine the packet size or complete
   *                              the frame.
   * @param what[out] Human readable string giving a reason why the packet was not found.
   *
   * @return Pointer to a valid VescPacket if successful. Otherwise, an empty pointer.
   */
  static VescPacketPtr createPacket(
    const Buffer::const_iterator & begin,
    const Buffer::const_iterator & end,
    int * num_bytes_needed, std::string * what);

  typedef std::function<VescPacketPtr(std::shared_ptr<VescFrame>)> CreateFn;

  /** Register a packet type with the factory. */
  static void registerPacketType(int payload_id, CreateFn fn);

  /**
   * Delete copy constructor and equals operator.
   */
  VescPacketFactory(const VescPacketFactory &) = delete;
  VescPacketFactory & operator=(const VescPacketFactory &) = delete;

private:
  VescPacketFactory();
  typedef std::map<int, CreateFn> FactoryMap;
  static FactoryMap * getMap();
};

template<typename PACKETTYPE>
class PacketFactoryTemplate
{
public:
  explicit PacketFactoryTemplate(int payload_id)
  {
    VescPacketFactory::registerPacketType(payload_id, &PacketFactoryTemplate::create);
  }

  static VescPacketPtr create(std::shared_ptr<VescFrame> frame)
  {
    return VescPacketPtr(new PACKETTYPE(frame));
  }
};

/** Use this macro to register packets */
#define REGISTER_PACKET_TYPE(id, klass) \
  static PacketFactoryTemplate<klass> global_ ## klass ## Factory((id));

}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_PACKET_FACTORY_HPP_
