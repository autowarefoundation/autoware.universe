/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/utility/dbg.hpp>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)

struct iphdr
{
  uint8_t version;
  uint8_t tos;
  uint16_t tot_len;

  uint16_t id;
  uint16_t frag_off;

  uint8_t ttl;
  uint8_t protocol;
  uint16_t check;

  uint32_t saddr;
  uint32_t daddr;
};

struct udphdr
{
  uint16_t source;
  uint16_t dest;
  uint16_t len;
  uint16_t check;
};

#pragma pack(pop)

class Jumbo
{
public:

  Jumbo()
    : ip_id_(0), buf_off_(0)
  {
  }

  bool new_fragment(const uint8_t* pkt_data, size_t pkt_data_size, 
   uint16_t* udp_port, const uint8_t**ip_data, size_t* ip_data_len);

private:

  uint16_t dst_port(const uint8_t* buf);

  uint16_t ip_id_;
  uint8_t  buf_[IP_LEN];
  uint16_t buf_off_;
}; 

inline bool Jumbo::new_fragment(const uint8_t* pkt_data, size_t pkt_data_size, 
    uint16_t* udp_port, const uint8_t** udp_data, size_t* udp_data_len)
{
  // Is it an ip packet ?
  const uint16_t* eth_type = (const uint16_t*)(pkt_data + 12);
  if (ntohs(*eth_type) != 0x0800)
    return false;

  // is it a udp packet?
  const struct iphdr* ip_hdr = (const struct iphdr*)(pkt_data + 14);
  if (ip_hdr->protocol != 0x11)
    return false;

  // ip data
  uint16_t ip_hdr_size = (ip_hdr->version & 0xf) * 4;
  uint16_t ip_len = ntohs(ip_hdr->tot_len);

  const uint8_t* ip_data = pkt_data + 14 + ip_hdr_size;
  uint16_t ip_data_len = ip_len - ip_hdr_size;

  // ip fragment
  uint16_t ip_id = ntohs (ip_hdr->id);
  uint16_t f_off = ntohs(ip_hdr->frag_off);
  uint16_t frag_flags  = (f_off >> 13);
  uint16_t frag_off    = (f_off & 0x1fff) * 8; // 8 octet boudary

#define MORE_FRAGS(flags) ((flags & 0x01) != 0)

  if (ip_id == ip_id_)
  {
    if (frag_off == buf_off_)
    {
      memcpy (buf_ + buf_off_, ip_data, ip_data_len);
      buf_off_ += ip_data_len;

      if (!MORE_FRAGS(frag_flags))
      {
#if 0
        printf ("--- end new packet. ip_id:0x%x, len:%d\n", ip_id_, buf_off_);

        for (uint16_t off = UDP_HDR_LEN, i = 0; off < buf_len(); off += 984, i++)
        {
          char title[32];
          sprintf (title, "jumbo %d ", i);

          hexdump (buf() + off, 32, title);
        }
#endif

        ip_id_ = 0;

        *udp_port = dst_port (buf_);
        *udp_data = buf_ + UDP_HDR_LEN;
        *udp_data_len = buf_off_ - UDP_HDR_LEN;
        return true;
      }
    }
  }
  else
  {
    if (frag_off == 0)
    {
      if (MORE_FRAGS(frag_flags))
      {
        ip_id_ = ip_id;
        buf_off_ = 0;

        memcpy (buf_ + buf_off_, ip_data, ip_data_len);
        buf_off_ += ip_data_len;

#if 0
        printf ("+++ start packet 0x%x\n", ip_id_);
#endif
      }
      else
      {
#if 0
        printf ("+++ non-fragment packet 0x%x\n", ip_id);
#endif

        *udp_port = dst_port (ip_data);
        *udp_data = ip_data + UDP_HDR_LEN;
        *udp_data_len = ip_data_len - UDP_HDR_LEN;
        return true;
      }
    }
  }

  return false;
}

inline uint16_t Jumbo::dst_port(const uint8_t* buf)
{
  const struct udphdr* udp_hdr = (const struct udphdr*)buf;
  return ntohs(udp_hdr->dest);
}

}  // namespace lidar
}  // namespace robosense
