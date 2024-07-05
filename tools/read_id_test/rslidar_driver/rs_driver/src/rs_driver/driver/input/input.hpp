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

#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/utility/buffer.hpp>

#include <functional>
#include <thread>
#include <cstring>

#define VLAN_HDR_LEN  4
#define ETH_HDR_LEN   42
#define ETH_LEN       (ETH_HDR_LEN + VLAN_HDR_LEN + 1500)
#define IP_LEN        65536 
#define UDP_HDR_LEN   8

namespace robosense
{
namespace lidar
{
class Input
{
public:
  Input(const RSInputParam& input_param);

  inline void regCallback(
      const std::function<void(const Error&)>& cb_excep,
      const std::function<std::shared_ptr<Buffer>(size_t)>& cb_get_pkt,
      const std::function<void(std::shared_ptr<Buffer>, bool)>& cb_put_pkt);

  virtual bool init() = 0;
  virtual bool start() = 0;
  virtual void stop();
  virtual ~Input()
  {
  }

protected:
  inline void pushPacket(std::shared_ptr<Buffer> pkt, bool stuffed = true);

  RSInputParam input_param_;
  std::function<std::shared_ptr<Buffer>(size_t size)> cb_get_pkt_;
  std::function<void(std::shared_ptr<Buffer>, bool)> cb_put_pkt_;
  std::function<void(const Error&)> cb_excep_;
  std::thread recv_thread_;
  bool to_exit_recv_;
  bool init_flag_;
  bool start_flag_;
};

inline Input::Input(const RSInputParam& input_param)
  : input_param_(input_param), to_exit_recv_(false), 
  init_flag_(false), start_flag_(false)
{
}

inline void Input::regCallback(
    const std::function<void(const Error&)>& cb_excep,
    const std::function<std::shared_ptr<Buffer>(size_t)>& cb_get_pkt, 
    const std::function<void(std::shared_ptr<Buffer>, bool)>& cb_put_pkt)
{
  cb_excep_   = cb_excep;
  cb_get_pkt_ = cb_get_pkt;
  cb_put_pkt_ = cb_put_pkt;
}

inline void Input::stop()
{
  if (start_flag_)
  {
    to_exit_recv_ = true;
    recv_thread_.join();

    start_flag_ = false;
  }
}

inline void Input::pushPacket(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  cb_put_pkt_(pkt, stuffed);
}

}  // namespace lidar
}  // namespace robosense
