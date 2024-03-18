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

#include "msg/rs_msg/lidar_point_cloud_msg.hpp"

#include "utility/yaml_reader.hpp"

#include <rs_driver/msg/packet.hpp>

namespace robosense
{
namespace lidar
{

class DestinationPointCloud
{
public:
  typedef std::shared_ptr<DestinationPointCloud> Ptr;

  virtual void init(const YAML::Node& config){}
  virtual void start() {}
  virtual void stop() {}
  virtual void sendPointCloud(const LidarPointCloudMsg& msg) = 0;
  virtual ~DestinationPointCloud() = default;
};

class DestinationPacket
{
public:
  typedef std::shared_ptr<DestinationPacket> Ptr;

  virtual void init(const YAML::Node& config){}
  virtual void start() {}
  virtual void stop() {}
  virtual void sendPacket(const Packet& msg) = 0;
  virtual ~DestinationPacket() = default;
};

enum SourceType
{
  MSG_FROM_LIDAR = 1,
  MSG_FROM_ROS_PACKET = 2,
  MSG_FROM_PCAP = 3,
  MSG_FROM_PROTO_PACKET = 4,
  MSG_FROM_PROTO_POINTCLOUD = 5
};

class Source
{
public:
  typedef std::shared_ptr<Source> Ptr;

  virtual void init(const YAML::Node& config) {}
  virtual void start() {}
  virtual void stop() {}
  virtual void regPointCloudCallback(DestinationPointCloud::Ptr dst);
  virtual void regPacketCallback(DestinationPacket::Ptr dst);
  virtual ~Source() = default;
  Source(SourceType src_type);

protected:

  void sendPacket(const Packet& msg);
  void sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);

  SourceType src_type_;
  std::vector<DestinationPointCloud::Ptr> pc_cb_vec_;
  std::vector<DestinationPacket::Ptr> pkt_cb_vec_;
};

inline Source::Source(SourceType src_type)
  : src_type_(src_type)
{
}

inline void Source::regPacketCallback(DestinationPacket::Ptr dst)
{
  pkt_cb_vec_.emplace_back(dst);
}

inline void Source::regPointCloudCallback(DestinationPointCloud::Ptr dst)
{
  pc_cb_vec_.emplace_back(dst);
}

inline void Source::sendPacket(const Packet& msg)
{
  for (auto iter : pkt_cb_vec_)
  {
    iter->sendPacket(msg);
  }
}

inline void Source::sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
{
  for (auto iter : pc_cb_vec_)
  {
    iter->sendPointCloud(*msg);
  }
}


}  // namespace lidar
}  // namespace robosense
