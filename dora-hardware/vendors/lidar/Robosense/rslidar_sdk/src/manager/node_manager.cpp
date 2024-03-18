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

#include "manager/node_manager.hpp"
#include "source/source_driver.hpp"
#include "source/source_pointcloud_ros.hpp"
#include "source/source_packet_ros.hpp"

namespace robosense
{
namespace lidar
{

void NodeManager::init(const YAML::Node& config)
{
  YAML::Node common_config = yamlSubNodeAbort(config, "common");

  int msg_source = 0;
  yamlRead<int>(common_config, "msg_source", msg_source, 0);

  bool send_packet_ros;
  yamlRead<bool>(common_config, "send_packet_ros", send_packet_ros, false);

  bool send_point_cloud_ros;
  yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);

  bool send_point_cloud_proto;
  yamlRead<bool>(common_config, "send_point_cloud_proto", send_point_cloud_proto, false);

  bool send_packet_proto;
  yamlRead<bool>(common_config, "send_packet_proto", send_packet_proto, false);

  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");

  for (uint8_t i = 0; i < lidar_config.size(); ++i)
  {
    std::shared_ptr<Source> source;

    switch (msg_source)
    {
      case SourceType::MSG_FROM_LIDAR:  // online lidar

        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : Online LiDAR" << RS_REND;
        RS_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_LIDAR);
        source->init(lidar_config[i]);
        break;

      case SourceType::MSG_FROM_ROS_PACKET:  // pkt from ros

        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : ROS" << RS_REND;
        RS_INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;

        source = std::make_shared<SourcePacketRos>();
        source->init(lidar_config[i]);
        break;

      case SourceType::MSG_FROM_PCAP:  // pcap

        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : Pcap" << RS_REND;
        RS_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;

        source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_PCAP);
        source->init(lidar_config[i]);
        break;

      default:
        RS_ERROR << "Unsupported LiDAR message source:" << msg_source << "." << RS_REND;
        exit(-1);
    }

    if (send_packet_ros)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send Packets To : ROS" << RS_REND;
      RS_DEBUG << "Msop Topic: " << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>() << RS_REND;
      RS_DEBUG << "------------------------------------------------------" << RS_REND;

      std::shared_ptr<DestinationPacket> dst = std::make_shared<DestinationPacketRos>();
      dst->init(lidar_config[i]);
      source->regPacketCallback(dst);
    }

    if (send_point_cloud_ros)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send PointCloud To : ROS" << RS_REND;
      RS_DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>()
               << RS_REND;
      RS_DEBUG << "------------------------------------------------------" << RS_REND;

      std::shared_ptr<DestinationPointCloud> dst = std::make_shared<DestinationPointCloudRos>();
      dst->init(lidar_config[i]);
      source->regPointCloudCallback(dst);
    }

    sources_.emplace_back(source);
  }
}

void NodeManager::start()
{
  for (auto& iter : sources_)
  {
    if (iter != nullptr)
    {
      iter->start();
    }
  }
}

void NodeManager::stop()
{
  for (auto& iter : sources_)
  {
    if (iter != nullptr)
    {
      iter->stop();
    }
  }
}

NodeManager::~NodeManager()
{
  stop();
}

}  // namespace lidar
}  // namespace robosense
