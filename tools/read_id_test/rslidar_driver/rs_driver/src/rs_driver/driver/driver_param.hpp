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

#include <rs_driver/common/rs_log.hpp>
#include <string>
#include <map>

namespace robosense
{
namespace lidar
{
enum LidarType  ///< LiDAR type
{
  // mechanical
  RS_MECH = 0x01,
  RS16 = RS_MECH,
  RS32,
  RSBP,
  RSHELIOS,
  RSHELIOS_16P,
  RS128,
  RS80,
  RS48,
  RSP128,
  RSP80,
  RSP48,

  // mems
  RS_MEMS = 0x20,
  RSM1 = RS_MEMS,
  RSM2,
  RSE1,

  // jumbo
  RS_JUMBO = 0x100,
  RSM1_JUMBO = RS_JUMBO + RSM1,
};

inline bool isMech(LidarType type)
{
  return ((LidarType::RS_MECH <= type) && (type < LidarType::RS_MEMS));
}

inline bool isMems (LidarType type)
{
  return ((LidarType::RS_MEMS <= type) && (type < LidarType::RS_JUMBO));
}

inline bool isJumbo (LidarType type)
{
  return (LidarType::RS_JUMBO <= type);
}

inline std::string lidarTypeToStr(const LidarType& type)
{
  std::string str = "";
  switch (type)
  {
    case LidarType::RS16:
      str = "RS16";
      break;
    case LidarType::RS32:
      str = "RS32";
      break;
    case LidarType::RSBP:
      str = "RSBP";
      break;
    case LidarType::RSHELIOS:
      str = "RSHELIOS";
      break;
    case LidarType::RSHELIOS_16P:
      str = "RSHELIOS_16P";
      break;
    case LidarType::RS128:
      str = "RS128";
      break;
    case LidarType::RS80:
      str = "RS80";
      break;
    case LidarType::RS48:
      str = "RS48";
      break;
    case LidarType::RSP128:
      str = "RSP128";
      break;
    case LidarType::RSP80:
      str = "RSP80";
      break;
    case LidarType::RSP48:
      str = "RSP48";
      break;
    case LidarType::RSM1:
      str = "RSM1";
      break;
    case LidarType::RSM2:
      str = "RSM2";
      break;
    case LidarType::RSE1:
      str = "RSE1";
      break;
    case LidarType::RSM1_JUMBO:
      str = "RSM1_JUMBO";
      break;
    default:
      str = "ERROR";
      RS_ERROR << "RS_ERROR" << RS_REND;
  }
  return str;
}

inline LidarType strToLidarType(const std::string& type)
{
  if (type == "RS16")
  {
    return LidarType::RS16;
  }
  else if (type == "RS32")
  {
    return LidarType::RS32;
  }
  else if (type == "RSBP")
  {
    return LidarType::RSBP;
  }
  else if (type == "RSHELIOS")
  {
    return LidarType::RSHELIOS;
  }
  else if (type == "RSHELIOS_16P")
  {
    return LidarType::RSHELIOS_16P;
  }
  else if (type == "RS128")
  {
    return LidarType::RS128;
  }
  else if (type == "RS80")
  {
    return LidarType::RS80;
  }
  else if (type == "RS48")
  {
    return LidarType::RS48;
  }
  else if (type == "RSP128")
  {
    return LidarType::RSP128;
  }
  else if (type == "RSP80")
  {
    return LidarType::RSP80;
  }
  else if (type == "RSP48")
  {
    return lidar::LidarType::RSP48;
  }
  else if (type == "RSM1")
  {
    return lidar::LidarType::RSM1;
  }
  else if (type == "RSM2")
  {
    return lidar::LidarType::RSM2;
  }
  else if (type == "RSE1")
  {
    return LidarType::RSE1;
  }
  else if (type == "RSM1_JUMBO")
  {
    return LidarType::RSM1_JUMBO;
  }
  else
  {
    RS_ERROR << "Wrong lidar type: " << type << RS_REND;
    RS_ERROR << "Please give correct type: RS16, RS32, RSBP, RSHELIOS, RSHELIOS_16P, RS48, RS80, RS128, RSP128, RSP80, RSP48, "
             << "RSM1, RSM1_JUMBO, RSM2, RSE1." 
             << RS_REND;
    exit(-1);
  }
}

enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};

inline std::string inputTypeToStr(const InputType& type)
{
  std::string str = "";
  switch (type)
  {
    case InputType::ONLINE_LIDAR:
      str = "ONLINE_LIDAR";
      break;
    case InputType::PCAP_FILE:
      str = "PCAP_FILE";
      break;
    case InputType::RAW_PACKET:
      str = "RAW_PACKET";
      break;
    default:
      str = "ERROR";
      RS_ERROR << "RS_ERROR" << RS_REND;
  }
  return str;
}

enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_BLKS,
  SPLIT_BY_CUSTOM_BLKS
};

struct RSTransformParam  ///< The Point transform parameter
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Transform Parameters " << RS_REND;
    RS_INFOL << "x: " << x << RS_REND;
    RS_INFOL << "y: " << y << RS_REND;
    RS_INFOL << "z: " << z << RS_REND;
    RS_INFOL << "roll: " << roll << RS_REND;
    RS_INFOL << "pitch: " << pitch << RS_REND;
    RS_INFOL << "yaw: " << yaw << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }
};

struct RSDecoderParam  ///< LiDAR decoder parameter
{
  bool config_from_file = false; ///< Internal use only for debugging
  std::string angle_path = "";   ///< Internal use only for debugging
  bool wait_for_difop = true;    ///< true: start sending point cloud until receive difop packet
  float min_distance = 0.0f;     ///< min/max distances of point cloud range. valid if min distance or max distance > 0
  float max_distance = 0.0f; 
  float start_angle = 0.0f;      ///< Start angle of point cloud
  float end_angle = 360.0f;      ///< End angle of point cloud
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  
                                 ///< 1: Split frames by split_angle;
                                 ///< 2: Split frames by fixed number of blocks;
                                 ///< 3: Split frames by custom number of blocks (num_blks_split)
  float split_angle = 0.0f;      ///< Split angle(degree) used to split frame, only be used when split_frame_mode=1
  uint16_t num_blks_split = 1;   ///< Number of packets in one frame, only be used when split_frame_mode=3
  bool use_lidar_clock = false;  ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool dense_points = false;     ///< true: discard NAN points; false: reserve NAN points
  bool ts_first_point = false;   ///< true: time-stamp point cloud with the first point; false: with the last point;
  RSTransformParam transform_param; ///< Used to transform points

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Decoder Parameters " << RS_REND;
    RS_INFOL << "wait_for_difop: " << wait_for_difop << RS_REND;
    RS_INFOL << "min_distance: " << min_distance << RS_REND;
    RS_INFOL << "max_distance: " << max_distance << RS_REND;
    RS_INFOL << "start_angle: " << start_angle << RS_REND;
    RS_INFOL << "end_angle: " << end_angle << RS_REND;
    RS_INFOL << "use_lidar_clock: " << use_lidar_clock << RS_REND;
    RS_INFOL << "dense_points: " << dense_points << RS_REND;
    RS_INFOL << "config_from_file: " << config_from_file << RS_REND;
    RS_INFOL << "angle_path: " << angle_path << RS_REND;
    RS_INFOL << "split_frame_mode: " << split_frame_mode << RS_REND;
    RS_INFOL << "split_angle: " << split_angle << RS_REND;
    RS_INFOL << "num_blks_split: " << num_blks_split << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
    transform_param.print();
  }

};

struct RSInputParam  ///< The LiDAR input parameter
{
  uint16_t msop_port = 6699;                   ///< Msop packet port number
  uint16_t difop_port = 7788;                  ///< Difop packet port number
  std::string host_address = "0.0.0.0";        ///< Address of host
  std::string group_address = "0.0.0.0";       ///< Address of multicast group
  std::string pcap_path = "";                  ///< Absolute path of pcap file
  bool pcap_repeat = true;                     ///< true: The pcap bag will repeat play
  float pcap_rate = 1.0f;                      ///< Rate to read the pcap file
  bool use_vlan = false;                       ///< Vlan on-off
  uint16_t user_layer_bytes = 0;    ///< Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes = 0;    ///< Bytes of tail layer. thers is no tail layer if it is 0

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Input Parameters " << RS_REND;
    RS_INFOL << "msop_port: " << msop_port << RS_REND;
    RS_INFOL << "difop_port: " << difop_port << RS_REND;
    RS_INFOL << "host_address: " << host_address << RS_REND;
    RS_INFOL << "group_address: " << group_address << RS_REND;
    RS_INFOL << "pcap_path: " << pcap_path << RS_REND;
    RS_INFOL << "pcap_rate: " << pcap_rate << RS_REND;
    RS_INFOL << "pcap_repeat: " << pcap_repeat << RS_REND;
    RS_INFOL << "use_vlan: " << use_vlan << RS_REND;
    RS_INFOL << "user_layer_bytes: " << user_layer_bytes << RS_REND;
    RS_INFOL << "tail_layer_bytes: " << tail_layer_bytes << RS_REND;
    RS_INFO << "------------------------------------------------------" << RS_REND;
  }

};

struct RSDriverParam  ///< The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::RS16;  ///< Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; ///< Input type
  std::string frame_id = "rslidar";  ///< The frame id of LiDAR mesage
  RSInputParam input_param;          ///< Input parameter
  RSDecoderParam decoder_param;      ///< Decoder parameter

  void print() const
  {
    RS_INFO << "------------------------------------------------------" << RS_REND;
    RS_INFO << "             RoboSense Driver Parameters " << RS_REND;
    RS_INFOL << "input type: " << inputTypeToStr(input_type) << RS_REND;
    RS_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << RS_REND;
    RS_INFOL << "frame_id: "   << frame_id << RS_REND;
    RS_INFOL << "------------------------------------------------------" << RS_REND;

    input_param.print();
    decoder_param.print();
  }

};

struct DeviceInfo
{
  uint8_t sn[6];
  uint8_t mac[6];
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
};

struct DeviceStatus
{
  float voltage = 0.0f;
};

}  // namespace lidar
}  // namespace robosense
