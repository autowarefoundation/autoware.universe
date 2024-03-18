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

#include "source/source.hpp"

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/utility/sync_queue.hpp>

namespace robosense
{
namespace lidar
{

class SourceDriver : public Source
{
public:

  virtual void init(const YAML::Node& config);
  virtual void start();
  virtual void stop();
  virtual void regPacketCallback(DestinationPacket::Ptr dst);
  virtual ~SourceDriver();

  SourceDriver(SourceType src_type);

protected:

  std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);
  void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
  void putPacket(const Packet& msg);
  void putException(const lidar::Error& msg);
  void processPointCloud();

  std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;
  std::thread point_cloud_process_thread_;
  bool to_exit_process_;
};

SourceDriver::SourceDriver(SourceType src_type)
  : Source(src_type), to_exit_process_(false)
{
}

inline void SourceDriver::init(const YAML::Node& config)
{
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  lidar::RSDriverParam driver_param;

  // input related
  yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 6699);
  yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port, 7788);
  yamlRead<std::string>(driver_config, "host_address", driver_param.input_param.host_address, "0.0.0.0");
  yamlRead<std::string>(driver_config, "group_address", driver_param.input_param.group_address, "0.0.0.0");
  yamlRead<bool>(driver_config, "use_vlan", driver_param.input_param.use_vlan, false);
  yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
  yamlRead<float>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
  yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, true);
  yamlRead<uint16_t>(driver_config, "user_layer_bytes", driver_param.input_param.user_layer_bytes, 0);
  yamlRead<uint16_t>(driver_config, "tail_layer_bytes", driver_param.input_param.tail_layer_bytes, 0);

  // decoder related
  std::string lidar_type;
  yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
  driver_param.lidar_type = strToLidarType(lidar_type);

  // decoder
  yamlRead<bool>(driver_config, "wait_for_difop", driver_param.decoder_param.wait_for_difop, true);
  yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
  yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
  yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
  yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
  yamlRead<bool>(driver_config, "dense_points", driver_param.decoder_param.dense_points, false);
  yamlRead<bool>(driver_config, "ts_first_point", driver_param.decoder_param.ts_first_point, false);

  // mechanical decoder
  yamlRead<bool>(driver_config, "config_from_file", driver_param.decoder_param.config_from_file, false);
  yamlRead<std::string>(driver_config, "angle_path", driver_param.decoder_param.angle_path, "");

  uint16_t split_frame_mode;
  yamlRead<uint16_t>(driver_config, "split_frame_mode", split_frame_mode, 1);
  driver_param.decoder_param.split_frame_mode = SplitFrameMode(split_frame_mode);

  yamlRead<float>(driver_config, "split_angle", driver_param.decoder_param.split_angle, 0);
  yamlRead<uint16_t>(driver_config, "num_blks_split", driver_param.decoder_param.num_blks_split, 0);

  // transform
  yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
  yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
  yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
  yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
  yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
  yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);

  switch (src_type_)
  {
    case SourceType::MSG_FROM_LIDAR:
      driver_param.input_type = InputType::ONLINE_LIDAR;
      break;
    case SourceType::MSG_FROM_PCAP:
      driver_param.input_type = InputType::PCAP_FILE;
      break;
    default:
      driver_param.input_type = InputType::RAW_PACKET;
      break;
  }

  driver_param.print();

  driver_ptr_.reset(new lidar::LidarDriver<LidarPointCloudMsg>());
  driver_ptr_->regPointCloudCallback(std::bind(&SourceDriver::getPointCloud, this), 
      std::bind(&SourceDriver::putPointCloud, this, std::placeholders::_1));
  driver_ptr_->regExceptionCallback(
      std::bind(&SourceDriver::putException, this, std::placeholders::_1));
  point_cloud_process_thread_ = std::thread(std::bind(&SourceDriver::processPointCloud, this));

  if (!driver_ptr_->init(driver_param))
  {
    RS_ERROR << "Driver Initialize Error...." << RS_REND;
    exit(-1);
  }
}

inline void SourceDriver::start()
{
  driver_ptr_->start();
}

inline SourceDriver::~SourceDriver()
{
  stop();
}

inline void SourceDriver::stop()
{
  driver_ptr_->stop();

  to_exit_process_ = true;
  point_cloud_process_thread_.join();
}

inline std::shared_ptr<LidarPointCloudMsg> SourceDriver::getPointCloud(void)
{
  std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();
  if (point_cloud.get() != NULL)
  {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

inline void SourceDriver::regPacketCallback(DestinationPacket::Ptr dst)
{
  Source::regPacketCallback(dst);

  if (pkt_cb_vec_.size() == 1)
  {
    driver_ptr_->regPacketCallback(
        std::bind(&SourceDriver::putPacket, this, std::placeholders::_1));
  }
}

inline void SourceDriver::putPacket(const Packet& msg)
{
  sendPacket(msg);
}

void SourceDriver::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
{
  point_cloud_queue_.push(msg);
}

void SourceDriver::processPointCloud()
{
  while (!to_exit_process_)
  {
    std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL)
    {
      continue;
    }

    sendPointCloud(msg);

    free_point_cloud_queue_.push(msg);
  }
}

inline void SourceDriver::putException(const lidar::Error& msg)
{
  switch (msg.error_code_type)
  {
    case lidar::ErrCodeType::INFO_CODE:
      RS_INFO << msg.toString() << RS_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      RS_WARNING << msg.toString() << RS_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      RS_ERROR << msg.toString() << RS_REND;
      break;
  }
}

}  // namespace lidar
}  // namespace robosense
