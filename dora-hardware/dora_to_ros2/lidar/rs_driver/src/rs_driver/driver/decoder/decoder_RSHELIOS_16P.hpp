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
#include <rs_driver/driver/decoder/decoder_mech.hpp>
#include <rs_driver/driver/decoder/decoder_RSHELIOS.hpp>

namespace robosense
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderRSHELIOS_16P : public DecoderMech<T_PointCloud>
{
public: 
  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRSHELIOS_16P() = default;

  explicit DecoderRSHELIOS_16P(const RSDecoderParam& param);

#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);

  void calcParam();

  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);
};

template <typename T_PointCloud>
inline RSDecoderMechConstParam& DecoderRSHELIOS_16P<T_PointCloud>::getConstParam()
{
  static RSDecoderMechConstParam param = 
  {
    1248 // msop len
      , 1248 // difop len
      , 4 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x5A} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 16 // laser number
    , 12 // blocks per packet
      , 32 // channels per block
      , 0.1f // distance min
      , 180.0f // distance max
      , 0.0025f // distance resolution
      , 0.0625f // temperature resolution

      // lens center
      , 0.03498f // RX
      , -0.015f // RY
      , 0.0f // RZ
  };

  float blk_ts = 55.56f;
  param.BLOCK_DURATION = blk_ts / 1000000;

  return param;
}

template <typename T_PointCloud>
inline void DecoderRSHELIOS_16P<T_PointCloud>::calcParam()
{
  float blk_ts = 55.56f;
  float firing_tss[] = 
  {
     0.00f,  3.15f,  6.30f,  9.45f, 13.26f, 17.08f, 20.56f, 23.71f,
    26.53f, 27.77f, 31.49f, 32.73f, 36.46f, 38.94f, 41.42f, 43.91f,
    55.56f, 58.70f, 61.85f, 65.00f, 68.82f, 72.64f, 76.12f, 79.27f, 
    82.08f, 83.32f, 87.05f, 88.29f, 92.01f, 94.50f, 96.98f, 99.46f
  };

  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
  {
    Rs16SingleReturnBlockIterator<RSHELIOSMsopPkt>::calcChannel(blk_ts, firing_tss, 
       this->mech_const_param_.CHAN_AZIS, this->mech_const_param_.CHAN_TSS);
  }
  else
  {
    Rs16DualReturnBlockIterator<RSHELIOSMsopPkt>::calcChannel(blk_ts, firing_tss, 
       this->mech_const_param_.CHAN_AZIS, this->mech_const_param_.CHAN_TSS);
  }
}

template <typename T_PointCloud>
inline RSEchoMode DecoderRSHELIOS_16P<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x00: // dual return
      return RSEchoMode::ECHO_DUAL;
    case 0x04: // strongest return
    case 0x05: // last return
    case 0x06: // nearest return
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline DecoderRSHELIOS_16P<T_PointCloud>::DecoderRSHELIOS_16P(const RSDecoderParam& param)
  : DecoderMech<T_PointCloud>(getConstParam(), param)
{
  this->packet_duration_ = 
    this->mech_const_param_.BLOCK_DURATION * this->const_param_.BLOCKS_PER_PKT * 2;

  calcParam();
}

template <typename T_PointCloud>
inline void DecoderRSHELIOS_16P<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RSHELIOSDifopPkt& pkt = *(const RSHELIOSDifopPkt*)(packet);
  this->template decodeDifopCommon<RSHELIOSDifopPkt>(pkt);

  RSEchoMode echo_mode = getEchoMode (pkt.return_mode);
  if (this->echo_mode_ != echo_mode)
  {
    this->echo_mode_ = echo_mode;
    this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
      this->blks_per_frame_ : (this->blks_per_frame_ >> 1);

    calcParam();
  }
}

template <typename T_PointCloud>
inline bool DecoderRSHELIOS_16P<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
  {
    return internDecodeMsopPkt<Rs16SingleReturnBlockIterator<RSHELIOSMsopPkt>>(pkt, size);
  }
  else
  {
    return internDecodeMsopPkt<Rs16DualReturnBlockIterator<RSHELIOSMsopPkt>>(pkt, size);
  }
}

template <typename T_PointCloud>
template <typename T_BlockIterator>
inline bool DecoderRSHELIOS_16P<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RSHELIOSMsopPkt& pkt = *(const RSHELIOSMsopPkt*)(packet);
  bool ret = false;

  this->temperature_ = parseTempInLe(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeUTCWithUs(&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeUTCWithUs (ts, (RSTimestampUTC*)&pkt.header.timestamp);
    }
  }

  T_BlockIterator iter(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION,
      this->block_az_diff_, this->fov_blind_ts_diff_);

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RSHELIOSMsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 2) != 0)
    {
      this->cb_excep_(Error(ERRCODE_WRONGMSOPBLKID));
      break;
    }

    int32_t block_az_diff;
    double block_ts_off;
    iter.get(blk, block_az_diff, block_ts_off);

    double block_ts = pkt_ts + block_ts_off;
    int32_t block_az = ntohs(block.azimuth);
    if (this->split_strategy_->newBlock(block_az))
    {
      this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
      this->first_point_ts_ = block_ts;
      ret = true;
    }

    for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
    {
      const RSChannel& channel = block.channels[chan]; 

      double chan_ts = block_ts + this->mech_const_param_.CHAN_TSS[chan];
      int32_t angle_horiz = block_az + 
        (int32_t)((float)block_az_diff * this->mech_const_param_.CHAN_AZIS[chan]);

      uint16_t laser = chan % 16;
      int32_t angle_vert = this->chan_angles_.vertAdjust(laser);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(laser, angle_horiz);
      float distance = ntohs(channel.distance) * this->const_param_.DISTANCE_RES;

      if (this->distance_section_.in(distance) && this->scan_section_.in(angle_horiz_final))
      {
        float x =  distance * COS(angle_vert) * COS(angle_horiz_final) + this->mech_const_param_.RX * COS(angle_horiz);
        float y = -distance * COS(angle_vert) * SIN(angle_horiz_final) - this->mech_const_param_.RX * SIN(angle_horiz);
        float z =  distance * SIN(angle_vert) + this->mech_const_param_.RZ;
        this->transformPoint(x, y, z);

        typename T_PointCloud::PointT point;
        setX(point, x);
        setY(point, y);
        setZ(point, z);
        setIntensity(point, channel.intensity);
        setTimestamp(point, chan_ts);
        setRing(point, (this->chan_angles_.toUserChan(laser)));

        this->point_cloud_->points.emplace_back(point);
      }
      else if (!this->param_.dense_points)
      {
        typename T_PointCloud::PointT point;
        setX(point, NAN);
        setY(point, NAN);
        setZ(point, NAN);
        setIntensity(point, 0);
        setTimestamp(point, chan_ts);
        setRing(point, (this->chan_angles_.toUserChan(laser)));

        this->point_cloud_->points.emplace_back(point);
      }

      this->prev_point_ts_ = chan_ts;
    }
  }

  this->prev_pkt_ts_ = pkt_ts;
  return ret;
}

}  // namespace lidar
}  // namespace robosense
