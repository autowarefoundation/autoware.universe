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

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint8_t id[2];
  uint16_t azimuth;
  RSChannel channels[32];
} RS32MsopBlock;

typedef struct
{
  RSMsopHeaderV1 header;
  RS32MsopBlock blocks[12];
  unsigned int index;
  uint16_t tail;
} RS32MsopPkt;

typedef struct
{
  uint8_t id[8];
  uint16_t rpm;
  RSEthNetV1 eth;
  RSFOV fov;
  uint8_t reserved0[2];
  uint16_t phase_lock_angle;
  RSVersionV1 version;
  uint8_t reserved1[242];
  RSSN sn;
  uint16_t zero_cali;
  uint8_t return_mode;
  uint16_t sw_ver;
  RSTimestampYMD timestamp;
  RSStatusV1 status;
  uint8_t reserved2[5];
  RSDiagnoV1 diagno;
  uint8_t gprmc[86];
  RSCalibrationAngle vert_angle_cali[32];
  RSCalibrationAngle horiz_angle_cali[32];
  uint8_t reserved3[586];
  uint16_t tail;
} RS32DifopPkt;

#pragma pack(pop)

inline void RS32DifopPkt2Adapter (const RS32DifopPkt& src, AdapterDifopPkt& dst)
{
  dst.rpm = src.rpm;
  dst.fov = src.fov;
  dst.return_mode = src.return_mode;
  dst.sn = src.sn;
  dst.eth = src.eth;
  dst.version = src.version;
  dst.status = src.status;

  for (uint16_t i = 0; i < 32; i++)
  {
    uint16_t v;

    // vert angles
    dst.vert_angle_cali[i].sign = src.vert_angle_cali[i].sign;

    v = ntohs(src.vert_angle_cali[i].value);
    v = (uint16_t)std::round(v * 0.1f);
    dst.vert_angle_cali[i].value = htons(v);

    // horiz_angles
    dst.horiz_angle_cali[i].sign = src.horiz_angle_cali[i].sign;

    v = ntohs(src.horiz_angle_cali[i].value);
    v = (uint16_t)std::round(v * 0.1f);
    dst.horiz_angle_cali[i].value = htons(v);
  }
}

template <typename T_PointCloud>
class DecoderRS32 : public DecoderMech<T_PointCloud>
{
public:

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
  virtual ~DecoderRS32() = default;

  explicit DecoderRS32(const RSDecoderParam& param);

#ifndef UNIT_TEST
protected:
#endif

  static RSDecoderMechConstParam& getConstParam();
  static RSEchoMode getEchoMode(uint8_t mode);

  template <typename T_BlockIterator>
  bool internDecodeMsopPkt(const uint8_t* pkt, size_t size);
};

template <typename T_PointCloud>
inline RSDecoderMechConstParam& DecoderRS32<T_PointCloud>::getConstParam()
{
  static RSDecoderMechConstParam param = 
  {
    1248 // msop len
      , 1248 // difop len
      , 8 // msop id len
      , 8 // difop id len
      , {0x55, 0xAA, 0x05, 0x0A, 0x5A, 0xA5, 0x50, 0xA0} // msop id
    , {0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55} // difop id
    , {0xFF, 0xEE} // block id
    , 32 // laser number
    , 12 // blocks per packet
      , 32 // channels per block
      , 0.4f // distance min
      , 230.0f // distance max
      , 0.005f // distance resolution
      , 0.0625f // temperature resolution

      // lens center
      , 0.03997f // RX
      , -0.01087f // RY
      , 0.0f // RZ
  };

  INIT_ONLY_ONCE();

  float blk_ts = 55.52f;
  float firing_tss[] = 
  {
    0.00f,  2.88f,  5.76f,  8.64f, 11.52f, 14.40f, 17.28f, 20.16f, 
    23.04f, 25.92f, 28.80f, 31.68f, 34.56f, 37.44f, 40.32f, 44.64f,
    1.44f,  4.32f,  7.20f, 10.08f, 12.96f, 15.84f, 18.72f, 21.60f,
    24.48f, 27.36f, 30.24f, 33.12f, 36.00f, 38.88f, 41.76f, 46.08f
  };

  param.BLOCK_DURATION = blk_ts / 1000000;
  for (uint16_t i = 0; i < sizeof(firing_tss)/sizeof(firing_tss[0]); i++)
  {
    param.CHAN_TSS[i] = (double)firing_tss[i] / 1000000;
    param.CHAN_AZIS[i] = firing_tss[i] / blk_ts;
  }

  return param;
}

template <typename T_PointCloud>
inline RSEchoMode DecoderRS32<T_PointCloud>::getEchoMode(uint8_t mode)
{
  switch (mode)
  {
    case 0x00: // dual return
      return RSEchoMode::ECHO_DUAL;
    case 0x01: // strongest return
    case 0x02: // last return
    default:
      return RSEchoMode::ECHO_SINGLE;
  }
}

template <typename T_PointCloud>
inline DecoderRS32<T_PointCloud>::DecoderRS32(const RSDecoderParam& param)
  : DecoderMech<T_PointCloud>(getConstParam(), param)
{
}

template <typename T_PointCloud>
inline void DecoderRS32<T_PointCloud>::decodeDifopPkt(const uint8_t* packet, size_t size)
{
  const RS32DifopPkt& orig = *(const RS32DifopPkt*)packet;
  AdapterDifopPkt adapter;
  RS32DifopPkt2Adapter (orig, adapter);

  this->template decodeDifopCommon<AdapterDifopPkt>(adapter);

  this->echo_mode_ = getEchoMode (adapter.return_mode);
  this->split_blks_per_frame_ = (this->echo_mode_ == RSEchoMode::ECHO_DUAL) ? 
    (this->blks_per_frame_ << 1) : this->blks_per_frame_;
}

template <typename T_PointCloud>
inline bool DecoderRS32<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
{
  if (this->echo_mode_ == RSEchoMode::ECHO_SINGLE)
  {
    return internDecodeMsopPkt<SingleReturnBlockIterator<RS32MsopPkt>>(pkt, size);
  }
  else
  {
    return internDecodeMsopPkt<DualReturnBlockIterator<RS32MsopPkt>>(pkt, size);
  }
}

template <typename T_PointCloud>
template <typename T_BlockIterator>
inline bool DecoderRS32<T_PointCloud>::internDecodeMsopPkt(const uint8_t* packet, size_t size)
{
  const RS32MsopPkt& pkt = *(const RS32MsopPkt*)(packet);
  bool ret = false;

  this->temperature_ = parseTempInLe(&(pkt.header.temp)) * this->const_param_.TEMPERATURE_RES;

  double pkt_ts = 0;
  if (this->param_.use_lidar_clock)
  {
    pkt_ts = parseTimeYMD(&pkt.header.timestamp) * 1e-6;
  }
  else
  {
    uint64_t ts = getTimeHost();

    // roll back to first block to approach lidar ts as near as possible.
    pkt_ts = ts * 1e-6 - this->getPacketDuration();

    if (this->write_pkt_ts_)
    {
      createTimeYMD (ts, (RSTimestampYMD*)&pkt.header.timestamp);
    }
  }

  T_BlockIterator iter(pkt, this->const_param_.BLOCKS_PER_PKT, this->mech_const_param_.BLOCK_DURATION,
      this->block_az_diff_, this->fov_blind_ts_diff_);

  for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
  {
    const RS32MsopBlock& block = pkt.blocks[blk];

    if (memcmp(this->const_param_.BLOCK_ID, block.id, 2) != 0)
    {
      this->cb_excep_(Error(ERRCODE_WRONGMSOPID));
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

      int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
      int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, angle_horiz);
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
        setRing(point, this->chan_angles_.toUserChan(chan));

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
        setRing(point, this->chan_angles_.toUserChan(chan));

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
