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

namespace robosense
{
namespace lidar
{

template <typename T_Packet>
class BlockIterator
{
public:

  static const int MAX_BLOCKS_PER_PKT = 12;

  void get(uint16_t blk, int32_t& az_diff, double& ts)
  {
    az_diff = az_diffs[blk];
    ts = tss[blk];
  }

  BlockIterator(const T_Packet& pkt, uint16_t blocks_per_pkt, double block_duration, 
      uint16_t block_az_duration, double fov_blind_duration)
    : pkt_(pkt), BLOCKS_PER_PKT(blocks_per_pkt), BLOCK_DURATION(block_duration), 
    BLOCK_AZ_DURATION(block_az_duration), FOV_BLIND_DURATION(fov_blind_duration) 
  {
    // assert(BLOCKS_PER_PKT <= MAX_BLOCKS_PER_PKT);
  }

protected:

  const T_Packet& pkt_;
  const uint16_t BLOCKS_PER_PKT;
  const double BLOCK_DURATION;
  const uint16_t BLOCK_AZ_DURATION;
  const double FOV_BLIND_DURATION;
  int32_t az_diffs[MAX_BLOCKS_PER_PKT];
  double tss[MAX_BLOCKS_PER_PKT];
};

template <typename T_Packet>
class SingleReturnBlockIterator : public BlockIterator<T_Packet>
{
public:

  SingleReturnBlockIterator(const T_Packet& pkt, uint16_t blocks_per_pkt, double block_duration, 
      uint16_t block_az_duration, double fov_blind_duration)
    : BlockIterator<T_Packet>(pkt, blocks_per_pkt, block_duration, block_az_duration, fov_blind_duration) 
  {
    double tss = 0;
    uint16_t blk = 0;
    for (; blk < (this->BLOCKS_PER_PKT - 1); blk++)
    {
      double ts_diff = this->BLOCK_DURATION;
      int32_t az_diff = ntohs(this->pkt_.blocks[blk+1].azimuth) - ntohs(this->pkt_.blocks[blk].azimuth);
      if (az_diff < 0) { az_diff += 36000; }

      // Skip FOV blind zone 
      if (az_diff > 100)
      {
        az_diff = this->BLOCK_AZ_DURATION;
        ts_diff = this->FOV_BLIND_DURATION;
      }

      this->az_diffs[blk] = az_diff;
      this->tss[blk] = tss;

      tss += ts_diff;
    }

    this->az_diffs[blk] = this->BLOCK_AZ_DURATION;
    this->tss[blk] = tss;
  }
};

template <typename T_Packet>
class DualReturnBlockIterator : public BlockIterator<T_Packet>
{
public:

  DualReturnBlockIterator(const T_Packet& pkt, uint16_t blocks_per_pkt, double block_duration,
      uint16_t block_az_duration, double fov_blind_duration)
    : BlockIterator<T_Packet>(pkt, blocks_per_pkt, block_duration, block_az_duration, fov_blind_duration) 
  {
    double tss = 0;
    uint16_t blk = 0;
    for (; blk < (this->BLOCKS_PER_PKT - 2); blk = blk + 2)
    {
      double ts_diff = this->BLOCK_DURATION;
      int32_t az_diff = ntohs(this->pkt_.blocks[blk+2].azimuth) - ntohs(this->pkt_.blocks[blk].azimuth);
      if (az_diff < 0) { az_diff += 36000; }

      //  Skip FOV blind zone
      if (az_diff > 100)
      {
        az_diff = this->BLOCK_AZ_DURATION;
        ts_diff = this->FOV_BLIND_DURATION;
      }

      this->az_diffs[blk] = this->az_diffs[blk+1] = az_diff;
      this->tss[blk] = this->tss[blk+1] = tss;

      tss += ts_diff;
    }

    this->az_diffs[blk] = this->az_diffs[blk+1] = this->BLOCK_AZ_DURATION;
    this->tss[blk] = this->tss[blk+1] = tss;
  }
};

template <typename T_Packet>
class ABDualReturnBlockIterator : public BlockIterator<T_Packet>
{
public:

  ABDualReturnBlockIterator(const T_Packet& pkt, uint16_t blocks_per_pkt, double block_duration,
      uint16_t block_az_duration, double fov_blind_duration)
    : BlockIterator<T_Packet>(pkt, blocks_per_pkt, block_duration, block_az_duration, fov_blind_duration) 
  {
    double ts_diff = this->BLOCK_DURATION;
    int32_t az_diff = ntohs(this->pkt_.blocks[2].azimuth) - ntohs(this->pkt_.blocks[0].azimuth);
    if (az_diff < 0) { az_diff += 36000; }

    //  Skip FOV blind zone
    if (az_diff > 100)
    {
      az_diff = this->BLOCK_AZ_DURATION;
      ts_diff = this->FOV_BLIND_DURATION;
    }

    if (ntohs(this->pkt_.blocks[0].azimuth) == ntohs(this->pkt_.blocks[1].azimuth)) // AAB
    {
      double tss = 0;
      this->az_diffs[0] = this->az_diffs[1] = az_diff;
      this->tss[0] = this->tss[1] = tss;

      tss += ts_diff;
      this->az_diffs[2] = this->BLOCK_AZ_DURATION;
      this->tss[2] = tss;
    }
    else // ABB
    {
      double tss = 0;
      this->az_diffs[0] = az_diff;
      this->tss[0] = tss;

      tss += ts_diff;
      this->az_diffs[1] = this->az_diffs[2] = this->BLOCK_AZ_DURATION;
      this->tss[1] = this->tss[2] = tss;
    }
  }
};

template <typename T_Packet>
class Rs16SingleReturnBlockIterator : public BlockIterator<T_Packet>
{
public:

  static 
  void calcChannel(const float blk_ts, const float firing_tss[], float az_percents[], double ts_diffs[])
  {
    for (uint16_t chan = 0; chan < 32; chan++)
    {
      az_percents[chan] = firing_tss[chan] / (blk_ts * 2);
      ts_diffs[chan] = (double)firing_tss[chan] / 1000000;
    }
  }

  Rs16SingleReturnBlockIterator(const T_Packet& pkt, uint16_t blocks_per_pkt, double block_duration, 
      uint16_t block_az_duration, double fov_blind_duration)
    : BlockIterator<T_Packet>(pkt, blocks_per_pkt, block_duration, block_az_duration, fov_blind_duration) 
  {
    double tss = 0;
    uint16_t blk = 0;
    for (; blk < (this->BLOCKS_PER_PKT - 1); blk++)
    {
      double ts_diff = this->BLOCK_DURATION * 2;
      int32_t az_diff = ntohs(this->pkt_.blocks[blk+1].azimuth) - ntohs(this->pkt_.blocks[blk].azimuth);
      if (az_diff < 0) { az_diff += 36000; }

      // Skip FOV blind zone 
      if (az_diff > 100)
      {
        az_diff = this->BLOCK_AZ_DURATION * 2;
        ts_diff = this->FOV_BLIND_DURATION;
      }

      this->az_diffs[blk] = az_diff;
      this->tss[blk] = tss;

      tss += ts_diff;
    }

    this->az_diffs[blk] = this->BLOCK_AZ_DURATION * 2;
    this->tss[blk] = tss;
  }
};

template <typename T_Packet>
class Rs16DualReturnBlockIterator : public BlockIterator<T_Packet>
{
public:

  static 
  void calcChannel(const float blk_ts, const float firing_tss[], float az_percents[], double ts_diffs[])
  {
    for (uint16_t chan = 0; chan < 32; chan++)
    {
      az_percents[chan] = firing_tss[chan%16] / blk_ts;
      ts_diffs[chan] = (double)firing_tss[chan%16] / 1000000;
    }
  }

  Rs16DualReturnBlockIterator(const T_Packet& pkt, uint16_t blocks_per_pkt, double block_duration,
      uint16_t block_az_duration, double fov_blind_duration)
    : BlockIterator<T_Packet>(pkt, blocks_per_pkt, block_duration, block_az_duration, fov_blind_duration) 
  {
    double tss = 0;
    uint16_t blk = 0;
    for (; blk < (this->BLOCKS_PER_PKT - 1); blk++)
    {
      double ts_diff = this->BLOCK_DURATION;
      int32_t az_diff = ntohs(this->pkt_.blocks[blk+1].azimuth) - ntohs(this->pkt_.blocks[blk].azimuth);
      if (az_diff < 0) { az_diff += 36000; }

      // Skip FOV blind zone 
      if (az_diff > 100)
      {
        az_diff = this->BLOCK_AZ_DURATION;
        ts_diff = this->FOV_BLIND_DURATION;
      }

      this->az_diffs[blk] = az_diff;
      this->tss[blk] = tss;

      tss += ts_diff;
    }

    this->az_diffs[blk] = this->BLOCK_AZ_DURATION;
    this->tss[blk] = tss;
  }
};

}  // namespace lidar
}  // namespace robosense
