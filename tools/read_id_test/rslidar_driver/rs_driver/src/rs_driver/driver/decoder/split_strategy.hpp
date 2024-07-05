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

class SplitStrategy
{
public:
  virtual bool newBlock(int32_t angle) = 0;
  virtual ~SplitStrategy() = default;
};

class SplitStrategyByAngle : public SplitStrategy
{
public:
  SplitStrategyByAngle (int32_t split_angle)
   : split_angle_(split_angle), prev_angle_(split_angle)
  {
  }

  virtual ~SplitStrategyByAngle() = default;

  virtual bool newBlock(int32_t angle)
  {
    if (angle < prev_angle_)
    {
      prev_angle_ -= 36000;
    }

    bool v = ((prev_angle_ < split_angle_) && (split_angle_ <= angle));
#if 0
    if (v) 
    {
      std::cout << prev_angle_ << "\t" << angle << std::endl;
    }
#endif
    prev_angle_ = angle;
    return v;
  }


#ifndef UNIT_TEST
private:
#endif
    const int32_t split_angle_;
    int32_t prev_angle_;
};

class SplitStrategyByNum : public SplitStrategy
{
public:
  SplitStrategyByNum (uint16_t* max_blks)
   : max_blks_(max_blks), blks_(0)
  {
  }

  virtual ~SplitStrategyByNum() = default;

  virtual bool newBlock(int32_t)
  {
    blks_++;
    if (blks_ >= *max_blks_)
    {
      blks_ = 0;
      return true;
    }

    return false;
  }

#ifndef UNIT_TEST
private:
#endif
  uint16_t* max_blks_;
  uint16_t blks_;
};

class SplitStrategyBySeq
{
public:

  SplitStrategyBySeq()
    : prev_seq_(0)
  {
    setSafeRange();
  }

  bool newPacket(uint16_t seq)
  {
    bool split = false;

    if (seq < safe_seq_min_) // rewind
    {
      prev_seq_ = seq;
      split = true;
    }
    else if (seq < prev_seq_)
    {
      // do nothing.
    }
    else if (seq <= safe_seq_max_)
    {
      prev_seq_ = seq;
    }
    else
    {
      if (prev_seq_ == 0) 
        prev_seq_ = seq;

      //do nothing.
    }

    setSafeRange();
    return split;
  }

#ifndef UNIT_TEST
private:
#endif

  constexpr static uint16_t RANGE = 10;

  void setSafeRange()
  {
    safe_seq_min_ = (prev_seq_ > RANGE) ? (prev_seq_ - RANGE) : 0;
    safe_seq_max_ = prev_seq_ + RANGE;
  }

  uint16_t prev_seq_;
  uint16_t safe_seq_min_;
  uint16_t safe_seq_max_;
};

}  // namespace lidar
}  // namespace robosense
