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

#include <rs_driver/common/rs_common.hpp>

#include <cmath>

namespace robosense
{
namespace lidar
{

//#define DBG

class Trigon
{
public:

  constexpr static int32_t ANGLE_MIN = -9000;
  constexpr static int32_t ANGLE_MAX = 45000;

  Trigon()
  {
    int32_t range = ANGLE_MAX - ANGLE_MIN;
#ifdef DBG
    o_angles_ = (int32_t*)malloc(range * sizeof(int32_t));
#endif
    o_sins_ = (float*)malloc(range * sizeof(float));
    o_coss_ = (float*)malloc(range * sizeof(float));

    for (int32_t i = ANGLE_MIN, j = 0; i < ANGLE_MAX; i++, j++)
    {
      double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.01);

#ifdef DBG
      o_angles_[j] = i;
#endif
      o_sins_[j] = (float)std::sin(rad);
      o_coss_[j] = (float)std::cos(rad);
    }

#ifdef DBG
    angles_ = o_angles_ - ANGLE_MIN;
#endif
    sins_ = o_sins_ - ANGLE_MIN;
    coss_ = o_coss_ - ANGLE_MIN;
  }

  ~Trigon()
  {
    free(o_coss_);
    free(o_sins_);
#ifdef DBG
    free(o_angles_);
#endif
  }

  float sin(int32_t angle)
  {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
    {
      angle = 0;
    }

    return sins_[angle];
  }

  float cos(int32_t angle)
  {
    if (angle < ANGLE_MIN || angle >= ANGLE_MAX)
    {
      angle = 0;
    }

    return coss_[angle];
  }

  void print()
  {
    for (int32_t i = -10; i < 10; i++)
    {
      std::cout << 
#ifdef DBG 
        angles_[i] << "\t" << 
#endif
        sins_[i] << "\t" << coss_[i] << std::endl;
    }
  }

private:
#ifdef DBG 
  int32_t* o_angles_;
  int32_t* angles_;
#endif
  float* o_sins_;
  float* o_coss_;
  float* sins_;
  float* coss_;
};

}  // namespace lidar
}  // namespace robosense
