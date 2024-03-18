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

class AzimuthSection
{
public:
  AzimuthSection(int32_t start, int32_t end)
  {
    full_round_ = (start == 0) && (end == 36000);

    start_ = start % 36000;
    end_ = end % 36000;
    cross_zero_ = (start_ > end_);
  }

  bool in(int32_t angle)
  {
    if (full_round_)
      return true;

    if (cross_zero_)
    {
      return (angle >= start_) || (angle < end_);
    }
    else
    {
      return (angle >= start_) && (angle < end_);
    }
  }

#ifndef UNIT_TEST
private:
#endif
  bool full_round_;
  int32_t start_;
  int32_t end_;
  bool cross_zero_;
};

class DistanceSection
{
public:
  DistanceSection (float min, float max, float usr_min, float usr_max)
    : min_(min), max_(max)
  {
    if (usr_min < 0) usr_min = 0;
    if (usr_max < 0) usr_max = 0;

    if ((usr_min != 0) || (usr_max != 0))
    {
      min_ = usr_min;
      max_ = usr_max;
    }
  }

  bool in(float distance)
  {
    return ((min_ <= distance) && (distance <= max_));
  }

#ifndef UNIT_TEST
private:
#endif

  float min_;
  float max_;
};

}  // namespace lidar
}  // namespace robosense
