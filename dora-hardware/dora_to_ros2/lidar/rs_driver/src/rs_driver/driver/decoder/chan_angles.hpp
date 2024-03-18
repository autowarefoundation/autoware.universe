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

#include <fstream>
#include <cmath>

namespace robosense
{
namespace lidar
{

#pragma pack(push, 1)
typedef struct
{
  uint8_t sign;
  uint16_t value;
} RSCalibrationAngle;
#pragma pack(pop)

class ChanAngles
{
public:

  ChanAngles(uint16_t chan_num)
    : chan_num_(chan_num)
  {
    vert_angles_.resize(chan_num_);
    horiz_angles_.resize(chan_num_);
    user_chans_.resize(chan_num_);
  }
  
  int loadFromFile(const std::string& angle_path)
  {
    std::vector<int32_t> vert_angles;
    std::vector<int32_t> horiz_angles;
    int ret = loadFromFile (angle_path, chan_num_, vert_angles, horiz_angles);
    if (ret < 0)
      return ret;

    if (vert_angles.size() != chan_num_)
    {
      return -1;
    }

    vert_angles_.swap(vert_angles);
    horiz_angles_.swap(horiz_angles);
    genUserChan(vert_angles_, user_chans_);
    return 0;
  }

  int loadFromDifop(const RSCalibrationAngle vert_angle_arr[], 
      const RSCalibrationAngle horiz_angle_arr[])
  {
    std::vector<int32_t> vert_angles;
    std::vector<int32_t> horiz_angles;
    int ret = 
      loadFromDifop (vert_angle_arr, horiz_angle_arr, chan_num_, vert_angles, horiz_angles);
    if (ret < 0)
      return ret;

    vert_angles_.swap(vert_angles);
    horiz_angles_.swap(horiz_angles);
    genUserChan(vert_angles_, user_chans_);
    return 0;
  }

  uint16_t toUserChan(uint16_t chan)
  {
    return user_chans_[chan];
  }

  int32_t horizAdjust(uint16_t chan, int32_t horiz)
  {
    return (horiz + horiz_angles_[chan]);
  }

  int32_t vertAdjust(uint16_t chan)
  {
    return vert_angles_[chan];
  }

  void print()
  {
    std::cout << "---------------------" << std::endl
              << "chan_num:" << chan_num_ << std::endl;

    std::cout << "vert_angle\thoriz_angle\tuser_chan" << std::endl;

    for (uint16_t chan = 0; chan < chan_num_; chan++)
    {
      std::cout << vert_angles_[chan] << "\t" 
                << horiz_angles_[chan] << "\t" 
                << user_chans_[chan] << std::endl;
    }
  }

#ifndef UNIT_TEST
private:
#endif

  static
  void genUserChan(const std::vector<int32_t>& vert_angles, std::vector<uint16_t>& user_chans)
  {
    user_chans.resize(vert_angles.size());

    for (size_t i = 0; i < vert_angles.size(); i++)
    {
      int32_t angle = vert_angles[i];
      uint16_t chan = 0;

      for (size_t j = 0; j < vert_angles.size(); j++)
      {
        if (vert_angles[j] < angle)
        {
          chan++;
        }
      }

      user_chans[i] = chan;
    }
  }

  static int loadFromFile(const std::string& angle_path, size_t size,
      std::vector<int32_t>& vert_angles, std::vector<int32_t>& horiz_angles)
  {
    vert_angles.clear();
    horiz_angles.clear();

    std::ifstream fd(angle_path.c_str(), std::ios::in);
    if (!fd.is_open())
    {
      std::cout << "fail to open angle file:" << angle_path << std::endl;
      return -1;
    }

    std::string line;
    for (size_t i = 0; i < size; i++)
    {
      if (!std::getline(fd, line))
        return -1;

      float vert = std::stof(line);

      float horiz = 0;
      size_t pos_comma = line.find_first_of(',');
      if (pos_comma != std::string::npos)
      {
        horiz = std::stof(line.substr(pos_comma+1));
      }

      vert_angles.emplace_back(static_cast<int32_t>(vert * 100));
      horiz_angles.emplace_back(static_cast<int32_t>(horiz * 100));
    }

    fd.close();
    return 0;
  }

  static int loadFromDifop(const RSCalibrationAngle* vert_angle_arr, 
      const RSCalibrationAngle* horiz_angle_arr, size_t size, 
      std::vector<int32_t>& vert_angles, std::vector<int32_t>& horiz_angles)
  {
    vert_angles.clear();
    horiz_angles.clear();

    for (size_t i = 0; i < size; i++)
    {
      const RSCalibrationAngle& vert = vert_angle_arr[i];
      const RSCalibrationAngle& horiz = horiz_angle_arr[i];
      int32_t v;

      if (vert.sign == 0xFF)
        return -1;

      v = ntohs(vert.value);
      if (vert.sign != 0) v = -v;
      vert_angles.emplace_back(v);

      if (!angleCheck (v))
        return -1;

      v = ntohs(horiz.value);
      if (horiz.sign != 0) v = -v;
      horiz_angles.emplace_back(v);

      if (!angleCheck (v))
        return -1;

    }

    return ((vert_angles.size() > 0) ? 0 : -1);
  }

  static bool angleCheck(int32_t v)
  {
    return ((-9000 <= v) && (v < 9000));
  }

  uint16_t chan_num_;
  std::vector<int32_t> vert_angles_;
  std::vector<int32_t> horiz_angles_;
  std::vector<uint16_t> user_chans_;
};

}  // namespace lidar
}  // namespace robosense
