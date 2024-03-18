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

#include <rs_driver/driver/decoder/decoder.hpp>
#include <rs_driver/driver/decoder/decoder_RS16.hpp>
#include <rs_driver/driver/decoder/decoder_RS32.hpp>
#include <rs_driver/driver/decoder/decoder_RSBP.hpp>
#include <rs_driver/driver/decoder/decoder_RSHELIOS.hpp>
#include <rs_driver/driver/decoder/decoder_RSHELIOS_16P.hpp>
#include <rs_driver/driver/decoder/decoder_RS128.hpp>
#include <rs_driver/driver/decoder/decoder_RS80.hpp>
#include <rs_driver/driver/decoder/decoder_RS48.hpp>
#include <rs_driver/driver/decoder/decoder_RSP128.hpp>
#include <rs_driver/driver/decoder/decoder_RSP80.hpp>
#include <rs_driver/driver/decoder/decoder_RSP48.hpp>
#include <rs_driver/driver/decoder/decoder_RSM1.hpp>
#include <rs_driver/driver/decoder/decoder_RSM2.hpp>
#include <rs_driver/driver/decoder/decoder_RSE1.hpp>
#include <rs_driver/driver/decoder/decoder_RSM1_Jumbo.hpp>

namespace robosense
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderFactory
{
public:

  static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(
      LidarType type, const RSDecoderParam& param);
};

template <typename T_PointCloud>
inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(
    LidarType type, const RSDecoderParam& param)
{
  std::shared_ptr<Decoder<T_PointCloud>> ret_ptr;

  switch (type)
  {
    case LidarType::RS16:
      ret_ptr = std::make_shared<DecoderRS16<T_PointCloud>>(param);
      break;
    case LidarType::RS32:
      ret_ptr = std::make_shared<DecoderRS32<T_PointCloud>>(param);
      break;
    case LidarType::RSBP:
      ret_ptr = std::make_shared<DecoderRSBP<T_PointCloud>>(param);
      break;
    case LidarType::RSHELIOS:
      ret_ptr = std::make_shared<DecoderRSHELIOS<T_PointCloud>>(param);
      break;
    case LidarType::RSHELIOS_16P:
      ret_ptr = std::make_shared<DecoderRSHELIOS_16P<T_PointCloud>>(param);
      break;
    case LidarType::RS128:
      ret_ptr = std::make_shared<DecoderRS128<T_PointCloud>>(param);
      break;
    case LidarType::RS80:
      ret_ptr = std::make_shared<DecoderRS80<T_PointCloud>>(param);
      break;
    case LidarType::RS48:
      ret_ptr = std::make_shared<DecoderRS48<T_PointCloud>>(param);
      break;
    case LidarType::RSP128:
      ret_ptr = std::make_shared<DecoderRSP128<T_PointCloud>>(param);
      break;
    case LidarType::RSP80:
      ret_ptr = std::make_shared<DecoderRSP80<T_PointCloud>>(param);
      break;
    case LidarType::RSP48:
      ret_ptr = std::make_shared<DecoderRSP48<T_PointCloud>>(param);
      break;
    case LidarType::RSM1:
      ret_ptr = std::make_shared<DecoderRSM1<T_PointCloud>>(param);
      break;
    case LidarType::RSM2:
      ret_ptr = std::make_shared<DecoderRSM2<T_PointCloud>>(param);
      break;
    case LidarType::RSE1:
      ret_ptr = std::make_shared<DecoderRSE1<T_PointCloud>>(param);
      break;
    case LidarType::RSM1_JUMBO:
      ret_ptr = std::make_shared<DecoderRSM1_Jumbo<T_PointCloud>>(param);
      break;
    default:
      RS_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << RS_REND;
      exit(-1);
  }

  return ret_ptr;
}

}  // namespace lidar
}  // namespace robosense
