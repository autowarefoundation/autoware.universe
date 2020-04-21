//==============================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==============================================================================

#ifndef UBLOX_SERIALIZATION_UBLOX_MSGS_H
#define UBLOX_SERIALIZATION_UBLOX_MSGS_H

#include <ros/console.h>
#include <ublox/serialization.h>
#include <ublox_msgs/ublox_msgs.h>

///
/// This file declares custom serializers for u-blox messages with dynamic
/// lengths and messages where the get/set messages have different sizes, but
/// share the same parameters, such as CfgDAT.
///

namespace ublox {

///
/// @brief Serializes the CfgDAT message which has a different length for
/// get/set.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::CfgDAT_<ContainerAllocator> > {
  typedef boost::call_traits<ublox_msgs::CfgDAT_<ContainerAllocator> > CallTraits;
  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.datumNum);
    stream.next(m.datumName);
    stream.next(m.majA);
    stream.next(m.flat);
    stream.next(m.dX);
    stream.next(m.dY);
    stream.next(m.dZ);
    stream.next(m.rotX);
    stream.next(m.rotY);
    stream.next(m.rotZ);
    stream.next(m.scale);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) {
    // this is the size of CfgDAT set messages
    // serializedLength is only used for writes so this is ok
    return 44;
  }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    // ignores datumNum & datumName
    stream.next(m.majA);
    stream.next(m.flat);
    stream.next(m.dX);
    stream.next(m.dY);
    stream.next(m.dZ);
    stream.next(m.rotX);
    stream.next(m.rotY);
    stream.next(m.rotZ);
    stream.next(m.scale);
  }
};

///
/// @brief Serializes the CfgGNSS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::CfgGNSS_<ContainerAllocator> > {
  typedef ublox_msgs::CfgGNSS_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.msgVer);
    stream.next(m.numTrkChHw);
    stream.next(m.numTrkChUse);
    stream.next(m.numConfigBlocks);
    m.blocks.resize(m.numConfigBlocks);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) ros::serialization::deserialize(stream, m.blocks[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 4 + 8 * m.numConfigBlocks; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.blocks.size() != m.numConfigBlocks) {
      ROS_ERROR("CfgGNSS numConfigBlocks must equal blocks size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.msgVer);
    stream.next(m.numTrkChHw);
    stream.next(m.numTrkChUse);
    stream.next(static_cast<typename Msg::_numConfigBlocks_type>(m.blocks.size()));
    for (std::size_t i = 0; i < m.blocks.size(); ++i) ros::serialization::serialize(stream, m.blocks[i]);
  }
};

///
/// @brief Serializes the CfgInf message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::CfgINF_<ContainerAllocator> > {
  typedef boost::call_traits<ublox_msgs::CfgINF_<ContainerAllocator> > CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    int num_blocks = count / 10;
    m.blocks.resize(num_blocks);
    for (std::size_t i = 0; i < num_blocks; ++i) ros::serialization::deserialize(stream, m.blocks[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 10 * m.blocks.size(); }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) ros::serialization::serialize(stream, m.blocks[i]);
  }
};

///
/// @brief Serializes the Inf message which has a dynamic length string.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::Inf_<ContainerAllocator> > {
  typedef boost::call_traits<ublox_msgs::Inf_<ContainerAllocator> > CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    m.str.resize(count);
    for (int i = 0; i < count; ++i) ros::serialization::deserialize(stream, m.str[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return m.str.size(); }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    for (std::size_t i = 0; i < m.str.size(); ++i) ros::serialization::serialize(stream, m.str[i]);
  }
};

///
/// @brief Serializes the MonVER message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::MonVER_<ContainerAllocator> > {
  typedef ublox_msgs::MonVER_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.swVersion);
    stream.next(m.hwVersion);

    m.extension.clear();
    int N = (count - 40) / 30;
    m.extension.reserve(N);
    typename Msg::_extension_type::value_type ext;
    for (int i = 0; i < N; i++) {
      // Read each extension string
      stream.next(ext);
      m.extension.push_back(ext);
    }
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 40 + (30 * m.extension.size()); }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.swVersion);
    stream.next(m.hwVersion);
    for (std::size_t i = 0; i < m.extension.size(); ++i) ros::serialization::serialize(stream, m.extension[i]);
  }
};

///
/// @brief Serializes the NavDGPS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavDGPS_<ContainerAllocator> > {
  typedef ublox_msgs::NavDGPS_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.iTOW);
    stream.next(m.age);
    stream.next(m.baseId);
    stream.next(m.baseHealth);
    stream.next(m.numCh);
    stream.next(m.status);
    stream.next(m.reserved1);
    m.sv.resize(m.numCh);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 16 + 12 * m.numCh; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sv.size() != m.numCh) {
      ROS_ERROR("NavDGPS numCh must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.age);
    stream.next(m.baseId);
    stream.next(m.baseHealth);
    stream.next(static_cast<typename Msg::_numCh_type>(m.sv.size()));
    stream.next(m.status);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

///
/// @brief Serializes the NavSBAS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSBAS_<ContainerAllocator> > {
  typedef ublox_msgs::NavSBAS_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.iTOW);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(m.cnt);
    stream.next(m.reserved0);
    m.sv.resize(m.cnt);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 12 + 12 * m.cnt; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sv.size() != m.cnt) {
      ROS_ERROR("NavSBAS cnt must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.geo);
    stream.next(m.mode);
    stream.next(m.sys);
    stream.next(m.service);
    stream.next(static_cast<typename Msg::_cnt_type>(m.sv.size()));
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

///
/// @brief Serializes the NavSAT message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSAT_<ContainerAllocator> > {
  typedef ublox_msgs::NavSAT_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.iTOW);
    stream.next(m.version);
    stream.next(m.numSvs);
    stream.next(m.reserved0);
    m.sv.resize(m.numSvs);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + 12 * m.numSvs; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sv.size() != m.numSvs) {
      ROS_ERROR("NavSAT numSvs must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.version);
    stream.next(static_cast<typename Msg::_numSvs_type>(m.sv.size()));
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

///
/// @brief Serializes the NavDGPS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::NavSVINFO_<ContainerAllocator> > {
  typedef ublox_msgs::NavSVINFO_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.iTOW);
    stream.next(m.numCh);
    stream.next(m.globalFlags);
    stream.next(m.reserved2);
    m.sv.resize(m.numCh);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + 12 * m.numCh; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sv.size() != m.numCh) {
      ROS_ERROR("NavSVINFO numCh must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(static_cast<typename Msg::_numCh_type>(m.sv.size()));
    stream.next(m.globalFlags);
    stream.next(m.reserved2);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

///
/// @brief Serializes the RxmRAW message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmRAW_<ContainerAllocator> > {
  typedef ublox_msgs::RxmRAW_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.rcvTOW);
    stream.next(m.week);
    stream.next(m.numSV);
    stream.next(m.reserved1);
    m.sv.resize(m.numSV);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + 24 * m.numSV; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sv.size() != m.numSV) {
      ROS_ERROR("RxmRAW numSV must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.rcvTOW);
    stream.next(m.week);
    stream.next(static_cast<typename Msg::_numSV_type>(m.sv.size()));
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

///
/// @brief Serializes the RxmRAWX message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmRAWX_<ContainerAllocator> > {
  typedef ublox_msgs::RxmRAWX_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.rcvTOW);
    stream.next(m.week);
    stream.next(m.leapS);
    stream.next(m.numMeas);
    stream.next(m.recStat);
    stream.next(m.version);
    stream.next(m.reserved1);
    m.meas.resize(m.numMeas);
    for (std::size_t i = 0; i < m.meas.size(); ++i) ros::serialization::deserialize(stream, m.meas[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 16 + 32 * m.numMeas; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.meas.size() != m.numMeas) {
      ROS_ERROR("RxmRAWX numMeas must equal meas size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.rcvTOW);
    stream.next(m.week);
    stream.next(m.leapS);
    stream.next(static_cast<typename Msg::_numMeas_type>(m.meas.size()));
    stream.next(m.recStat);
    stream.next(m.version);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.meas.size(); ++i) ros::serialization::serialize(stream, m.meas[i]);
  }
};

///
/// @brief Serializes the RxmSFRBX message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmSFRBX_<ContainerAllocator> > {
  typedef ublox_msgs::RxmSFRBX_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.gnssId);
    stream.next(m.svId);
    stream.next(m.reserved0);
    stream.next(m.freqId);
    stream.next(m.numWords);
    stream.next(m.chn);
    stream.next(m.version);
    stream.next(m.reserved1);
    m.dwrd.resize(m.numWords);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) ros::serialization::deserialize(stream, m.dwrd[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + 4 * m.numWords; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.dwrd.size() != m.numWords) {
      ROS_ERROR("RxmSFRBX numWords must equal dwrd size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.gnssId);
    stream.next(m.svId);
    stream.next(m.reserved0);
    stream.next(m.freqId);
    stream.next(static_cast<typename Msg::_numWords_type>(m.dwrd.size()));
    stream.next(m.chn);
    stream.next(m.version);
    stream.next(m.reserved1);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) ros::serialization::serialize(stream, m.dwrd[i]);
  }
};

///
/// @brief Serializes the RxmSVSI message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmSVSI_<ContainerAllocator> > {
  typedef ublox_msgs::RxmSVSI_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.iTOW);
    stream.next(m.week);
    stream.next(m.numVis);
    stream.next(m.numSV);
    m.sv.resize(m.numSV);
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::deserialize(stream, m.sv[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + 6 * m.numSV; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sv.size() != m.numSV) {
      ROS_ERROR("RxmSVSI numSV must equal sv size");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.week);
    stream.next(m.numVis);
    stream.next(static_cast<typename Msg::_numSV_type>(m.sv.size()));
    for (std::size_t i = 0; i < m.sv.size(); ++i) ros::serialization::serialize(stream, m.sv[i]);
  }
};

///
/// @brief Serializes the RxmALM message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmALM_<ContainerAllocator> > {
  typedef ublox_msgs::RxmALM_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    if (count == 40) {
      typename Msg::_dwrd_type::value_type temp;
      m.dwrd.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    }
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + (4 * m.dwrd.size()); }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) ros::serialization::serialize(stream, m.dwrd[i]);
  }
};

///
/// @brief Serializes the RxmEPH message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::RxmEPH_<ContainerAllocator> > {
  typedef ublox_msgs::RxmEPH_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    if (count == 104) {
      typename Msg::_sf1d_type::value_type temp1;
      typename Msg::_sf2d_type::value_type temp2;
      typename Msg::_sf3d_type::value_type temp3;

      m.sf1d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp1);
        m.sf1d.push_back(temp1);
      }
      m.sf2d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp2);
        m.sf2d.push_back(temp2);
      }
      m.sf3d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp3);
        m.sf3d.push_back(temp3);
      }
    }
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for (std::size_t i = 0; i < m.sf1d.size(); ++i) ros::serialization::serialize(stream, m.sf1d[i]);
    for (std::size_t i = 0; i < m.sf2d.size(); ++i) ros::serialization::serialize(stream, m.sf2d[i]);
    for (std::size_t i = 0; i < m.sf3d.size(); ++i) ros::serialization::serialize(stream, m.sf3d[i]);
  }
};

///
/// @brief Serializes the AidALM message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::AidALM_<ContainerAllocator> > {
  typedef ublox_msgs::AidALM_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.svid);
    stream.next(m.week);

    m.dwrd.clear();
    if (count == 40) {
      typename Msg::_dwrd_type::value_type temp;
      m.dwrd.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp);
        m.dwrd.push_back(temp);
      }
    }
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 8 + (4 * m.dwrd.size()); }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.week);
    for (std::size_t i = 0; i < m.dwrd.size(); ++i) ros::serialization::serialize(stream, m.dwrd[i]);
  }
};

///
/// @brief Serializes the AidEPH message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::AidEPH_<ContainerAllocator> > {
  typedef ublox_msgs::AidEPH_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.svid);
    stream.next(m.how);
    m.sf1d.clear();
    m.sf2d.clear();
    m.sf3d.clear();

    if (count == 104) {
      typename Msg::_sf1d_type::value_type temp1;
      typename Msg::_sf2d_type::value_type temp2;
      typename Msg::_sf3d_type::value_type temp3;
      m.sf1d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp1);
        m.sf1d.push_back(temp1);
      }
      m.sf2d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp2);
        m.sf2d.push_back(temp2);
      }
      m.sf3d.resize(8);
      for (std::size_t i = 0; i < 8; ++i) {
        stream.next(temp3);
        m.sf3d.push_back(temp3);
      }
    }
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) {
    return 8 + (4 * m.sf1d.size()) + (4 * m.sf2d.size()) + (4 * m.sf3d.size());
  }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.svid);
    stream.next(m.how);
    for (std::size_t i = 0; i < m.sf1d.size(); ++i) ros::serialization::serialize(stream, m.sf1d[i]);
    for (std::size_t i = 0; i < m.sf2d.size(); ++i) ros::serialization::serialize(stream, m.sf2d[i]);
    for (std::size_t i = 0; i < m.sf3d.size(); ++i) ros::serialization::serialize(stream, m.sf3d[i]);
  }
};

///
/// @brief Serializes the EsfMEAS message which has a repeated block and an
/// optional block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::EsfMEAS_<ContainerAllocator> > {
  typedef boost::call_traits<ublox_msgs::EsfMEAS_<ContainerAllocator> > CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.timeTag);
    stream.next(m.flags);
    stream.next(m.id);

    bool calib_valid = m.flags & m.FLAGS_CALIB_T_TAG_VALID;
    int data_size = (count - (calib_valid ? 12 : 8)) / 4;
    // Repeating block
    m.data.resize(data_size);
    for (std::size_t i = 0; i < data_size; ++i) ros::serialization::deserialize(stream, m.data[i]);
    // Optional block
    if (calib_valid) {
      m.calibTtag.resize(1);
      ros::serialization::deserialize(stream, m.calibTtag[0]);
    }
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) {
    return 4 + 8 * m.data.size() + 4 * m.calibTtag.size();
  }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.timeTag);
    stream.next(m.flags);
    stream.next(m.id);
    for (std::size_t i = 0; i < m.data.size(); ++i) ros::serialization::serialize(stream, m.data[i]);
    for (std::size_t i = 0; i < m.calibTtag.size(); ++i) ros::serialization::serialize(stream, m.calibTtag[i]);
  }
};

///
/// @brief Serializes the EsfRAW message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::EsfRAW_<ContainerAllocator> > {
  typedef boost::call_traits<ublox_msgs::EsfRAW_<ContainerAllocator> > CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.reserved0);
    m.blocks.clear();
    int num_blocks = (count - 4) / 8;
    m.blocks.resize(num_blocks);
    for (std::size_t i = 0; i < num_blocks; ++i) ros::serialization::deserialize(stream, m.blocks[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 4 + 8 * m.blocks.size(); }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    ros::serialization::OStream stream(data, size);
    stream.next(m.reserved0);
    for (std::size_t i = 0; i < m.blocks.size(); ++i) ros::serialization::serialize(stream, m.blocks[i]);
  }
};

///
/// @brief Serializes the EsfSTATUS message which has a repeated block.
///
template <typename ContainerAllocator>
struct Serializer<ublox_msgs::EsfSTATUS_<ContainerAllocator> > {
  typedef ublox_msgs::EsfSTATUS_<ContainerAllocator> Msg;
  typedef boost::call_traits<Msg> CallTraits;

  static void read(const uint8_t* data, uint32_t count, typename CallTraits::reference m) {
    ros::serialization::IStream stream(const_cast<uint8_t*>(data), count);
    stream.next(m.iTOW);
    stream.next(m.version);
    stream.next(m.fusionMode);
    stream.next(m.reserved2);
    stream.next(m.numSens);
    m.sens.resize(m.numSens);
    for (std::size_t i = 0; i < m.sens.size(); ++i) ros::serialization::deserialize(stream, m.sens[i]);
  }

  static uint32_t serializedLength(typename CallTraits::param_type m) { return 16 + 4 * m.numSens; }

  static void write(uint8_t* data, uint32_t size, typename CallTraits::param_type m) {
    if (m.sens.size() != m.numSens) {
      ROS_ERROR("Writing EsfSTATUS message: numSens must equal size of sens");
    }
    ros::serialization::OStream stream(data, size);
    stream.next(m.iTOW);
    stream.next(m.version);
    stream.next(m.fusionMode);
    stream.next(m.reserved2);
    stream.next(static_cast<typename Msg::_numSens_type>(m.sens.size()));
    for (std::size_t i = 0; i < m.sens.size(); ++i) ros::serialization::serialize(stream, m.sens[i]);
  }
};

}  // namespace ublox

#endif  // UBLOX_SERIALIZATION_UBLOX_MSGS_H
