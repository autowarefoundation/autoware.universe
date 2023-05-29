// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#ifndef VESC_DRIVER__VESC_PACKET_HPP_
#define VESC_DRIVER__VESC_PACKET_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#define CRCPP_USE_CPP11
#include "vesc_driver/crc.hpp"

namespace vesc_driver
{

typedef std::vector<uint8_t> Buffer;
typedef std::pair<Buffer::iterator, Buffer::iterator> BufferRange;
typedef std::pair<Buffer::const_iterator, Buffer::const_iterator> BufferRangeConst;

/** The raw frame for communicating with the VESC */
class VescFrame
{
public:
  virtual ~VescFrame() {}

  // getters
  virtual const Buffer & frame() const
  {
    return *frame_;
  }

  // VESC packet properties
  static const int VESC_MAX_PAYLOAD_SIZE = 1024;           ///< Maximum VESC payload size, in bytes
  static const int VESC_MIN_FRAME_SIZE = 5;                ///< Smallest VESC frame size, in bytes
  // v Largest VESC frame size, in bytes
  static const int VESC_MAX_FRAME_SIZE = 6 + VESC_MAX_PAYLOAD_SIZE;
  static const unsigned int VESC_SOF_VAL_SMALL_FRAME = 2;  ///< VESC start of "small" frame value
  static const unsigned int VESC_SOF_VAL_LARGE_FRAME = 3;  ///< VESC start of "large" frame value
  static const unsigned int VESC_EOF_VAL = 3;              ///< VESC end-of-frame value

  /** CRC parameters for the VESC */
  static constexpr CRC::Parameters<crcpp_uint16,
    16> CRC_TYPE = {0x1021, 0x0000, 0x0000, false, false};

protected:
  /** Construct frame with specified payload size. */
  explicit VescFrame(int payload_size);

  std::shared_ptr<Buffer> frame_;  ///< Stores frame data, shared_ptr for shallow copy
  BufferRange payload_;              ///< View into frame's payload section

private:
  /** Construct from buffer. Used by VescPacketFactory factory. */
  VescFrame(const BufferRangeConst & frame, const BufferRangeConst & payload);

  /** Give VescPacketFactory access to private constructor. */
  friend class VescPacketFactory;
};

/*------------------------------------------------------------------------------------------------*/

/** A VescPacket is a VescFrame with a non-zero length payload */
class VescPacket : public VescFrame
{
public:
  virtual ~VescPacket() {}

  virtual const std::string & name() const
  {
    return name_;
  }

protected:
  VescPacket(const std::string & name, int payload_size, int payload_id);
  VescPacket(const std::string & name, std::shared_ptr<VescFrame> raw);

private:
  std::string name_;
};

typedef std::shared_ptr<VescPacket> VescPacketPtr;
typedef std::shared_ptr<VescPacket const> VescPacketConstPtr;

/*------------------------------------------------------------------------------------------------*/

class VescPacketFWVersion : public VescPacket
{
public:
  explicit VescPacketFWVersion(std::shared_ptr<VescFrame> raw);

  int fwMajor() const;
  int fwMinor() const;

  std::string     hwname() const;
  const uint8_t * uuid()  const;
  bool     paired() const;
  uint8_t  devVersion() const;

private:
  int minor_;
  int major_;
  std::string hwname_;
  bool paired_;
  uint8_t uuid_[12];
  uint8_t devVersion_;
};

class VescPacketRequestFWVersion : public VescPacket
{
public:
  VescPacketRequestFWVersion();
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketValues : public VescPacket
{
public:
  explicit VescPacketValues(std::shared_ptr<VescFrame> raw);

  double  temp_fet() const;
  double  temp_motor() const;
  double  avg_motor_current() const;
  double  avg_input_current() const;
  double  avg_id() const;
  double  avg_iq() const;
  double  duty_cycle_now() const;
  double  rpm() const;
  double  duty_now() const;
  double  v_in() const;
  double  amp_hours() const;
  double  amp_hours_charged() const;
  double  watt_hours() const;
  double  watt_hours_charged() const;
  int32_t tachometer() const;
  int32_t tachometer_abs() const;
  int     fault_code() const;
  double  pid_pos_now() const;
  int32_t controller_id() const;

  double  temp_mos1() const;
  double  temp_mos2() const;
  double  temp_mos3() const;
  double  avg_vd() const;
  double  avg_vq()  const;
};

class VescPacketRequestValues : public VescPacket
{
public:
  VescPacketRequestValues();
};
/*------------------------------------------------------------------------------------------------*/

class VescPacketSetDuty : public VescPacket
{
public:
  explicit VescPacketSetDuty(double duty);

  //  double duty() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetCurrent : public VescPacket
{
public:
  explicit VescPacketSetCurrent(double current);

  //  double current() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetCurrentBrake : public VescPacket
{
public:
  explicit VescPacketSetCurrentBrake(double current_brake);

  //  double current_brake() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetRPM : public VescPacket
{
public:
  explicit VescPacketSetRPM(double rpm);

  //  double rpm() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetPos : public VescPacket
{
public:
  explicit VescPacketSetPos(double pos);

  //  double pos() const;
};

/*------------------------------------------------------------------------------------------------*/

class VescPacketSetServoPos : public VescPacket
{
public:
  explicit VescPacketSetServoPos(double servo_pos);

  //  double servo_pos() const;
};

/*------------------------------------------------------------------------------------------------*/
class VescPacketRequestImu : public VescPacket
{
public:
  VescPacketRequestImu();
};

class VescPacketImu : public VescPacket
{
public:
  explicit VescPacketImu(std::shared_ptr<VescFrame> raw);

  int    mask()  const;

  double yaw()   const;
  double pitch() const;
  double roll()  const;

  double acc_x() const;
  double acc_y() const;
  double acc_z() const;

  double gyr_x() const;
  double gyr_y() const;
  double gyr_z() const;

  double mag_x() const;
  double mag_y() const;
  double mag_z() const;

  double q_w() const;
  double q_x() const;
  double q_y() const;
  double q_z() const;

private:
  double getFloat32Auto(uint32_t * pos) const;

  uint32_t mask_;
  double roll_;
  double pitch_;
  double yaw_;

  double acc_x_;
  double acc_y_;
  double acc_z_;

  double gyr_x_;
  double gyr_y_;
  double gyr_z_;

  double mag_x_;
  double mag_y_;
  double mag_z_;

  double q0_;
  double q1_;
  double q2_;
  double q3_;
};

/*------------------------------------------------------------------------------------------------*/
}  // namespace vesc_driver

#endif  // VESC_DRIVER__VESC_PACKET_HPP_
