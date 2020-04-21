// Copyright (C) 2007, 2009, 2010, 2012, 2015Yaxin Liu, Patrick Beeson, Austin Robot Technology, Jack O'Quin
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- base class for accessing the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef VELODYNE_DRIVER_INPUT_H
#define VELODYNE_DRIVER_INPUT_H

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <string>

#include <ros/ros.h>
#include <velodyne_msgs/VelodynePacket.h>

namespace velodyne_driver
{
static uint16_t DATA_PORT_NUMBER = 2368;      // default data port
static uint16_t POSITION_PORT_NUMBER = 8308;  // default position port

/** @brief Velodyne input base class */
class Input
{
public:
  Input(ros::NodeHandle private_nh, uint16_t port);
  virtual ~Input() {}

  /** @brief Read one Velodyne packet.
   *
   * @param pkt points to VelodynePacket message
   *
   * @returns 0 if successful,
   *          -1 if end of file
   *          > 0 if incomplete packet (is this possible?)
   */
  virtual int getPacket(velodyne_msgs::VelodynePacket *pkt,
                        const double time_offset) = 0;

  virtual void setPacketRate( const double packet_rate ) = 0; // necessary for automatic adjustment of rpm

protected:
  ros::NodeHandle private_nh_;
  uint16_t port_;
  std::string devip_str_;
  bool gps_time_;
};

/** @brief Live Velodyne input from socket. */
class InputSocket: public Input
{
public:
  InputSocket(ros::NodeHandle private_nh,
              uint16_t port = DATA_PORT_NUMBER);
  virtual ~InputSocket();

  virtual int getPacket(velodyne_msgs::VelodynePacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

  void setPacketRate( const double packet_rate ) ; // necessary for automatic adjustment of rpm

private:
  int sockfd_;
  in_addr devip_;
};


/** @brief Velodyne input from PCAP dump file.
 *
 * Dump files can be grabbed by libpcap, Velodyne's DSR software,
 * ethereal, wireshark, tcpdump, or the \ref vdump_command.
 */
class InputPCAP: public Input
{
public:
  InputPCAP(ros::NodeHandle private_nh,
            uint16_t port = DATA_PORT_NUMBER,
            double packet_rate = 0.0,
            std::string filename = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);
  virtual ~InputPCAP();

  virtual int getPacket(velodyne_msgs::VelodynePacket *pkt,
                        const double time_offset);
  void setDeviceIP(const std::string& ip);

  void setPacketRate( const double packet_rate ); // necessary for automatic adjustment of rpm

private:
  ros::Rate packet_rate_;
  ros::Duration *pwait_time;
  std::string filename_;
  pcap_t *pcap_;
  bpf_program pcap_packet_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_INPUT_H
