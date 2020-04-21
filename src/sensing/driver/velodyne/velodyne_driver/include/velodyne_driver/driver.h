// Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
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

#ifndef VELODYNE_DRIVER_DRIVER_H
#define VELODYNE_DRIVER_DRIVER_H

#include <string>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <dynamic_reconfigure/server.h>

#include <velodyne_driver/input.h>
#include <velodyne_driver/VelodyneNodeConfig.h>

namespace velodyne_driver
{

class VelodyneDriver
{
public:
  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh,
                 std::string const & node_name = ros::this_node::getName());
  ~VelodyneDriver() {}

  bool poll(void);

private:
  // Callback for dynamic reconfigure
  void callback(velodyne_driver::VelodyneNodeConfig &config,
              uint32_t level);
  // Callback for diagnostics update for lost communication with vlp
  void diagTimerCallback(const ros::TimerEvent&event);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_driver::
              VelodyneNodeConfig> > srv_;

  // configuration parameters
  struct
  {
    std::string frame_id;            // tf frame ID
    std::string model;               // device model name
    int    npackets;                 // number of packets to collect
    double rpm;                      // device rotation rate (RPMs)
    double alpha;
    int cut_angle;                   // cutting angle in 1/100°
    double time_offset;              // time in seconds added to each velodyne time stamp
    bool enabled;                    // polling is enabled
    bool timestamp_first_packet;
  }
  config_;

  boost::shared_ptr<Input> input_;
  ros::Publisher output_;
  int last_azimuth_;

  /* diagnostics updater */
  ros::Timer diag_timer_;
  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;
  /* auto rpm detection members */
  uint32_t prev_packet_toh;
  uint16_t prev_packet_azm;
  uint32_t curr_packet_toh;
  uint16_t curr_packet_azm;
  double   auto_rpm;
  double   auto_alpha;
  uint32_t auto_npackets;
  double   auto_packet_rate;
  uint8_t  curr_packet_rmode; //    [strongest return or farthest mode => Singular Retruns per firing]
                              // or [Both  => Dual Retruns per fire]
  uint8_t  curr_packet_sensor_model; // extract the sensor id from packet
  double   slot_time ;
  uint8_t  num_slots;
  uint8_t  active_slots;
  double   firing_cycle;
  std::string dump_file; // string to hold pcap file name
};

}  // namespace velodyne_driver

#endif  // VELODYNE_DRIVER_DRIVER_H
