/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "latlon_muxer/node.hpp"

LatLonMuxer::LatLonMuxer() : nh_(""), pnh_("~")
{
  control_cmd_pub_ =
    pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_cmd", 1, true);
  lat_control_cmd_sub_ =
    pnh_.subscribe("input/lateral/control_cmd", 1, &LatLonMuxer::latCtrlCmdCallback, this);
  lon_control_cmd_sub_ =
    pnh_.subscribe("input/longitudinal/control_cmd", 1, &LatLonMuxer::lonCtrlCmdCallback, this);
}

void LatLonMuxer::publishCmd()
{
  if (!lat_cmd_ || !lon_cmd_) {
    return;
  }

  autoware_control_msgs::ControlCommandStamped out;
  out.header.stamp = ros::Time::now();
  out.header.frame_id = "base_link";
  out.control.steering_angle = lat_cmd_->control.steering_angle;
  out.control.steering_angle_velocity = lat_cmd_->control.steering_angle_velocity;
  out.control.velocity = lon_cmd_->control.velocity;
  out.control.acceleration = lon_cmd_->control.acceleration;

  control_cmd_pub_.publish(out);
}

void LatLonMuxer::latCtrlCmdCallback(
  const autoware_control_msgs::ControlCommandStamped::ConstPtr input_msg)
{
  lat_cmd_ = std::make_shared<autoware_control_msgs::ControlCommandStamped>(*input_msg);
  publishCmd();
}

void LatLonMuxer::lonCtrlCmdCallback(
  const autoware_control_msgs::ControlCommandStamped::ConstPtr input_msg)
{
  lon_cmd_ = std::make_shared<autoware_control_msgs::ControlCommandStamped>(*input_msg);
  publishCmd();
}
