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

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <boost/thread.hpp>
#include <string>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "velodyne_driver/driver.h"

namespace velodyne_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet() : running_(false) {}

  ~DriverNodelet()
  {
    if (running_) {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;  ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<VelodyneDriver> dvr_;  ///< driver implementation class
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new VelodyneDriver(getNodeHandle(), getPrivateNodeHandle(), getName()));

  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>(
    new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while (ros::ok()) {
    // poll device until end of file
    running_ = dvr_->poll();
    if (!running_) ROS_ERROR_THROTTLE(1.0, "DriverNodelet::devicePoll - Failed to poll device.");
  }
  running_ = false;
}

}  // namespace velodyne_driver

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_driver::DriverNodelet, nodelet::Nodelet)
