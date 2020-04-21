/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "velodyne_pointcloud/interpolate.h"

namespace velodyne_pointcloud
{
class InterpolateNodelet : public nodelet::Nodelet
{
public:
  InterpolateNodelet() {}
  ~InterpolateNodelet() {}

private:
  virtual void onInit();
  boost::shared_ptr<Interpolate> interpolate_;
};

/** @brief Nodelet initialization. */
void InterpolateNodelet::onInit()
{
  interpolate_.reset(new Interpolate(getNodeHandle(), getPrivateNodeHandle()));
}

}  // namespace velodyne_pointcloud

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(velodyne_pointcloud::InterpolateNodelet, nodelet::Nodelet)
