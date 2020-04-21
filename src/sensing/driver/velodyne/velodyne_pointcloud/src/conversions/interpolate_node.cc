/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <ros/ros.h>
#include "velodyne_pointcloud/interpolate.h"

/** Main node entry point. */
int main(int argc, char ** argv)
{
  ros::init(argc, argv, "interpolate_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  velodyne_pointcloud::Interpolate interpolate(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}
