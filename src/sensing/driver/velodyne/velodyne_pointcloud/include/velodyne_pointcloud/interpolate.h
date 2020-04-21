/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#ifndef _VELODYNE_POINTCLOUD_INTERPOLATE_H_
#define _VELODYNE_POINTCLOUD_INTERPOLATE_H_ 1

#include <deque>
#include <string>

#include <ros/ros.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

#include <velodyne_pointcloud/pointcloudXYZIRADT.h>

namespace velodyne_pointcloud
{
class Interpolate
{
public:
  Interpolate(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Interpolate() {}

private:
  void callback(velodyne_pointcloud::CloudNodeConfig & config, uint32_t level);
  void processPoints(
    const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & points_xyziradt);
  void processTwist(const geometry_msgs::TwistStamped::ConstPtr & twist_msg);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  ros::Subscriber velodyne_points_ex_sub_;
  ros::Subscriber twist_sub_;
  ros::Publisher velodyne_points_interpolate_pub_;
  ros::Publisher velodyne_points_interpolate_ex_pub_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  std::deque<geometry_msgs::TwistStamped> twist_queue_;

  std::string base_link_frame_;
};

}  // namespace velodyne_pointcloud

#endif  // _VELODYNE_POINTCLOUD_INTERPOLATE_H_
