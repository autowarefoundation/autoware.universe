/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 1

#include <deque>
#include <string>

#include <ros/ros.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne_pointcloud/CloudNodeConfig.h>

#include <velodyne_pointcloud/pointcloudXYZIRADT.h>
#include <velodyne_pointcloud/rawdata.h>

namespace velodyne_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Convert() {}

private:
  void callback(velodyne_pointcloud::CloudNodeConfig & config, uint32_t level);
  void processScan(const velodyne_msgs::VelodyneScan::ConstPtr & scanMsg);
  visualization_msgs::MarkerArray createVelodyneModelMakerMsg(const std_msgs::Header & header);
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  ros::Subscriber velodyne_scan_;
  ros::Publisher velodyne_points_pub_;
  ros::Publisher velodyne_points_ex_pub_;
  ros::Publisher velodyne_points_invalid_near_pub_;
  ros::Publisher velodyne_points_combined_ex_pub_;
  ros::Publisher marker_array_pub_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig> > srv_;

  boost::shared_ptr<velodyne_rawdata::RawData> data_;

  int num_points_threshold_;
  std::vector<float> invalid_intensity_array_;
  std::string base_link_frame_;

  /// configuration parameters
  typedef struct
  {
    int npackets;  ///< number of packets to combine
  } Config;
  Config config_;
};

}  // namespace velodyne_pointcloud

#endif  // _VELODYNE_POINTCLOUD_CONVERT_H_
