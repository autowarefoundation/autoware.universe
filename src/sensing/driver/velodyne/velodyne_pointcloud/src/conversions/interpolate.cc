/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <velodyne_pointcloud/interpolate.h>

#include <tf/tf.h>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>

#include <velodyne_pointcloud/func.h>

namespace velodyne_pointcloud
{
/** @brief Constructor. */
Interpolate::Interpolate(ros::NodeHandle node, ros::NodeHandle private_nh)
: tf2_listener_(tf2_buffer_), base_link_frame_("base_link")
{
  // advertise
  velodyne_points_interpolate_pub_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne_points_interpolate", 10);
  velodyne_points_interpolate_ex_pub_ =
    node.advertise<sensor_msgs::PointCloud2>("velodyne_points_interpolate_ex", 10);

  // subscribe
  twist_sub_ = node.subscribe(
    "/vehicle/status/twist", 10, &Interpolate::processTwist, (Interpolate *)this,
    ros::TransportHints().tcpNoDelay(true));
  velodyne_points_ex_sub_ = node.subscribe(
    "velodyne_points_ex", 10, &Interpolate::processPoints, (Interpolate *)this,
    ros::TransportHints().tcpNoDelay(true));
}

void Interpolate::processTwist(const geometry_msgs::TwistStamped::ConstPtr & twist_msg)
{
  twist_queue_.push_back(*twist_msg);

  while (!twist_queue_.empty()) {
    //for replay rosbag
    if (twist_queue_.front().header.stamp > twist_msg->header.stamp) {
      twist_queue_.pop_front();
    } else if (twist_queue_.front().header.stamp < twist_msg->header.stamp - ros::Duration(1.0)) {
      twist_queue_.pop_front();
    } else {
      break;
    }
  }
}

void Interpolate::processPoints(
  const pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::ConstPtr & points_xyziradt)
{
  if (
    velodyne_points_interpolate_pub_.getNumSubscribers() <= 0 &&
    velodyne_points_interpolate_ex_pub_.getNumSubscribers() <= 0) {
    return;
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  tf2::Transform tf2_base_link_to_sensor;
  getTransform(points_xyziradt->header.frame_id, base_link_frame_, &tf2_base_link_to_sensor);
  interpolate_points_xyziradt = interpolate(points_xyziradt, twist_queue_, tf2_base_link_to_sensor);

  if (velodyne_points_interpolate_pub_.getNumSubscribers() > 0) {
    const auto interpolate_points_xyzir = convert(interpolate_points_xyziradt);
    velodyne_points_interpolate_pub_.publish(interpolate_points_xyzir);
  }
  if (velodyne_points_interpolate_ex_pub_.getNumSubscribers() > 0) {
    velodyne_points_interpolate_ex_pub_.publish(interpolate_points_xyziradt);
  }
}

bool Interpolate::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
    return false;
  }
  return true;
}

}  // namespace velodyne_pointcloud
