/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/point_cloud.h"
#include "pluginlib/class_list_macros.h"
#include "pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.hpp"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "visualization_msgs/Marker.h"

namespace pointcloud_to_laserscan
{
PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet() {}

void PointCloudToLaserScanNodelet::onInit()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  private_nh_ = getPrivateNodeHandle();

  private_nh_.param<std::string>("target_frame", target_frame_, "");
  private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
  private_nh_.param<double>("min_height", min_height_, std::numeric_limits<double>::min());
  private_nh_.param<double>("max_height", max_height_, std::numeric_limits<double>::max());

  private_nh_.param<double>("angle_min", angle_min_, -M_PI);
  private_nh_.param<double>("angle_max", angle_max_, M_PI);
  private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 180.0);
  private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
  private_nh_.param<double>("range_min", range_min_, 0.0);
  private_nh_.param<double>("range_max", range_max_, std::numeric_limits<double>::max());
  private_nh_.param<double>("inf_epsilon", inf_epsilon_, 1.0);

  int concurrency_level;
  private_nh_.param<int>("concurrency_level", concurrency_level, 1);
  private_nh_.param<bool>("use_inf", use_inf_, true);

  // Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
  if (concurrency_level == 1) {
    nh_ = getNodeHandle();
  } else {
    nh_ = getMTNodeHandle();
  }

  // Only queue one pointcloud per running thread
  if (concurrency_level > 0) {
    input_queue_size_ = concurrency_level;
  } else {
    input_queue_size_ = boost::thread::hardware_concurrency();
  }

  // if pointcloud target frame specified, we need to filter by transform availability
  if (!target_frame_.empty()) {
    tf2_.reset(new tf2_ros::Buffer());
    tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
    message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
    message_filter_->registerCallback(
      boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    message_filter_->registerFailureCallback(
      boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
  } else { // otherwise setup direct subscription
    sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
  }

  laserscan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(
    "vscan/laserscan", 1, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
    boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
    "vscan/pointcloud", 1, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
    boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  ray_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(
    "vscan/ray", 1, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
    boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  stixel_viz_pub_ = nh_.advertise<visualization_msgs::Marker>(
    "vscan/stixel", 1, boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
    boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
}

void PointCloudToLaserScanNodelet::connectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (
    (laserscan_pub_.getNumSubscribers() > 0 || pointcloud_pub_.getNumSubscribers() > 0 ||
    ray_viz_pub_.getNumSubscribers() > 0 || stixel_viz_pub_.getNumSubscribers() > 0) &&
    sub_.getSubscriber().getNumPublishers() == 0)
  {
    NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
    sub_.subscribe(nh_, "cloud_in", input_queue_size_);
  }
}

void PointCloudToLaserScanNodelet::disconnectCb()
{
  boost::mutex::scoped_lock lock(connect_mutex_);
  if (
    laserscan_pub_.getNumSubscribers() == 0 && pointcloud_pub_.getNumSubscribers() == 0 &&
    ray_viz_pub_.getNumSubscribers() == 0 && stixel_viz_pub_.getNumSubscribers() == 0)
  {
    NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
    sub_.unsubscribe();
  }
}

void PointCloudToLaserScanNodelet::failureCb(
  const sensor_msgs::PointCloud2ConstPtr & cloud_msg,
  tf2_ros::filter_failure_reasons::FilterFailureReason reason)
{
  NODELET_WARN_STREAM_THROTTLE(
    1.0, "Can't transform pointcloud from frame " <<
      cloud_msg->header.frame_id << " to " << message_filter_->getTargetFramesString() <<
      " at time " << cloud_msg->header.stamp << ", reason: " << reason);
}

void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{
  // build laserscan output
  sensor_msgs::LaserScan output;
  output.header = cloud_msg->header;
  if (!target_frame_.empty()) {
    output.header.frame_id = target_frame_;
  }

  output.angle_min = angle_min_;
  output.angle_max = angle_max_;
  output.angle_increment = angle_increment_;
  output.time_increment = 0.0;
  output.scan_time = scan_time_;
  output.range_min = range_min_;
  output.range_max = range_max_;

  // build pointcloud2 output
  sensor_msgs::PointCloud2 pointcloud_output;
  const int no_data = -1;
  std::vector<int> v_pointcloud_index;

  // build ray output
  visualization_msgs::Marker ray_output;

  // build stixel output
  visualization_msgs::Marker stixel_output;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

  // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
  if (use_inf_) {
    output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
  } else {
    output.ranges.assign(ranges_size, output.range_max + inf_epsilon_);
  }

  v_pointcloud_index.assign(ranges_size, no_data);

  sensor_msgs::PointCloud2ConstPtr cloud_out;
  sensor_msgs::PointCloud2Ptr cloud;

  // Transform cloud if necessary
  if (!(output.header.frame_id == cloud_msg->header.frame_id)) {
    try {
      cloud.reset(new sensor_msgs::PointCloud2);
      tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
      cloud_out = cloud;
    } catch (tf2::TransformException & ex) {
      NODELET_ERROR_STREAM("Transform failure: " << ex.what());
      return;
    }
  } else {
    cloud_out = cloud_msg;
  }

  // Iterate through pointcloud
  int pointcloud_index = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"),
    iter_y(*cloud_out, "y"), iter_z(*cloud_out, "z");
    iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++pointcloud_index)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      NODELET_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    if (*iter_z > max_height_ || *iter_z < min_height_) {
      NODELET_DEBUG(
        "rejected for height %f not in range (%f, %f)\n", *iter_z, min_height_, max_height_);
      continue;
    }

    double range = hypot(*iter_x, *iter_y);
    if (range < range_min_) {
      NODELET_DEBUG(
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_,
        *iter_x, *iter_y, *iter_z);
      continue;
    }
    if (range > range_max_) {
      NODELET_DEBUG(
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)", range, range_max_,
        *iter_x, *iter_y, *iter_z);
      continue;
    }

    double angle = atan2(*iter_y, *iter_x);
    if (angle < output.angle_min || angle > output.angle_max) {
      NODELET_DEBUG(
        "rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
      continue;
    }

    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output.angle_min) / output.angle_increment;
    if (range < output.ranges[index]) {
      output.ranges[index] = range;
      v_pointcloud_index.at(index) = pointcloud_index;
    }
  }
  laserscan_pub_.publish(output);

  pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_out, "x"), iter_y(*cloud_out, "y"),
  iter_z(*cloud_out, "z");
  for (size_t i = 0; i < v_pointcloud_index.size(); ++i) {
    if (v_pointcloud_index.at(i) != no_data) {
      pcl::PointXYZ point;
      point.x = *(iter_x + v_pointcloud_index.at(i));
      point.y = *(iter_y + v_pointcloud_index.at(i));
      point.z = *(iter_z + v_pointcloud_index.at(i));
      pcl_pointcloud.push_back(point);
    }
  }
  pcl::toROSMsg(pcl_pointcloud, pointcloud_output);
  pointcloud_output.header = output.header;
  pointcloud_pub_.publish(pointcloud_output);

  // ray msg
  ray_output.header = output.header;
  ray_output.id = 0;
  ray_output.type = visualization_msgs::Marker::LINE_LIST;
  ray_output.action = visualization_msgs::Marker::MODIFY;
  ray_output.lifetime = ros::Duration(0.0);
  ray_output.scale.x = 0.05;
  ray_output.color.a = 0.1;  // Don't forget to set the alpha!
  ray_output.color.r = 1.0;
  ray_output.color.g = 1.0;
  ray_output.color.b = 0.0;
  for (size_t i = 0; i < pcl_pointcloud.size(); ++i) {
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    ray_output.points.push_back(point);
    point.x = pcl_pointcloud.at(i).x;
    point.y = pcl_pointcloud.at(i).y;
    point.z = pcl_pointcloud.at(i).z;
    ray_output.points.push_back(point);
  }

  ray_viz_pub_.publish(ray_output);

  // stixel msg
  stixel_output.header = output.header;
  stixel_output.id = 0;
  stixel_output.type = visualization_msgs::Marker::LINE_LIST;
  stixel_output.action = visualization_msgs::Marker::MODIFY;
  stixel_output.lifetime = ros::Duration(0.0);
  stixel_output.scale.x = 0.1;
  stixel_output.color.a = 0.5;  // Don't forget to set the alpha!
  stixel_output.color.r = 0.0;
  stixel_output.color.g = 1.0;
  stixel_output.color.b = 0.0;
  for (size_t i = 0; i < pcl_pointcloud.size(); ++i) {
    geometry_msgs::Point point;
    point.x = pcl_pointcloud.at(i).x;
    point.y = pcl_pointcloud.at(i).y;
    point.z = min_height_;
    stixel_output.points.push_back(point);
    point.x = pcl_pointcloud.at(i).x;
    point.y = pcl_pointcloud.at(i).y;
    point.z = pcl_pointcloud.at(i).z;
    stixel_output.points.push_back(point);
  }
  stixel_viz_pub_.publish(stixel_output);
}

}  // namespace pointcloud_to_laserscan

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet)
