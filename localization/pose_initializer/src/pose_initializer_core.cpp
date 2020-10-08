/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pose_initializer/pose_initializer_core.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_conversions/pcl_conversions.h>

double getGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap, const tf2::Vector3 & point)
{
  constexpr double radius = 1.0 * 1.0;
  const double x = point.getX();
  const double y = point.getY();

  double height = INFINITY;
  for (const auto & p : pcdmap->points) {
    const double dx = x - p.x;
    const double dy = y - p.y;
    const double sd = (dx * dx) + (dy * dy);
    if (sd < radius) {
      height = std::min(height, static_cast<double>(p.z));
    }
  }
  return std::isfinite(height) ? height : point.getZ();
}

PoseInitializer::PoseInitializer(ros::NodeHandle nh, ros::NodeHandle private_nh)
: nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_), map_frame_("map")
{
  initial_pose_sub_ = nh_.subscribe("initialpose", 10, &PoseInitializer::callbackInitialPose, this);
  map_points_sub_ = nh_.subscribe("pointcloud_map", 1, &PoseInitializer::callbackMapPoints, this);

  bool use_first_gnss_topic = true;
  private_nh_.getParam("use_first_gnss_topic", use_first_gnss_topic);
  if (use_first_gnss_topic) {
    gnss_pose_sub_ = nh_.subscribe("gnss_pose_cov", 1, &PoseInitializer::callbackGNSSPoseCov, this);
  }

  initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose3d", 10);

  ndt_client_ =
    nh_.serviceClient<autoware_localization_srvs::PoseWithCovarianceStamped>("ndt_align_srv");
  ndt_client_.waitForExistence(ros::Duration(1.0));  // TODO

  gnss_service_ =
    nh.advertiseService("pose_initializer_srv", &PoseInitializer::serviceInitial, this);
}

PoseInitializer::~PoseInitializer() {}

void PoseInitializer::callbackMapPoints(
  const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr)
{
  std::string map_frame_ = map_points_msg_ptr->header.frame_id;
  map_ptr_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_ptr_);
}

bool PoseInitializer::serviceInitial(
  autoware_localization_srvs::PoseWithCovarianceStamped::Request & req,
  autoware_localization_srvs::PoseWithCovarianceStamped::Response & res)
{
  gnss_pose_sub_.shutdown();  // get only first topic

  geometry_msgs::PoseWithCovarianceStamped::Ptr add_height_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  getHeight(req.pose_with_cov, add_height_pose_msg_ptr);

  // TODO
  add_height_pose_msg_ptr->pose.covariance[0] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[1 * 6 + 1] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[2 * 6 + 2] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[3 * 6 + 3] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[4 * 6 + 4] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[5 * 6 + 5] = 1.0;

  geometry_msgs::PoseWithCovarianceStamped::Ptr aligned_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  const bool succeeded_align = callAlignService(*add_height_pose_msg_ptr, aligned_pose_msg_ptr);

  if (succeeded_align) {
    initial_pose_pub_.publish(*aligned_pose_msg_ptr);
    return true;
  } else {
    return false;
  }
}

void PoseInitializer::callbackInitialPose(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_cov_msg_ptr)
{
  gnss_pose_sub_.shutdown();  // get only first topic

  geometry_msgs::PoseWithCovarianceStamped::Ptr add_height_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  // TODO
  add_height_pose_msg_ptr->pose.covariance[0] = 2.0;
  add_height_pose_msg_ptr->pose.covariance[1 * 6 + 1] = 2.0;
  add_height_pose_msg_ptr->pose.covariance[2 * 6 + 2] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[3 * 6 + 3] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[4 * 6 + 4] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[5 * 6 + 5] = 0.3;

  geometry_msgs::PoseWithCovarianceStamped::Ptr aligned_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  const bool succeeded_align = callAlignService(*add_height_pose_msg_ptr, aligned_pose_msg_ptr);

  if (succeeded_align) {
    initial_pose_pub_.publish(*aligned_pose_msg_ptr);
  }
}

// NOTE Still not usable callback
void PoseInitializer::callbackGNSSPoseCov(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_cov_msg_ptr)
{
  gnss_pose_sub_.shutdown();  // get only first topic

  // TODO check service is available

  geometry_msgs::PoseWithCovarianceStamped::Ptr add_height_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  getHeight(*pose_cov_msg_ptr, add_height_pose_msg_ptr);

  // TODO
  add_height_pose_msg_ptr->pose.covariance[0] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[1 * 6 + 1] = 1.0;
  add_height_pose_msg_ptr->pose.covariance[2 * 6 + 2] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[3 * 6 + 3] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[4 * 6 + 4] = 0.01;
  add_height_pose_msg_ptr->pose.covariance[5 * 6 + 5] = 3.14;

  geometry_msgs::PoseWithCovarianceStamped::Ptr aligned_pose_msg_ptr(
    new geometry_msgs::PoseWithCovarianceStamped);
  const bool succeeded_align = callAlignService(*add_height_pose_msg_ptr, aligned_pose_msg_ptr);

  if (succeeded_align) {
    initial_pose_pub_.publish(*aligned_pose_msg_ptr);
  }
}

bool PoseInitializer::getHeight(
  const geometry_msgs::PoseWithCovarianceStamped & input_pose_msg,
  const geometry_msgs::PoseWithCovarianceStamped::Ptr & output_pose_msg_ptr)
{
  std::string fixed_frame = input_pose_msg.header.frame_id;
  tf2::Vector3 point(
    input_pose_msg.pose.pose.position.x, input_pose_msg.pose.pose.position.y,
    input_pose_msg.pose.pose.position.z);

  if (map_ptr_) {
    tf2::Transform transform;
    try {
      const auto stamped =
        tf2_buffer_.lookupTransform(map_frame_, fixed_frame, ros::Time(0), ros::Duration(1.0));
      tf2::fromMsg(stamped.transform, transform);
    } catch (tf2::TransformException & exception) {
      ROS_WARN_STREAM("failed to lookup transform: " << exception.what());
    }

    point = transform * point;
    point.setZ(getGroundHeight(map_ptr_, point));
    point = transform.inverse() * point;
  }

  *output_pose_msg_ptr = input_pose_msg;
  output_pose_msg_ptr->pose.pose.position.x = point.getX();
  output_pose_msg_ptr->pose.pose.position.y = point.getY();
  output_pose_msg_ptr->pose.pose.position.z = point.getZ();

  return true;
}

bool PoseInitializer::callAlignService(
  const geometry_msgs::PoseWithCovarianceStamped & input_pose_msg,
  const geometry_msgs::PoseWithCovarianceStamped::Ptr & output_pose_msg_ptr)
{
  autoware_localization_srvs::PoseWithCovarianceStamped srv;
  srv.request.pose_with_cov = input_pose_msg;

  ROS_INFO("[pose_initializer] call NDT Align Server");
  if (ndt_client_.call(srv)) {
    ROS_INFO("[pose_initializer] called NDT Align Server");
    // NOTE temporary cov
    srv.response.pose_with_cov.pose.covariance[0] = 1.0;
    srv.response.pose_with_cov.pose.covariance[1 * 6 + 1] = 1.0;
    srv.response.pose_with_cov.pose.covariance[2 * 6 + 2] = 0.01;
    srv.response.pose_with_cov.pose.covariance[3 * 6 + 3] = 0.01;
    srv.response.pose_with_cov.pose.covariance[4 * 6 + 4] = 0.01;
    srv.response.pose_with_cov.pose.covariance[5 * 6 + 5] = 0.2;
    *output_pose_msg_ptr = srv.response.pose_with_cov;
    return true;
  } else {
    ROS_ERROR("[pose_initializer] could not call NDT Align Server");
    return false;
  }
}
