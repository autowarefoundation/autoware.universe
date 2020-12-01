// Copyright 2020 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef POSE_INITIALIZER__POSE_INITIALIZER_CORE_HPP_
#define POSE_INITIALIZER__POSE_INITIALIZER_CORE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "autoware_localization_srvs/srv/pose_with_covariance_stamped.hpp"

class PoseInitializer : public rclcpp::Node
{
public:
  PoseInitializer();
  ~PoseInitializer();

private:
  void callbackMapPoints(sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg_ptr);
  void serviceInitial(
    const std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Request> req,
    std::shared_ptr<autoware_localization_srvs::srv::PoseWithCovarianceStamped::Response> res);
  void callbackInitialPose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr);
  void callbackGNSSPoseCov(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_cov_msg_ptr);

  bool getHeight(
    const geometry_msgs::msg::PoseWithCovarianceStamped & input_pose_msg,
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr output_pose_msg_ptr);
  void callAlignServiceAndPublishResult(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gnss_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;

  rclcpp::Client<autoware_localization_srvs::srv::PoseWithCovarianceStamped>::SharedPtr ndt_client_;

  rclcpp::Service<autoware_localization_srvs::srv::PoseWithCovarianceStamped>::SharedPtr
    gnss_service_;

  tf2::BufferCore tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_;
  std::string map_frame_;

  // With the currently available facilities for calling a service, there is no
  // easy way of detecting whether an answer was received within a reasonable
  // amount of time. So, as a sanity check, we check whether a response for the
  // previous request was received when a new request is sent.
  uint32_t request_id_ = 0;
  uint32_t response_id_ = 0;
};

#endif  // POSE_INITIALIZER__POSE_INITIALIZER_CORE_HPP_
