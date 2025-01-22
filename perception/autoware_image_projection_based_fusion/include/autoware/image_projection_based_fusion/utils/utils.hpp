// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_
#define AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_

#define EIGEN_MPL2_ONLY

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include "autoware/image_projection_based_fusion/fusion_node.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "autoware_perception_msgs/msg/shape.hpp"
#include "tier4_perception_msgs/msg/detected_object_with_feature.hpp"
#include <sensor_msgs/msg/camera_info.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <optional>
#include <string>
#include <vector>

namespace autoware::image_projection_based_fusion
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

struct PointData
{
  float distance;
  size_t orig_index;
};

bool checkCameraInfo(const sensor_msgs::msg::CameraInfo & camera_info);

std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time);

Eigen::Affine3d transformToEigen(const geometry_msgs::msg::Transform & t);

void closest_cluster(
  const PointCloudMsgType & cluster, const double cluster_2d_tolerance, const int min_cluster_size,
  const pcl::PointXYZ & center, PointCloudMsgType & out_cluster);

void updateOutputFusedObjects(
  std::vector<DetectedObjectWithFeature> & output_objs, std::vector<PointCloudMsgType> & clusters,
  const std::vector<size_t> & clusters_data_size, const PointCloudMsgType & in_cloud,
  const std_msgs::msg::Header & in_roi_header, const tf2_ros::Buffer & tf_buffer,
  const int min_cluster_size, const int max_cluster_size, const float cluster_2d_tolerance,
  std::vector<DetectedObjectWithFeature> & output_fused_objects);

geometry_msgs::msg::Point getCentroid(const PointCloudMsgType & pointcloud);

}  // namespace autoware::image_projection_based_fusion

#endif  // AUTOWARE__IMAGE_PROJECTION_BASED_FUSION__UTILS__UTILS_HPP_
