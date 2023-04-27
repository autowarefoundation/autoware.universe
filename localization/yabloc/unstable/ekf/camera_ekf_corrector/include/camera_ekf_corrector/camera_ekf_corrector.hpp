// Copyright 2023 TIER IV, Inc.
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

#pragma once

#include "camera_ekf_corrector/hierarchical_cost_map.hpp"

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace yabloc::ekf_corrector
{
cv::Point2f cv2pt(const Eigen::Vector3f v);
float abs_cos(const Eigen::Vector3f & t, float deg);

class CameraEkfCorrector : public rclcpp::Node
{
public:
  using LineSegment = pcl::PointXYZLNormal;
  using LineSegments = pcl::PointCloud<LineSegment>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using Float32 = std_msgs::msg::Float32;

  CameraEkfCorrector();

private:
  const float far_weight_gain_;
  const float logit_gain_;

  HierarchicalCostMap cost_map_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_bounding_box_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_line_segments_cloud_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_initialpose_;
  rclcpp::Subscription<Float32>::SharedPtr sub_height_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_pose_cov_;
  rclcpp::Publisher<PoseCovStamped>::SharedPtr pub_debug_pose_cov_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_scored_cloud_;

  std::list<PoseCovStamped> pose_buffer_;
  std::function<float(float)> score_converter_;

  Float32 latest_height_;
  bool enable_switch_{false};

  void on_line_segments(const PointCloud2 & msg);
  void on_ll2(const PointCloud2 & msg);
  void on_bounding_box(const PointCloud2 & msg);
  void on_pose_cov(const PoseCovStamped & msg);

  std::pair<LineSegments, LineSegments> split_line_segments(const PointCloud2 & msg);

  float compute_logit(const LineSegments & line_segments_cloud, const Eigen::Vector3f & self_position);

  std::pair<LineSegments, LineSegments> filt(const LineSegments & lines);
  std::optional<PoseCovStamped> get_synchronized_pose(const rclcpp::Time & stamp);

  void publish_visualize_markers(const ParticleArray & particles);

  PoseCovStamped estimate_pose_with_covariance(
    const PoseCovStamped & init, const LineSegments & line_segments_cloud,
    const LineSegments & iffy_line_segments_cloud);

  pcl::PointCloud<pcl::PointXYZI> evaluate_cloud(
    const LineSegments & line_segments_cloud, const Eigen::Vector3f & self_position);
};
}  // namespace yabloc::ekf_corrector