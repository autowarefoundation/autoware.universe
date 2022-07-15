#pragma once
#include "particle_filter/hierarchical_cost_map.hpp"

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace particle_filter
{
class CameraParticleCorrector : public AbstCorrector
{
public:
  using LineSegment = pcl::PointCloud<pcl::PointNormal>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::Pose;
  CameraParticleCorrector();

private:
  void lsdCallback(const PointCloud2 & msg);
  void ll2Callback(const PointCloud2 & msg);
  void poseCallback(const PoseStamped & msg);

  float computeScore(const LineSegment & lsd_cloud, const Eigen::Vector3f & self_position);

  pcl::PointCloud<pcl::PointXYZI> evaluateCloud(
    const LineSegment & lsd_cloud, const Eigen::Vector3f & self_position);

  LineSegment transformCloud(const LineSegment & src, const Eigen::Affine3f & transform) const;
  Pose meanParticles(const ParticleArray & particle_array) const;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_scored_cloud_;

  const float score_offset_;
  const float max_raw_score_;
  const float min_prob_;
  const float far_weight_gain_;
  HierarchicalCostMap cost_map_;
};
}  // namespace particle_filter