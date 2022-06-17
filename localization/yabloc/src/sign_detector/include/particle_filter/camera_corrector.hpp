#pragma once
#include "particle_filter/hierarchical_cost_map.hpp"

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sign_detector_msgs/msg/cloud_with_pose.hpp>

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

  CameraParticleCorrector()
  : AbstCorrector("camera_particle_corrector"),
    image_size_(declare_parameter<int>("image_size", 800)),
    max_range_(declare_parameter<float>("max_range", 20.f)),
    gamma_(declare_parameter<float>("gamma", 3.0f)),
    score_offset_(declare_parameter<float>("score_offset", -64.f)),
    max_raw_score_(declare_parameter<float>("max_raw_score", 5000.0)),
    min_prob_(declare_parameter<float>("min_prob", 0.01)),
    cost_map_(max_range_, image_size_, gamma_)
  {
    using std::placeholders::_1;

    auto lsd_callback = std::bind(&CameraParticleCorrector::lsdCallback, this, _1);
    lsd_sub_ = create_subscription<PointCloud2>("/lsd_cloud", 10, lsd_callback);

    auto ll2_callback = std::bind(&CameraParticleCorrector::ll2Callback, this, _1);
    ll2_sub_ = create_subscription<PointCloud2>("/ll2_cloud", 10, ll2_callback);

    auto pose_callback = std::bind(&CameraParticleCorrector::poseCallback, this, _1);
    pose_sub_ = create_subscription<PoseStamped>("/particle_pose", 10, pose_callback);

    image_pub_ = create_publisher<Image>("/match_image", 10);
    marker_pub_ = create_publisher<MarkerArray>("/cost_map_range", 10);
  }

private:
  void lsdCallback(const PointCloud2 & msg);
  void ll2Callback(const PointCloud2 & msg);
  void poseCallback(const PoseStamped & msg);

  float computeScore(const LineSegment & ls_cloud, cv::Mat & ll2_image);
  LineSegment transformCloud(const LineSegment & src, const Eigen::Affine3f & transform);

  rclcpp::Subscription<PointCloud2>::SharedPtr lsd_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr ll2_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;

  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;

  const int image_size_;
  const float max_range_;
  const float gamma_;
  const float score_offset_;
  const float max_raw_score_;
  const float min_prob_;
  HierarchicalCostMap cost_map_;
};
}  // namespace particle_filter