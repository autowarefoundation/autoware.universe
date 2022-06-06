#pragma once
#include "common/gammma_conveter.hpp"

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>

#include <modularized_particle_filter_msgs/msg/cloud_with_pose.hpp>
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
  using CloudWithPose = modularized_particle_filter_msgs::msg::CloudWithPose;
  using Image = sensor_msgs::msg::Image;

  CameraParticleCorrector()
  : AbstCorrector("camera_particle_corrector"),
    image_size_(declare_parameter<int>("image_size", 800)),
    max_range_(declare_parameter<float>("max_range", 20.f)),
    gamma_(declare_parameter<float>("gamma", 3.0f)),
    score_offset_(declare_parameter<float>("score_offset", -64.f)),
    max_raw_score_(declare_parameter<float>("max_raw_score", 5000.0)),
    min_prob_(declare_parameter<float>("min_prob", 0.01))
  {
    using std::placeholders::_1;

    auto lsd_callback = std::bind(&CameraParticleCorrector::lsdCallback, this, _1);
    lsd_sub_ = create_subscription<PointCloud2>("/lsd_cloud", 10, lsd_callback);

    auto ll2_callback = std::bind(&CameraParticleCorrector::ll2Callback, this, _1);
    ll2_sub_ = create_subscription<CloudWithPose>("/ll2_cloud", 10, ll2_callback);

    image_pub_ = create_publisher<Image>("/match_image", 10);

    gamma_converter.reset(gamma_);
  }

private:
  void lsdCallback(const PointCloud2 & msg);
  void ll2Callback(const CloudWithPose & msg);
  cv::Mat buildLl2Image(const LineSegment & cloud);
  cv::Point2f toCvPoint(const Eigen::Vector3f & p);

  float computeScore(const LineSegment & ls_cloud, cv::Mat & ll2_image);
  LineSegment transformCloud(const LineSegment & src, const Eigen::Affine3f & transform);

  rclcpp::Subscription<PointCloud2>::SharedPtr lsd_sub_;
  rclcpp::Subscription<CloudWithPose>::SharedPtr ll2_sub_;

  rclcpp::Publisher<Image>::SharedPtr image_pub_;
  std::optional<CloudWithPose> latest_cloud_with_pose_{std::nullopt};

  GammaConverter gamma_converter{4.0f};
  const int image_size_;
  const float max_range_;
  const float gamma_;
  const float score_offset_;
  const float max_raw_score_;
  const float min_prob_;
};
}  // namespace particle_filter