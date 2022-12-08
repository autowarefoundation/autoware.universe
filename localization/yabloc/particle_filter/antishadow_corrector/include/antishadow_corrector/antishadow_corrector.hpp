#pragma once

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::modularized_particle_filter
{
class AntishadowCorrector : public modularized_particle_filter::AbstCorrector
{
public:
  using LineSegments = pcl::PointCloud<pcl::PointNormal>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  AntishadowCorrector();

private:
  const int IMAGE_RADIUS = 250;
  const float METRIC_PER_PIXEL = 0.1;  // [m/pixel]
  const float score_min_;
  const float score_max_;
  const float weight_min_;
  const bool print_statistics_;

  rclcpp::Subscription<Image>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_stamped_;
  LineSegments ll2_cloud_;
  std::optional<Sophus::SE3f> latest_pose_{std::nullopt};
  Eigen::Vector3f last_mean_position_{0, 0, 0};

  void on_lsd(const Image & msg);
  void on_ll2(const PointCloud2 & msg);
  void on_pose_stamped(const PoseStamped & msg);

  float compute_score(const LineSegments & src, const cv::Mat & lsd_image) const;

  cv::Point2f cv_pt2(const Eigen::Vector3f & v) const;

  std::function<float(float)> define_normalize_score() const;

  void print_particle_statistics(const ParticleArray & array) const;
};
}  // namespace pcdless::modularized_particle_filter
