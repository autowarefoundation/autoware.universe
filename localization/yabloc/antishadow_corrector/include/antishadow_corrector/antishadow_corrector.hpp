#pragma once

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>
#include <sophus/geometry.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace modularized_particle_filter
{
class AntishadowCorrector : public modularized_particle_filter::AbstCorrector
{
public:
  using LineSegment = pcl::PointCloud<pcl::PointNormal>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Image = sensor_msgs::msg::Image;
  AntishadowCorrector();

private:
  const int IMAGE_RADIUS = 250;
  const float METRIC_PER_PIXEL = 0.1;  // [m/pixel]

  rclcpp::Subscription<Image>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  LineSegment ll2_cloud_;

  void onLsd(const Image & msg);
  void onLl2(const PointCloud2 & msg);

  LineSegment transformCloud(const LineSegment & src, const Sophus::SE3f & transform) const;
  float computeScore(const LineSegment & src, const cv::Mat & lsd_image) const;

  cv::Point2f cv_pt2(const Eigen::Vector3f & v) const;
};
}  // namespace modularized_particle_filter
