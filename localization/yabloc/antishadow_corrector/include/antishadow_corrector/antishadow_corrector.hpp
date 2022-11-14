#pragma once

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>

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
  void onLsd(const Image & msg);
  void onLl2(const PointCloud2 & msg);

  LineSegment transformCloud(const LineSegment & src, const Eigen::Affine3f & transform) const;

  LineSegment ll2_cloud_;

  rclcpp::Subscription<Image>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
};
}  // namespace modularized_particle_filter
