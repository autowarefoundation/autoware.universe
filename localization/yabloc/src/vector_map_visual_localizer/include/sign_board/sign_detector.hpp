#pragma once

#include "common/static_tf_subscriber.hpp"

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace sign_board
{
class SignDetector : public AbstCorrector
{
public:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  SignDetector();

private:
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_sign_board_;
  common::StaticTfSubscriber tf_subscriber_;

  std::optional<CameraInfo> info_;
  std::optional<Eigen::Affine3f> camera_extrinsic_;
  pcl::PointCloud<pcl::PointNormal> sign_board_;

  void callbackImage(const Image & image);
  void callbackLl2(const PointCloud2 & ll2);
  void callbackInfo(const CameraInfo & info);
};
};  // namespace sign_board