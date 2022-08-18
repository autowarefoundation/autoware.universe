#pragma once
#include "common/static_tf_subscriber.hpp"

#include <Eigen/Geometry>
#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace particle_filter
{
class SignCorrector : public modularized_particle_filter::AbstCorrector
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using Vec3Vec = std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>;
  SignCorrector();

private:
  const int blur_size_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<ParticleArray>::SharedPtr sub_particle_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  std::vector<Vec3Vec> sign_boards_;
  lanelet::LaneletMapPtr lanelet_map_{nullptr};
  std::optional<CameraInfo> camera_info_{std::nullopt};

  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  common::StaticTfSubscriber tf_subscriber_;

  std::optional<std::vector<cv::Point2i>> projectBoard(
    const Eigen::Matrix3f & K, const Eigen::Affine3f & T, const Eigen::Affine3f & transform,
    const Vec3Vec & contour);

  void execute(const rclcpp::Time & stamp, cv::Mat image);
  void extractNearSign(const PoseStamped & pose_stamped);

  void mapCallback(const HADMapBin & msg);
  void poseCallback(const PoseStamped & msg);
  void infoCallback(const CameraInfo & msg);
  void imageCallback(const Image & msg);
};
}  // namespace particle_filter