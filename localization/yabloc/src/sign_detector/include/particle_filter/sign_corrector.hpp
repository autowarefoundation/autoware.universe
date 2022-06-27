#pragma once
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

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace particle_filter
{
class SignCorrector : public AbstCorrector
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  SignCorrector();

private:
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<ParticleArray>::SharedPtr sub_particle_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;

  pcl::PointCloud<pcl::PointNormal>::Ptr linestrings_{nullptr};
  lanelet::LaneletMapPtr lanelet_map_{nullptr};
  std::optional<CameraInfo> camera_info_{std::nullopt};

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};

  void execute(const rclcpp::Time & stamp, cv::Mat image);
  void listenExtrinsicTf(const std::string & frame_id);
  void extractNearSign(const PoseStamped & pose_stamped);

  void mapCallback(const HADMapBin & msg);
  void poseCallback(const PoseStamped & msg);
  void infoCallback(const CameraInfo & msg);
  void imageCallback(const Image & msg);
};
}  // namespace particle_filter