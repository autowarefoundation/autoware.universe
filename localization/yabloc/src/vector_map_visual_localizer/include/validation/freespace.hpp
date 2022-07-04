#pragma once
#include "validation/graph_segmentation.hpp"

#include <Eigen/Geometry>
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

namespace validation
{
class FreeSpace : public rclcpp::Node
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  FreeSpace();

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
  cv::Mat segmented_;

  cv::Ptr<cv::SimpleBlobDetector> detector_;

  void listenExtrinsicTf(const std::string & frame_id);

  void extractNearLanelet(const PoseStamped & pose_stamped);

  void mapCallback(const HADMapBin & msg);
  void poseCallback(const PoseStamped & msg);
  void particleCallback(const ParticleArray & msg);
  void infoCallback(const CameraInfo & msg);
  void imageCallback(const Image & msg);

  void incrementAlongLine(cv::Mat image, const cv::Point2i & from, const cv::Point2i & to);

  using GraphSegmentation = cv::ximgproc::modified::GraphSegmentation;
  cv::Ptr<GraphSegmentation> segmentation_;
};
}  // namespace validation