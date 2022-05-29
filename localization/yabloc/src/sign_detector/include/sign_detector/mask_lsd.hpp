#pragma once
#include "sign_detector/synchro_lsd.hpp"

#include <lsd/lsd.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/video.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MaskLsd : public rclcpp::Node
{
public:
  MaskLsd() : Node("mask_lsd")
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/sensing/camera/traffic_light/camera_info", 10, std::bind(&MaskLsd::infoCallback, this, _1));

    subscriber = std::make_shared<SyncroSubscriber>(
      rclcpp::Node::SharedPtr{this}, "/eagleye/pose",
      "/sensing/camera/traffic_light/image_raw/compressed");
    subscriber->setCallback(std::bind(&MaskLsd::synchroCallback, this, _1, _2));

    lsd = cv::lsd::createLineSegmentDetector(cv::lsd::LSD_REFINE_ADV);

    sub_map_ = create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
      "/map/vector_map", rclcpp::QoS(10).transient_local().reliable(),
      std::bind(&MaskLsd::mapCallback, this, _1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  std::optional<sensor_msgs::msg::CameraInfo> info_{std::nullopt};
  std::optional<cv::Mat> scaled_intrinsic{std::nullopt};
  std::shared_ptr<SyncroSubscriber> subscriber;
  cv::Ptr<cv::lsd::LineSegmentDetector> lsd;
  pcl::PointCloud<pcl::PointNormal>::Ptr linestrings_ = nullptr;
  const float max_range_ = 20.f;
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void infoCallback(const sensor_msgs::msg::CameraInfo & msg)
  {
    info_ = msg;
    listenExtrinsicTf(info_->header.frame_id);
  }

  void overlayLl2(
    cv::Mat & lsd_image, const pcl::PointCloud<pcl::PointNormal> & ll2,
    const geometry_msgs::msg::Pose & pose);

  void synchroCallback(
    const geometry_msgs::msg::PoseStamped &, const sensor_msgs::msg::CompressedImage &);

  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);

  pcl::PointCloud<pcl::PointNormal> poseCallback(
    const geometry_msgs::msg::PoseStamped & pose_stamped);

  void listenExtrinsicTf(const std::string & frame_id);
};