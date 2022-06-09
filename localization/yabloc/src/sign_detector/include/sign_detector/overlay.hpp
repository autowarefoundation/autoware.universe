#pragma once
#include "sign_detector/timer.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sign_detector_msgs/msg/cloud_with_pose.hpp>

#include <boost/circular_buffer.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Overlay : public rclcpp::Node
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using CloudWithPose = sign_detector_msgs::msg::CloudWithPose;

  Overlay() : Node("overlay"), pose_buffer_{20}
  {
    using std::placeholders::_1;
    using namespace std::literals::chrono_literals;

    const rclcpp::QoS qos = rclcpp::QoS(10);
    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/sensing/camera/traffic_light/camera_info", 10, std::bind(&Overlay::infoCallback, this, _1));
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
      "/sensing/camera/traffic_light/image_raw/compressed", qos,
      std::bind(&Overlay::imageCallback, this, _1));
    sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/particle_pose", 10, std::bind(&Overlay::poseCallback, this, _1));
    sub_cloud_pose_ = create_subscription<CloudWithPose>(
      "/ll2_cloud", 10, std::bind(&Overlay::cloudPoseCallback, this, _1));

    pub_image_ = create_publisher<sensor_msgs::msg::Image>("/overlay_image", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Subscription<CloudWithPose>::SharedPtr sub_cloud_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::optional<sensor_msgs::msg::CameraInfo> info_{std::nullopt};
  std::optional<Eigen::Affine3f> camera_extrinsic_{std::nullopt};
  std::optional<CloudWithPose> latest_cloud_with_pose_{std::nullopt};
  boost::circular_buffer<geometry_msgs::msg::PoseStamped> pose_buffer_;

  void infoCallback(const sensor_msgs::msg::CameraInfo & msg);
  void imageCallback(const sensor_msgs::msg::Image & msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped & msg);
  void cloudPoseCallback(const CloudWithPose & msg);

  void listenExtrinsicTf(const std::string & frame_id);

  void overlay(const cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp);
};