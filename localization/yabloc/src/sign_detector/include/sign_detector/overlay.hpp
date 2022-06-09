#pragma once
#include "sign_detector/timer.hpp"

#include <eigen3/Eigen/Geometry>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sign_detector_msgs/msg/cloud_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <boost/circular_buffer.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class Overlay : public rclcpp::Node
{
public:
  using Pose = geometry_msgs::msg::Pose;
  using Marker = visualization_msgs::msg::Marker;
  using CloudWithPose = sign_detector_msgs::msg::CloudWithPose;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using LineSegments = pcl::PointCloud<pcl::PointNormal>;

  Overlay() : Node("overlay"), pose_buffer_{20}
  {
    using std::placeholders::_1;
    using namespace std::literals::chrono_literals;
    // Subscriber
    {
      const rclcpp::QoS qos = rclcpp::QoS(10);
      sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/sensing/camera/traffic_light/camera_info", 10,
        std::bind(&Overlay::infoCallback, this, _1));
      sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        "/sensing/camera/traffic_light/image_raw/compressed", qos,
        std::bind(&Overlay::imageCallback, this, _1));
      sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/particle_pose", 10, std::bind(&Overlay::poseCallback, this, _1));
      sub_cloud_pose_ = create_subscription<CloudWithPose>(
        "/ll2_cloud", 10, std::bind(&Overlay::cloudPoseCallback, this, _1));
      sub_cloud_ = create_subscription<PointCloud2>(
        "/lsd_cloud", 10, std::bind(&Overlay::cloudCallback, this, _1));

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }
    // Publisher
    {
      pub_vis_ = create_publisher<Marker>("/marker", 10);
      pub_image_ = create_publisher<sensor_msgs::msg::Image>("/overlay_image", 10);
    }

    {
      declare_parameter("offset", std::vector<double>{0, 0, 0, 0, 0, 0});
      param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
      auto cb = [this](const rclcpp::Parameter & p) {
        if (p.as_double_array().size() == 6) {
          auto v = p.as_double_array();
          this->ex_extrinsic_ = buildAffine(v);
          RCLCPP_WARN_STREAM(get_logger(), "update: " << this->ex_extrinsic_.matrix());
        }
      };
      cb_handle_ = param_subscriber_->add_parameter_callback("offset", cb);
    }
  }

private:
  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  Eigen::Affine3f ex_extrinsic_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  rclcpp::Subscription<CloudWithPose>::SharedPtr sub_cloud_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_cloud_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_vis_;
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
  void cloudCallback(const PointCloud2 & msg);

  void listenExtrinsicTf(const std::string & frame_id);

  void overlay(const cv::Mat & image, const Pose & pose, const rclcpp::Time & stamp);
  void makeVisMarker(const LineSegments & ls, const Pose & pose, const rclcpp::Time & stamp);

  Eigen::Affine3f buildAffine(const std::vector<double> & v)
  {
    Eigen::Affine3f a;
    double r = v[3], p = v[4], y = v[5];
    Eigen::Translation3f t(v[0], v[1], v[2]);
    Eigen::Quaternionf R = Eigen::AngleAxisf(y, Eigen::Vector3f::UnitZ()) *
                           Eigen::AngleAxisf(p, Eigen::Vector3f::UnitY()) *
                           Eigen::AngleAxisf(r, Eigen::Vector3f::UnitX());
    return t * R;
  }
};