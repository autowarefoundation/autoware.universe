#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vml_common/camera_info_subscriber.hpp>
#include <vml_common/static_tf_subscriber.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace imgproc
{
class Mapping : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Image = sensor_msgs::msg::Image;

  Mapping();

private:
  const float min_segment_length_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_rgb_image_;
  // Subscriber
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  vml_common::CameraInfoSubscriber info_;
  vml_common::StaticTfSubscriber tf_subscriber_;

  std::list<TwistStamped::ConstSharedPtr> twist_list_;

  const int IMAGE_RADIUS = 250;
  const float METRIC_PER_PIXEL = 0.1;  // [m/pixel]
  cv::Mat histogram_image_;

  void onTwist(TwistStamped::ConstSharedPtr msg);
  void onLineSegments(const PointCloud2 & msg);
  void transformImage(const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp);

  cv::Point2f cv_pt2(const Eigen::Vector3f & v) const;
  Eigen::Vector3f eigen_vec3f(const cv::Point2f & v) const;

  void popObsoleteMsg(const rclcpp::Time & oldest_stamp);
  void draw(const PointCloud2 & cloud_msg);

  Sophus::SE3f accumulateTravelDistance(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp);
};
}  // namespace imgproc
