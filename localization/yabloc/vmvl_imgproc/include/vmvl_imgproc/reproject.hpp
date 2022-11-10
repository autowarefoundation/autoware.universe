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
class Reprojector : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  Reprojector();

private:
  const float min_segment_length_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  // Subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  vml_common::CameraInfoSubscriber info_;
  vml_common::StaticTfSubscriber tf_subscriber_;

  std::list<Image::ConstSharedPtr> image_list_;
  std::list<TwistStamped::ConstSharedPtr> twist_list_;

  const int IMAGE_RADIUS = 250;
  const float METRIC_PER_PIXEL = 0.1;  // [m/pixel]
  cv::Mat histogram_image_;

  void onImage(Image::ConstSharedPtr msg);
  void onTwist(TwistStamped::ConstSharedPtr msg);
  void onLineSegments(const PointCloud2 & msg);
  void transformImage(const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp);

  cv::Point2f cv_pt2(const Eigen::Vector3f & v) const;
  Eigen::Vector3f eigen_vec3f(const cv::Point2f & v) const;

  void popObsoleteMsg();
  // void reproject(const Image & old_image_msg, const PointCloud2 & cloud_msg);
  void draw(const PointCloud2 & cloud_msg);

  cv::Mat applyPerspective(const cv::Mat & image);

  Sophus::SE3f accumulateTravelDistance(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp);
};
}  // namespace imgproc