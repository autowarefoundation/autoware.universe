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
  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  // Subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  vml_common::CameraInfoSubscriber info_;
  vml_common::StaticTfSubscriber tf_subscriber_;

  std::list<Image::ConstSharedPtr> image_list_;
  std::list<TwistStamped::ConstSharedPtr> twist_list_;

  void onImage(Image::ConstSharedPtr msg);
  void onTwist(TwistStamped::ConstSharedPtr msg);
  void onLineSegments(const PointCloud2 & msg);

  void popObsoleteMsg(const rclcpp::Time & stamp);
  void reproject(const Image & old_image, const Image & cur_image);

  Sophus::SE3f accumulateTravelDistance(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp);
};
}  // namespace imgproc