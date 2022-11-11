#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vml_common/camera_info_subscriber.hpp>
#include <vml_common/static_tf_subscriber.hpp>
#include <vml_common/synchro_subscriber.hpp>

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
  const float gain_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  // Subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  SynchroSubscriber<Image, PointCloud2> synchro_subscriber_;

  vml_common::CameraInfoSubscriber info_;
  vml_common::StaticTfSubscriber tf_subscriber_;

  std::list<Image> image_list_;
  std::list<TwistStamped::ConstSharedPtr> twist_list_;

  using ProjectFunc = std::function<std::optional<cv::Point2f>(const cv::Point2f &)>;

  struct Parameter
  {
    Sophus::SE3f extrinsic_;
    Eigen::Matrix3f K_;
    Eigen::Matrix3f Kinv_;
  };
  struct TransformPair
  {
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
  };

  std::optional<Parameter> param_{std::nullopt};

  // Callback
  void onSynchro(const Image & image_msg, const PointCloud2 & lsd_msg);
  void onTwist(TwistStamped::ConstSharedPtr msg);

  std::vector<TransformPair> makeTransformPairs(
    ProjectFunc func, pcl::PointCloud<pcl::PointNormal> & segments);

  void popObsoleteMsg();
  // void reproject(
  //   const Image & old_image_msg, const Image & current_image_msg, const PointCloud2 & cloud_msg);

  void drawTransformedPixel(const std::vector<TransformPair> & pairs, cv::Mat image);

  void tryDefineParam();
  ProjectFunc defineProjectionFunction(const Sophus::SE3f & odom);

  std::vector<cv::Point2i> line2Polygon(const cv::Point2f & from, const cv::Point2f & to) const;
  Sophus::SE3f accumulateTravelDistance(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp) const;
};
}  // namespace imgproc