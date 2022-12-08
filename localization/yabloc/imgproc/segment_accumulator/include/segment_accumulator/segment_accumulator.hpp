#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/gamma_converter.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::accumulator
{
class SegmentAccumulator : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using LineSegments = pcl::PointCloud<pcl::PointNormal>;

  SegmentAccumulator();

private:
  const float min_segment_length_;
  const int IMAGE_RADIUS = 250;
  const float METRIC_PER_PIXEL = 0.1;  // [m/pixel]
  const int default_map_value_;
  const float map_update_interval_;
  cv::Mat histogram_image_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_rgb_image_;
  // Subscriber
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_stamped_;
  common::CameraInfoSubscriber info_;
  common::StaticTfSubscriber tf_subscriber_;
  LineSegments ll2_cloud_;

  std::function<std::optional<Eigen::Vector3f>(const Eigen::Vector3f & u)> back_project_func_{
    nullptr};

  std::vector<cv::Point2i> visible_area_polygon_;
  std::list<TwistStamped::ConstSharedPtr> twist_list_;

  std::optional<Sophus::SE3f> latest_pose_{std::nullopt};
  common::GammaConverter gamma_converter{6.0f};

  void on_ll2(const PointCloud2 & msg);
  void on_pose_stamped(const PoseStamped & msg);
  void on_twist(TwistStamped::ConstSharedPtr msg);
  void on_line_segments(const PointCloud2 & msg);
  void transform_image(const Sophus::SE3f & odom);

  cv::Point2f cv_pt2(const Eigen::Vector3f & v) const;
  Eigen::Vector3f eigen_vec3f(const cv::Point2f & v) const;

  void pop_obsolete_msg(const rclcpp::Time & oldest_stamp);
  void draw(const LineSegments::Ptr & line_segments);

  void define_project_function();
  LineSegments::Ptr filt_line_segments_by_ll2(const LineSegments::Ptr & lines);

  Sophus::SE3f accumulate_travel_distance(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp);
};
}  // namespace pcdless::accumulator
