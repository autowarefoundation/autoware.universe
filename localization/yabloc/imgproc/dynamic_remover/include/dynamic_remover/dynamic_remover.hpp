#pragma once
#include <opencv4/opencv2/core.hpp>
#include <pcdless_common/camera_info_subscriber.hpp>
#include <pcdless_common/static_tf_subscriber.hpp>
#include <pcdless_common/synchro_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::dynamic_remover
{
class DynamicRemover : public rclcpp::Node
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  DynamicRemover();

private:
  const float min_segment_length_;
  const int polygon_thick_;
  const float gap_threshold_;
  const int search_iteration_max_;
  const size_t backward_frame_interval_;

  common::CameraInfoSubscriber info_;
  common::StaticTfSubscriber tf_subscriber_;

  // Publisher
  rclcpp::Publisher<Image>::SharedPtr pub_old_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_cur_image_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_cloud_;
  // Subscriber
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<TwistStamped>::SharedPtr sub_twist_;
  common::SynchroSubscriber<Image, PointCloud2> synchro_subscriber_;

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
    size_t id;
    std::vector<cv::Point2f> src;
    std::vector<cv::Point2f> dst;
  };
  struct GapResult
  {
    size_t id;
    cv::Point2f final_offset;
    float gap;
  };

  std::optional<Parameter> param_{std::nullopt};

  // Callback
  void on_synchro(const Image & image_msg, const PointCloud2 & lsd_msg);
  void on_twist(TwistStamped::ConstSharedPtr msg);

  std::vector<TransformPair> make_transform_pairs(
    ProjectFunc func, pcl::PointCloud<pcl::PointNormal> & segments);

  void pop_obsolete_msg();

  std::unordered_map<size_t, GapResult> compute_gap(
    const std::vector<TransformPair> & pairs, const cv::Mat & old_image, const cv::Mat & cur_image);

  void visualize_and_publish(
    const std::vector<TransformPair> & pairs, const std::unordered_map<size_t, GapResult> & gap_map,
    const cv::Mat & old_image, const cv::Mat & cur_image);

  void try_define_param();
  ProjectFunc define_projection_function(const Sophus::SE3f & odom);

  void publish_cloud(
    const pcl::PointCloud<pcl::PointNormal> & src,
    const std::unordered_map<size_t, GapResult> & gaps, const rclcpp::Time & stamp);

  std::vector<cv::Point2i> line_to_polygon(const cv::Point2f & from, const cv::Point2f & to) const;

  Sophus::SE3f accumulate_travel_distance(
    const rclcpp::Time & from_stamp, const rclcpp::Time & to_stamp) const;
};
}  // namespace pcdless::dynamic_remover