#pragma once

#include "camera_ekf_corrector/hierarchical_cost_map.hpp"

#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::ekf_corrector
{
cv::Point2f cv2pt(const Eigen::Vector3f v);
float abs_cos(const Eigen::Vector3f & t, float deg);

class CameraEkfCorrector : public rclcpp::Node
{
public:
  using LineSegment = pcl::PointXYZLNormal;
  using LineSegments = pcl::PointCloud<LineSegment>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::Pose;
  CameraEkfCorrector();

private:
  const float far_weight_gain_;
  HierarchicalCostMap cost_map_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_bounding_box_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_map_image_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  Eigen::Vector3f last_mean_position_;
  std::optional<PoseStamped> latest_pose_{std::nullopt};
  std::function<float(float)> score_converter_;

  bool enable_switch_{true};

  void on_lsd(const PointCloud2 & msg);
  void on_ll2(const PointCloud2 & msg);
  void on_bounding_box(const PointCloud2 & msg);
  void on_pose(const PoseStamped & msg);

  std::pair<LineSegments, LineSegments> split_linesegments(const PointCloud2 & msg);

  float compute_logit(const LineSegments & lsd_cloud, const Eigen::Vector3f & self_position);

  std::pair<LineSegments, LineSegments> filt(const LineSegments & lines);
};
}  // namespace pcdless::ekf_corrector