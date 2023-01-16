#pragma once

#include "camera_particle_corrector/hierarchical_cost_map.hpp"

#include <modularized_particle_filter/correction/abst_corrector.hpp>
#include <opencv4/opencv2/core.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::modularized_particle_filter
{
cv::Point2f cv2pt(const Eigen::Vector3f v);
float abs_cos(const Eigen::Vector3f & t, float deg);

class CameraParticleCorrector : public modularized_particle_filter::AbstCorrector
{
public:
  using LineSegment = pcl::PointXYZLNormal;
  using LineSegments = pcl::PointCloud<LineSegment>;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Image = sensor_msgs::msg::Image;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Pose = geometry_msgs::msg::Pose;
  using Bool = std_msgs::msg::Bool;
  using String = std_msgs::msg::String;
  using SetBool = std_srvs::srv::SetBool;
  CameraParticleCorrector();

private:
  const float min_prob_;
  const float far_weight_gain_;
  common::HierarchicalCostMap cost_map_;

  rclcpp::Subscription<PointCloud2>::SharedPtr sub_bounding_box_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_lsd_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_ll2_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_;
  rclcpp::Service<SetBool>::SharedPtr switch_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<Image>::SharedPtr pub_map_image_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_scored_cloud_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_scored_posteriori_cloud_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;

  Eigen::Vector3f last_mean_position_;
  std::optional<PoseStamped> latest_pose_{std::nullopt};
  std::function<float(float)> score_converter_;

  bool enable_switch_{true};

  void on_lsd(const PointCloud2 & msg);
  void on_ll2(const PointCloud2 & msg);
  void on_bounding_box(const PointCloud2 & msg);
  void on_pose(const PoseStamped & msg);
  void on_timer();
  void on_service(SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response);

  std::pair<LineSegments, LineSegments> split_linesegments(const PointCloud2 & msg);

  float compute_logit(const LineSegments & lsd_cloud, const Eigen::Vector3f & self_position);

  pcl::PointCloud<pcl::PointXYZI> evaluate_cloud(
    const LineSegments & lsd_cloud, const Eigen::Vector3f & self_position);

  std::pair<LineSegments, LineSegments> filt(const LineSegments & lines);
};
}  // namespace pcdless::modularized_particle_filter