#ifndef POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_
#define POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_

#include "pose_estimator_manager/manager_client.hpp"
#include "pose_estimator_manager/plugin_interface.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace multi_pose_estimator
{
class PoseEstimatorManager : public rclcpp::Node
{
public:
  using SetBool = std_srvs::srv::SetBool;
  using String = std_msgs::msg::String;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseCovStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  PoseEstimatorManager();

private:
  const int pcd_density_threshold;
  rclcpp::Subscription<PoseCovStamped>::SharedPtr sub_pose_cov_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_map_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_grid_marker_;
  rclcpp::Publisher<String>::SharedPtr pub_debug_string_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<SetBool>::SharedPtr switch_service_server_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unordered_map<std::string, ManagerClient::SharedPtr> clients_;
  std::optional<PoseCovStamped> latest_pose_{std::nullopt};

  pluginlib::ClassLoader<PluginInterface> plugin_loader_;
  std::shared_ptr<PluginInterface> switch_rule_plugin_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_areas_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  bool toggle_mode(bool enable_ndt);
  void publish_occupied_area(const std_msgs::msg::Header & header);

  void on_service(SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response);
  void on_map(PointCloud2::ConstSharedPtr msg);
  void on_timer();
  void on_pose_cov(PoseCovStamped::ConstSharedPtr msg);

  void load_switch_rule_plugin(rclcpp::Node & node, const std::string & name);
};
}  // namespace multi_pose_estimator

#endif /* POSE_ESTIMATOR_MANAGER__POSE_ESTIMATOR_MANAGER_HPP_ */