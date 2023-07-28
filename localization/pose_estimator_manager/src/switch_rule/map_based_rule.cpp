#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

#include "pose_estimator_manager/switch_rule/grid_info.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace multi_pose_estimator
{
std::vector<PoseEstimatorName> MapBasedRule::supporting_pose_estimators()
{
  return {PoseEstimatorName::NDT, PoseEstimatorName::YABLOC, PoseEstimatorName::EAGLEYE};
}

MapBasedRule::MapBasedRule(rclcpp::Node & node)
: BaseSwitchRule(node),
  pcd_density_threshold_(node.declare_parameter<int>("pcd_occupancy_rule/pcd_density_threshold"))
{
  RCLCPP_INFO_STREAM(get_logger(), "MapBasedRule is initialized");

  using std::placeholders::_1;
  const auto map_qos = rclcpp::QoS(1).transient_local().reliable();

  auto on_point_cloud_map = std::bind(&MapBasedRule::on_point_cloud_map, this, _1);
  auto on_vector_map = std::bind(&MapBasedRule::on_vector_map, this, _1);
  auto on_pose_cov = std::bind(&MapBasedRule::on_pose_cov, this, _1);
  sub_vector_map_ =
    node.create_subscription<HADMapBin>("~/input/vector_map", map_qos, on_vector_map);
  sub_point_cloud_map_ =
    node.create_subscription<PointCloud2>("~/input/pointcloud_map", map_qos, on_point_cloud_map);
  sub_pose_cov_ =
    node.create_subscription<PoseCovStamped>("~/input/pose_with_covariance", 10, on_pose_cov);
}

std::unordered_map<PoseEstimatorName, bool> MapBasedRule::update()
{
  RCLCPP_WARN_STREAM(get_logger(), "pluginlib try to choice the best localizer");

  std::unordered_map<PoseEstimatorName, bool> toggle_list;
  toggle_list[PoseEstimatorName::NDT] = true;
  toggle_list[PoseEstimatorName::YABLOC] = false;
  toggle_list[PoseEstimatorName::EAGLEYE] = false;

  if (!latest_pose_.has_value()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Unable to determine which estimation to use, due to lack of latest position");
    return toggle_list;
  }

  auto position = latest_pose_->pose.pose.position;
  pcl::PointXYZ query(position.x, position.y, position.z);

  std::vector<int> indices;
  std::vector<float> distances;
  const int count = kdtree_->radiusSearch(query, 50, indices, distances, 0);

  std::stringstream ss;
  ss << "PCD occupancy: " << count << " > " << pcd_density_threshold_ << "\n";
  const bool is_ndt_mode = count > pcd_density_threshold_;
  if (is_ndt_mode) {
    toggle_list[PoseEstimatorName::NDT] = true;
    toggle_list[PoseEstimatorName::YABLOC] = false;
    toggle_list[PoseEstimatorName::EAGLEYE] = false;
    ss << "NDT";
  } else {
    if (position.x > 89374) {
      toggle_list[PoseEstimatorName::NDT] = false;
      toggle_list[PoseEstimatorName::YABLOC] = false;
      toggle_list[PoseEstimatorName::EAGLEYE] = true;
      ss << "Eagleye";
    } else {
      toggle_list[PoseEstimatorName::NDT] = false;
      toggle_list[PoseEstimatorName::YABLOC] = true;
      toggle_list[PoseEstimatorName::EAGLEYE] = false;
      ss << "YabLoc";
    }
  }

  ss << "\nEagleye: " << (eagleye_area_.within(position) ? "ON" : "OFF");

  debug_string_msg_ = ss.str();

  return toggle_list;
}

std::string MapBasedRule::debug_string()
{
  return debug_string_msg_;
}

MapBasedRule::MarkerArray MapBasedRule::debug_marker_array()
{
  MarkerArray array_msg;
  visualization_msgs::msg::Marker msg;

  msg.ns = "pcd_occupancy";
  msg.id = 0;
  msg.header.frame_id = "map";
  msg.scale.set__x(3.0f).set__y(3.0f).set__z(3.f);
  msg.color.set__r(1.0f).set__g(1.0f).set__b(0.2f).set__a(1.0f);

  if (occupied_areas_) {
    for (auto p : occupied_areas_->points) {
      geometry_msgs::msg::Point geometry_point{};
      geometry_point.set__x(p.x).set__y(p.y).set__z(p.z);
      msg.points.push_back(geometry_point);
    }
  }

  array_msg.markers.push_back(msg);
  return array_msg;
}

void MapBasedRule::on_vector_map(HADMapBin::ConstSharedPtr msg)
{
  RCLCPP_INFO_STREAM(get_logger(), "subscribed binary vector map");
  eagleye_area_.init(msg);
}

void MapBasedRule::on_point_cloud_map(PointCloud2::ConstSharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);

  std::unordered_map<GridInfo, size_t> grid_point_count;
  for (pcl::PointXYZ xyz : cloud) {
    grid_point_count[GridInfo(xyz.x, xyz.y)] += 1;
  }

  occupied_areas_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (const auto [grid, count] : grid_point_count) {
    if (count > 50) {
      occupied_areas_->push_back(grid.get_center_point());
    }
  }

  kdtree_ = pcl::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  kdtree_->setInputCloud(occupied_areas_);

  RCLCPP_INFO_STREAM(
    get_logger(), "pose_estiamtor_manager has loaded PCD map " << occupied_areas_->size());
}

void MapBasedRule::on_pose_cov(PoseCovStamped::ConstSharedPtr msg)
{
  latest_pose_ = *msg;
}

}  // namespace multi_pose_estimator