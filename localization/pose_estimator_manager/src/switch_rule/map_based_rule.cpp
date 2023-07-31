#include "pose_estimator_manager/switch_rule/map_based_rule.hpp"

#include "pose_estimator_manager/switch_rule/grid_info.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace multi_pose_estimator
{
std::vector<PoseEstimatorName> MapBasedRule::supporting_pose_estimators()
{
  return {PoseEstimatorName::NDT, PoseEstimatorName::YABLOC, PoseEstimatorName::EAGLEYE};
}

MapBasedRule::MapBasedRule(
  rclcpp::Node & node, const std::unordered_set<PoseEstimatorName> & running_estimator_list)
: BaseSwitchRule(node),
  pcd_density_threshold_(node.declare_parameter<int>("pcd_occupancy_rule/pcd_density_threshold")),
  running_estimator_list_(running_estimator_list)
{
  RCLCPP_INFO_STREAM(get_logger(), "MapBasedRule is initialized");

  using std::placeholders::_1;
  const auto latch_qos = rclcpp::QoS(1).transient_local().reliable();

  auto on_vector_map = std::bind(&MapBasedRule::on_vector_map, this, _1);
  auto on_point_cloud_map = std::bind(&MapBasedRule::on_point_cloud_map, this, _1);
  auto on_pose_cov = std::bind(&MapBasedRule::on_pose_cov, this, _1);
  auto on_initialization_state = [this](InitializationState::ConstSharedPtr msg) -> void {
    initialization_state_ = *msg;
  };
  auto on_eagleye_fix = [this](NavSatFix::ConstSharedPtr) -> void {
    eagleye_is_initialized = true;
  };

  sub_vector_map_ =
    node.create_subscription<HADMapBin>("~/input/vector_map", latch_qos, on_vector_map);
  sub_point_cloud_map_ =
    node.create_subscription<PointCloud2>("~/input/pointcloud_map", latch_qos, on_point_cloud_map);
  sub_pose_cov_ =
    node.create_subscription<PoseCovStamped>("~/input/pose_with_covariance", 10, on_pose_cov);
  sub_initialization_state_ = node.create_subscription<InitializationState>(
    "~/input/initialization_state", latch_qos, on_initialization_state);
  sub_eagleye_fix_ = node.create_subscription<NavSatFix>("~/input/eagleye/fix", 10, on_eagleye_fix);

  //
  initialization_state_.state = InitializationState::UNINITIALIZED;
}

bool MapBasedRule::eagleye_is_available() const
{
  if (running_estimator_list_.count(PoseEstimatorName::EAGLEYE) == 0) {
    return false;
  }

  if (!eagleye_is_initialized) {
    return false;
  }

  if (!latest_pose_.has_value()) {
    return false;
  }

  return eagleye_area_.within(latest_pose_->pose.pose.position);
}
bool MapBasedRule::yabloc_is_available() const
{
  return running_estimator_list_.count(PoseEstimatorName::YABLOC) != 0;
}
bool MapBasedRule::ndt_is_available() const
{
  return running_estimator_list_.count(PoseEstimatorName::NDT) != 0;
}

std::unordered_map<PoseEstimatorName, bool> MapBasedRule::update()
{
  // (1) If the localization state is not 'INITIALIZED'
  if (initialization_state_.state != InitializationState::INITIALIZED) {
    debug_string_msg_ = "enable All\nlocalization is not initialized";
    RCLCPP_WARN_STREAM(
      get_logger(), "Enable all estimators because localization component is not initialized");
    return {
      {PoseEstimatorName::NDT, true},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, true},
    };
  }

  // (2) If no pose are published, enable all;
  if (!latest_pose_.has_value()) {
    debug_string_msg_ = "enable All\nestimated pose has not been published yet";
    RCLCPP_WARN_STREAM(
      get_logger(), "Unable to determine which estimation to use, due to lack of latest position");
    return {
      {PoseEstimatorName::NDT, true},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, true},
    };
  }

  // (3) If no pose are published, enable all;
  if (eagleye_is_available()) {
    debug_string_msg_ = "enable Eagleye\nthe vehicle is within eagleye_area";
    RCLCPP_WARN_STREAM(get_logger(), "Enable eagleye");
    return {
      {PoseEstimatorName::NDT, false},
      {PoseEstimatorName::YABLOC, false},
      {PoseEstimatorName::EAGLEYE, true},
    };
  }

  // (4) If yabloc is disabled, enable NDT
  if (!yabloc_is_available()) {
    debug_string_msg_ = "enable NDT\nonly NDT is available";
    RCLCPP_WARN_STREAM(get_logger(), "Enable NDT");
    return {
      {PoseEstimatorName::NDT, true},
      {PoseEstimatorName::YABLOC, false},
      {PoseEstimatorName::EAGLEYE, false},
    };
  }

  // (5) If ndt is disabled, enable YabLoc
  if (!ndt_is_available()) {
    debug_string_msg_ = "enable YabLoc\nonly yabloc is available";
    RCLCPP_WARN_STREAM(get_logger(), "Enable YABLOC");
    return {
      {PoseEstimatorName::NDT, false},
      {PoseEstimatorName::YABLOC, true},
      {PoseEstimatorName::EAGLEYE, false},
    };
  }

  // (6) If PCD density is enough,
  const auto position = latest_pose_->pose.pose.position;
  const pcl::PointXYZ query(position.x, position.y, position.z);
  std::vector<int> indices;
  std::vector<float> distances;
  const int count = kdtree_->radiusSearch(query, 50, indices, distances, 0);

  std::stringstream ss;
  const bool is_ndt_mode = count > pcd_density_threshold_;
  std::unordered_map<PoseEstimatorName, bool> toggle_list;
  if (is_ndt_mode) {
    toggle_list[PoseEstimatorName::NDT] = true;
    toggle_list[PoseEstimatorName::YABLOC] = false;
    toggle_list[PoseEstimatorName::EAGLEYE] = false;
    ss << "enable NDT";
  } else {
    toggle_list[PoseEstimatorName::NDT] = false;
    toggle_list[PoseEstimatorName::YABLOC] = true;
    toggle_list[PoseEstimatorName::EAGLEYE] = false;
    ss << "enable YabLoc";
  }
  ss << "\npcd occupancy: " << count << " > " << pcd_density_threshold_;
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

  //
  for (const auto & marker : eagleye_area_.debug_marker_array().markers) {
    array_msg.markers.push_back(marker);
  }

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