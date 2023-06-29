#include "pose_estimator_manager/grid_info.hpp"
#include "pose_estimator_manager/pose_estimator_manager.hpp"

#include <pcl_conversions/pcl_conversions.h>

#include <sstream>
namespace multi_pose_estimator
{
PoseEstimatorManager::PoseEstimatorManager()
: Node("pose_estimator_manager"),
  pcd_density_threshold(declare_parameter<int>("pcd_density_threshold"))
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  const auto map_qos = rclcpp::QoS(1).transient_local().reliable();

  // Publisher
  pub_grid_marker_ =
    create_publisher<PointCloud2>("/pcd_occupied_area", rclcpp::QoS(1).transient_local());
  pub_debug_string_ = create_publisher<String>("/debug/pcd_ratio_string", 10);

  // Subscriber
  auto on_map = std::bind(&PoseEstimatorManager::on_map, this, _1);
  sub_map_ = create_subscription<PointCloud2>("/map/pointcloud_map", map_qos, on_map);
  auto on_pose_cov = std::bind(&PoseEstimatorManager::on_pose_cov, this, _1);
  sub_pose_cov_ =
    create_subscription<PoseCovStamped>("/localization/pose_with_covariance", 10, on_pose_cov);

  // Service server
  auto on_service = std::bind(&PoseEstimatorManager::on_service, this, _1, _2);
  switch_service_server_ = create_service<SetBool>("/toggle_ndt", on_service);

  // Service client
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  clients_.emplace(
    "ndt", std::make_shared<ManagerClient>(this, "/ndt_enable_srv", service_callback_group_));
  clients_.emplace(
    "yabloc", std::make_shared<ManagerClient>(this, "/yabloc_enable_srv", service_callback_group_));

  // Timer callback
  auto on_timer = std::bind(&PoseEstimatorManager::on_timer, this);
  timer_ =
    rclcpp::create_timer(this, this->get_clock(), rclcpp::Rate(1).period(), std::move(on_timer));

  toggle_mode(true);
}

bool PoseEstimatorManager::toggle_mode(bool enable_ndt)
{
  RCLCPP_INFO_STREAM(get_logger(), "ENTER: toggle pose estimator");
  if (enable_ndt) {
    clients_.at("ndt")->enable();
    clients_.at("yabloc")->disable();
  } else {
    clients_.at("ndt")->disable();
    clients_.at("yabloc")->enable();
  }
  RCLCPP_INFO_STREAM(get_logger(), "EXIT: toggle pose estimator");
  return true;
}

void PoseEstimatorManager::on_service(
  SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response)
{
  toggle_mode(request->data);
  response->success = true;

  RCLCPP_INFO_STREAM(get_logger(), "override mode and disable timer callback");
  timer_->reset();
}

void PoseEstimatorManager::on_map(PointCloud2::ConstSharedPtr msg)
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

  publish_occupied_area(msg->header);
  RCLCPP_INFO_STREAM(
    get_logger(), "on_map: PCD map initialization is done " << occupied_areas_->size());
}

void PoseEstimatorManager::publish_occupied_area(const std_msgs::msg::Header & header)
{
  // Convert to msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*occupied_areas_, cloud_msg);
  cloud_msg.header = header;
  pub_grid_marker_->publish(cloud_msg);
}

void PoseEstimatorManager::on_timer()
{
  RCLCPP_INFO_STREAM(get_logger(), "on_timer");

  if (!latest_pose_.has_value()) {
    RCLCPP_WARN_STREAM(
      get_logger(), "Unable to determine which estimation to use, due to lack of latest position");
    return;
  }

  auto position = latest_pose_->pose.pose.position;
  pcl::PointXYZ query(position.x, position.y, position.z);

  std::vector<int> indices;
  std::vector<float> distances;
  const int count = kdtree_->radiusSearch(query, 50, indices, distances, 0);
  const bool is_ndt_mode = count > pcd_density_threshold;
  toggle_mode(is_ndt_mode);

  {
    String msg;
    std::stringstream ss;
    ss << "PCD occupancy: " << count << " > " << pcd_density_threshold << "\n";
    ss << (is_ndt_mode ? "NDT" : "YabLoc");

    msg.data = ss.str();
    pub_debug_string_->publish(msg);
  }
}

void PoseEstimatorManager::on_pose_cov(PoseCovStamped::ConstSharedPtr msg)
{
  latest_pose_ = *msg;
}

}  // namespace multi_pose_estimator