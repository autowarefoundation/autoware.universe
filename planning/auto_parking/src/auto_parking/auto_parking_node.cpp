// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "auto_parking/auto_parking_node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <tier4_autoware_utils/geometry/geometry.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

using lanelet::utils::getId;
using lanelet::utils::to2D;
using Pose = geometry_msgs::msg::Pose;
using PoseStamped = geometry_msgs::msg::PoseStamped;

namespace 
{
// get nearest parking lot by distance
std::shared_ptr<lanelet::ConstPolygon3d> filterNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);
  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  *linked_parking_lot = *std::min_element(all_parking_lots.begin(), all_parking_lots.end(),
    [&](const lanelet::ConstPolygon3d& p1, const lanelet::ConstPolygon3d& p2) {
    const double dist_p1 = boost::geometry::distance(current_position, to2D(p1).basicPolygon());
    const double dist_p2 = boost::geometry::distance(current_position, to2D(p2).basicPolygon());
    return dist_p1 < dist_p2;
  });
  if (linked_parking_lot) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

Pose transformPose(const Pose & pose, const TransformStamped & transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}
}  // namespace

namespace auto_parking
{
void AutoParkingNode::goalPublisher(const PoseStamped msg)
{
  goal_pose_pub_->publish(msg);
}

TransformStamped AutoParkingNode::getTransform(
  const std::string & from, const std::string & to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void AutoParkingNode::onMap(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void AutoParkingNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_ = msg;

  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;
  stop_checker_->addTwist(twist);
}

void AutoParkingNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void AutoParkingNode::onSetActiveStatus(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  if(req->data) {
    reset();
    active_ = true;
  } else {
    active_ = false;
  }
  res->success = true;
  return;
}

PlannerCommonParam AutoParkingNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");
  p.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");
  p.maximum_turning_radius = declare_parameter<double>("maximum_turning_radius");
  p.turning_radius_size = declare_parameter<int>("turning_radius_size");
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

bool AutoParkingNode::isArrived(const Pose& goal)
{
  // Check distance to current goal
  if (node_param_.th_arrived_distance_m < tier4_autoware_utils::calcDistance2d(current_pose_.pose, goal)) {
    return false;
  }
  return true;
}

void AutoParkingNode::filterGoalPoseinParkingLot(const lanelet::ConstLineString3d center_line, 
                                                 Pose& goal)
{
  for(size_t i = 0; i < center_line.size(); i++){
    const lanelet::Point3d search_point(lanelet::InvalId, center_line[i].x(), center_line[i].y(), 0.0);
    if(lanelet::geometry::within(search_point, nearest_parking_lot_->basicPolygon())){
      goal.position.x = center_line[i].x();
      goal.position.y = center_line[i].y();
      goal.position.z = 0.0;
      const auto yaw = std::atan2(
        center_line[i+1].y() - center_line[i].y(), center_line[i+1].x() - center_line[i].x());
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      goal.orientation = tf2::toMsg(q);
      return;
    }
  }
}

bool AutoParkingNode::isInParkingLot()
{
  const auto & p = current_pose_.pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);
  if(lanelet::geometry::within(search_point, nearest_parking_lot_->basicPolygon())){
    return true;
  }
  return false;
}

bool AutoParkingNode::initAutoParking()
{
  const auto & p = current_pose_.pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  nearest_parking_lot_ =
    filterNearestParkinglot(lanelet_map_ptr_, search_point.basicPoint2d());
  if(!nearest_parking_lot_){
    RCLCPP_INFO(get_logger(), "No parking lot found!!");
    return false;
  }

  const auto & all_parking_spaces_ = 
    lanelet::utils::query::getAllParkingSpaces(lanelet_map_ptr_);

  nearest_parking_spaces_ = 
    lanelet::utils::query::getLinkedParkingSpaces(*nearest_parking_lot_, all_parking_spaces_);

  const auto & all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const auto & all_road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  parking_lot_lanelets_ = 
    lanelet::utils::query::getLinkedLanelets(*nearest_parking_lot_, all_road_lanelets);

  // Get closest lanelet to ego-vehicle
  if(!lanelet::utils::query::getClosestLanelet(all_road_lanelets, 
      current_pose_.pose, &current_lanelet_)){
    RCLCPP_INFO(get_logger(), "No close lanelet to vehicle found!!");
    return false;
  }

  // Order parking lot lanelets by route length  
  // - Sort lanelets
  std::sort(parking_lot_lanelets_.begin(), parking_lot_lanelets_.end(), 
      [&](const lanelet::ConstLanelet& l1, const lanelet::ConstLanelet& l2){ 
      const auto l1_length = routing_graph_ptr_->shortestPath(current_lanelet_, l1, {}, false)->size();
      const auto l2_length = routing_graph_ptr_->shortestPath(current_lanelet_, l2, {}, false)->size();
      return ( l1_length < l2_length);
      });

  // Set parking lot goal, goal at exit of parking lot
  // Assuming parking lot has one entry, longest route is from
  // entry to exit.
  lanelet::ConstLineString3d exit_llt_cline = 
  parking_lot_lanelets_.back().centerline(); // final lanelet from sorted lanelets
  filterGoalPoseinParkingLot(exit_llt_cline, parking_goal_);
  
  // print out sortet lanelet ID's for DEBUG.
  // for (const auto & lanelet : parking_lot_lanelets_) {
  //   // check if parking space is close to lanelet
  //   RCLCPP_INFO(get_logger(),"LaneId: %li \n", getId(lanelet));
  // }

  return true;
}

bool AutoParkingNode::findParkingSpace()
{
  const auto & p = current_pose_.pose.position;
  const lanelet::Point2d search_point(lanelet::InvalId, p.x, p.y);

  // Set occupancy map and current pose
  algo_->setMap(*occupancy_grid_);
  const auto current_pose_in_costmap_frame = transformPose(
  current_pose_.pose,
  getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

  // Cycle through parking spaces
  for(const auto & parking_space : nearest_parking_spaces_){
    const double dist = boost::geometry::distance(to2D(parking_space).basicLineString(), search_point);
    // Check if parking space is nearby                                 
    if(node_param_.th_parking_space_distance_m < dist){
      continue;
    }

    // Compute two goal poses for each parking space
    double distance_thresh = boost::geometry::distance(parking_space.front().basicPoint(), 
                                                       parking_space.back().basicPoint()) / 4;
    lanelet::ConstPoints3d p_space_fw_bk;
    p_space_fw_bk.push_back(parking_space.back());
    p_space_fw_bk.push_back(parking_space.front());

    // Compute parking space poses, orientations i = 0 , 1 
    // Check if freespace trajectory available then set parking space goal
    // Check back to front pose first then opposite

    for(auto i = 0; i < 2; i++){

      auto in1 = i % 2;
      auto in2 = (i + 1) % 2;

      Eigen::Vector3d direction =
        p_space_fw_bk[in2].basicPoint() - p_space_fw_bk[in1].basicPoint();
      direction.normalize();

      // Set pose 
      const Eigen::Vector3d goal_pose = p_space_fw_bk[in1].basicPoint() + direction * distance_thresh;
      const auto yaw = std::atan2(direction.y(), direction.x());
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      
      Pose goal;
      goal.position.x = goal_pose.x();
      goal.position.y = goal_pose.y();
      goal.position.z = goal_pose.z();
      goal.orientation = tf2::toMsg(q);

      current_goal_.pose = goal;
      current_goal_.header.frame_id = odom_->header.frame_id;
      current_goal_.header.stamp = rclcpp::Time();

      const auto goal_pose_in_costmap_frame = transformPose(
      current_goal_.pose, 
      getTransform(occupancy_grid_->header.frame_id, current_goal_.header.frame_id));
      bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);

      if(result){
        RCLCPP_INFO(get_logger(), "Auto-parking: Found free parking space!");
        return true;
      }

      continue;
    }
  }

  // No free parking spaces found
  return false;
}

void AutoParkingNode::onEngage(EngageMsg::ConstSharedPtr msg)
{
  is_engaged_ = msg->engage;
}

void AutoParkingNode::engageAutonomous()
{
  // engage
  auto req = std::make_shared<EngageSrv::Request>();
  req->engage = true;
  RCLCPP_INFO(get_logger(), "Auto-parking: auto-engage client request");
  if (!client_engage_->service_is_ready()) {
    RCLCPP_INFO(get_logger(), "Auto-parking: auto-engage client is unavailable");
    return;
  }
  client_engage_->async_send_request(
    req, []([[maybe_unused]] rclcpp::Client<EngageSrv>::SharedFuture result) {});
}

void AutoParkingNode::reset()
{
  set_parking_lot_goal_ = false;
  set_parking_space_goal_ = false;
  active_ = false;
}

void AutoParkingNode::onTimer()
{
  // Publish active status message
  std_msgs::msg::Bool is_active_msg;
  is_active_msg.data = active_;
  active_status_pub_->publish(is_active_msg);

  if (!active_){
    return;
  }

  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  // Check all inputs are ready
  if (!odom_ || !lanelet_map_ptr_ || !routing_graph_ptr_ ||
      !engage_sub_ || !client_engage_ ||
      current_pose_.header.frame_id == "") {
    active_ = false;
    return;
  }

  // Publish parking lot entrance goal
  if(!set_parking_lot_goal_){
    // Initialize variables
    if(!initAutoParking()){ 
      RCLCPP_INFO(get_logger(), "Auto-parking: Initialization failed!");
      active_ = false;
      return; 
    }
    current_goal_.header.frame_id = odom_->header.frame_id;
    current_goal_.header.stamp = rclcpp::Time();
    current_goal_.pose = parking_goal_;
    goalPublisher(current_goal_);
    RCLCPP_INFO(get_logger(), "Auto-parking: Publishing parking lot goal");
    set_parking_lot_goal_ = true;
  }

  // Arrived at parking lot exit without finding parking space
  if(set_parking_lot_goal_ && isArrived(parking_goal_)){
    RCLCPP_INFO(get_logger(), "Auto-parking: Failed to find parking space");
    active_ = false;
    return;
  }
  
  if(isInParkingLot() && occupancy_grid_)
  {
    // Search parking spaces if inside parking lot
    if(!set_parking_space_goal_)
    { 
      RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 2000, "Auto-parking: Searching...");
      if(findParkingSpace()){
        goalPublisher(current_goal_);
        RCLCPP_INFO(get_logger(), "Auto-parking: Publishing parking space goal");
        set_parking_space_goal_ = true;
      }
    }
    // if parking space goal set
    // check if astar goal is still valid, else replan
    else
    {
      // Set occupancy map and current pose
      algo_->setMap(*occupancy_grid_);
      const auto current_pose_in_costmap_frame = transformPose(
      current_pose_.pose,
      getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));
      const auto goal_pose_in_costmap_frame = transformPose(
      current_goal_.pose, 
      getTransform(occupancy_grid_->header.frame_id, current_goal_.header.frame_id));
      bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
      if(!result) { 
        set_parking_space_goal_ = false;
        current_goal_.pose = parking_goal_;
        goalPublisher(current_goal_);
        return;
      }
    }
  }

  // Arrived at parking space goal
  if(set_parking_space_goal_ && isArrived(current_goal_.pose)){
    RCLCPP_INFO(get_logger(), "Auto-parking: Complete!");
    active_ = false;
    return;
  }

  // If ego stopped/stuck replan: TODO
  // const auto is_vehicle_stopped = stop_checker_->isVehicleStopped(1.0);

  // Engage autonomous once a goal is set
  // For smooth transition remove vehicle stopping after 
  // publishing goal pose in freespace planner node
  if(!is_engaged_ && !isArrived(current_goal_.pose)){
    engageAutonomous();
  }
}

AutoParkingNode::AutoParkingNode(const rclcpp::NodeOptions & node_options)
: Node("auto_parking", node_options)
{

  // Auto-park params
  {
    auto & p = node_param_;
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_parking_space_distance_m = declare_parameter<double>("th_parking_space_distance_m");
    p.update_rate = declare_parameter<double>("update_rate");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
  }

  // set vehicle_info for astar
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m + node_param_.vehicle_shape_margin_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m + node_param_.vehicle_shape_margin_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m + node_param_.vehicle_shape_margin_m / 2;
  }

  // Subscribers
  {
    lanelet_map_sub_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "~/input/lanelet_map_bin", rclcpp::QoS{10}.transient_local(),
    std::bind(&AutoParkingNode::onMap, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odometry", rclcpp::QoS{100},
    std::bind(&AutoParkingNode::onOdometry, this, std::placeholders::_1));

    engage_sub_ = create_subscription<EngageMsg>(
    "~/input/engage", rclcpp::QoS{5}, std::bind(&AutoParkingNode::onEngage, this, std::placeholders::_1));

    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", 10, std::bind(&AutoParkingNode::onOccupancyGrid, this, std::placeholders::_1));
  }
  
  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    goal_pose_pub_ = this->create_publisher<PoseStamped>("~/output/fixed_goal", qos);
    active_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/output/active_status", qos);
  }

  // Service
  {
    srv_set_active_ = create_service<std_srvs::srv::SetBool>(
    "~/service/set_active", 
    std::bind(
    &AutoParkingNode::onSetActiveStatus, this, std::placeholders::_1, std::placeholders::_2));
  }
  
  // Client
  {
    client_engage_ = this->create_client<EngageSrv>("~/service/engage");
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&AutoParkingNode::onTimer, this));
  }

  // Initialize Freespace planning algorithm - astar
  {
    const auto planner_common_param = getPlannerCommonParam();
    algo_ = std::make_unique<AstarSearch>(planner_common_param, vehicle_shape_, *this);
  }

  reset();
  stop_checker_ = std::make_unique<motion_utils::VehicleStopCheckerBase>(this, 1.0 + 1.0);
  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(auto_parking::AutoParkingNode)
