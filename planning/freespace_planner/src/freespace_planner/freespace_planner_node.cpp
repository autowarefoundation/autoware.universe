// Copyright 2021 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Robotec.AI sp. z o.o.

#include "freespace_planner/freespace_planner.hpp"

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>


namespace autoware
{
namespace planning
{
namespace freespace_planner
{

using autoware_auto_planning_msgs::action::PlannerCostmap;
using autoware_auto_planning_msgs::msg::Trajectory;
using common::vehicle_constants_manager::declare_and_get_vehicle_constants;


geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

astar_search::AstarWaypoints adjustWaypointsSize(
  const astar_search::AstarWaypoints & astar_waypoints)
{
  auto max_length = Trajectory::CAPACITY;
  auto input_length = astar_waypoints.waypoints.size();

  if (input_length > max_length) {
    astar_search::AstarWaypoints resized_vector;
    resized_vector.header = astar_waypoints.header;

    // input_length substraction to handle handle max_length multiplicity
    auto elements_to_skip_per_step = static_cast<int64_t>(std::floor(input_length / max_length));
    // must be +1 to actually skip an element
    auto stride = elements_to_skip_per_step + 1;

    auto waypoints_iter = astar_waypoints.waypoints.begin();
    auto points_to_skip = static_cast<int64_t>(input_length - max_length);
    int64_t skipped_points_count = 0;
    // substract by elements_to_skip_per_step to prevent from skipping too many points
    while (skipped_points_count < points_to_skip) {
      resized_vector.waypoints.push_back(*waypoints_iter);
      waypoints_iter += std::min(stride, points_to_skip - skipped_points_count + 1);
      skipped_points_count += elements_to_skip_per_step;
    }

    // copy rest of waypoints
    resized_vector.waypoints.insert(
      resized_vector.waypoints.end(), waypoints_iter, astar_waypoints.waypoints.end());
    return resized_vector;
  }

  return astar_waypoints;
}

autoware_auto_planning_msgs::msg::Trajectory createTrajectory(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const astar_search::AstarWaypoints & astar_waypoints,
  const float & velocity)
{
  Trajectory trajectory;
  trajectory.header = astar_waypoints.header;

  for (const auto & awp : astar_waypoints.waypoints) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;

    point.pose = awp.pose.pose;
    point.pose.position.z = current_pose.pose.position.z;  // height = const

    // switch sign by forward/backward
    // velocity = const
    point.longitudinal_velocity_mps = (awp.is_back ? -1.0f : 1.0f) * (velocity / 3.6f);

    trajectory.points.push_back(point);
  }

  return trajectory;
}

FreespacePlannerNode::FreespacePlannerNode(const rclcpp::NodeOptions & node_options)
: Node("freespace_planner", node_options)
{
  using std::placeholders::_1;
  using namespace std::literals::chrono_literals;

  auto throw_if_negative = [](int64_t number, const std::string & name) {
      if (number < 0) {
        throw std::runtime_error(
                name + " = " + std::to_string(number) +
                " shouldn't be negative.");
      }
    };

  // NodeParam
  {
    node_param_.waypoints_velocity = declare_parameter("waypoints_velocity", 5.0);
  }

  // AstarParam
  {
    // base configs
    astar_param_.use_back = declare_parameter("use_back", true);
    astar_param_.only_behind_solutions = declare_parameter("only_behind_solutions", false);
    astar_param_.time_limit = declare_parameter("time_limit", 5000.0);

    auto vehicle_dimension_margin = declare_parameter("vehicle_dimension_margin", 0.0);
    auto vehicle_constants = declare_and_get_vehicle_constants(*this);
    astar_param_.robot_shape.length = vehicle_constants.vehicle_length + vehicle_dimension_margin;
    astar_param_.robot_shape.width = vehicle_constants.vehicle_width + vehicle_dimension_margin;
    astar_param_.robot_shape.cg2back =
      vehicle_constants.cg_to_rear + vehicle_constants.overhang_rear +
      (vehicle_dimension_margin / 2.0);
    astar_param_.minimum_turning_radius = vehicle_constants.minimum_turning_radius;

    astar_param_.maximum_turning_radius = declare_parameter("maximum_turning_radius", 6.0);
    astar_param_.maximum_turning_radius = std::max(
      astar_param_.maximum_turning_radius,
      astar_param_.minimum_turning_radius);
    auto tr_size = declare_parameter("turning_radius_size", 11);
    throw_if_negative(tr_size, "turning_radius_size");
    astar_param_.turning_radius_size = static_cast<size_t>(tr_size);

    // search configs
    auto th_size = declare_parameter("theta_size", 48);
    throw_if_negative(th_size, "theta_size");
    astar_param_.theta_size = static_cast<size_t>(th_size);
    astar_param_.reverse_weight = declare_parameter("reverse_weight", 2.00);
    astar_param_.goal_lateral_tolerance = declare_parameter("goal_lateral_tolerance", 0.25);
    astar_param_.goal_longitudinal_tolerance =
      declare_parameter("goal_longitudinal_tolerance", 1.0);
    astar_param_.goal_angular_tolerance = declare_parameter("goal_angular_tolerance", 0.05236);

    // costmap configs
    astar_param_.obstacle_threshold = declare_parameter("obstacle_threshold", 100);
    astar_param_.distance_heuristic_weight = declare_parameter("distance_heuristic_weight", 1.0);
  }

  // Hybrid A* implementation
  {
    astar_ = std::make_unique<astar_search::AstarSearch>(astar_param_);
  }

  // Publisher
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_debug_pub_ =
      create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/debug/trajectory", qos);
    pose_array_trajectory_debug_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>("~/debug/trajectory_pose_array", qos);
  }

  // Action client
  {
    map_client_ = rclcpp_action::create_client<PlannerCostmapAction>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "generate_costmap");

    while (!map_client_->wait_for_action_server(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map server.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for costmap generator action service...");
    }
  }

  // Action service
  {
    plan_trajectory_srv_ = rclcpp_action::create_server<PlanTrajectoryAction>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "plan_parking_trajectory",
      [this](auto uuid, auto goal) {return this->handleGoal(uuid, goal);},
      [this](auto goal_handle) {return this->handleCancel(goal_handle);},
      [this](auto goal_handle) {return this->handleAccepted(goal_handle);});
  }

  // TF
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  }
}

rclcpp_action::GoalResponse FreespacePlannerNode::handleGoal(
  const rclcpp_action::GoalUUID &,
  const std::shared_ptr<const PlanTrajectoryAction::Goal>)
{
  if (isPlanning()) {
    RCLCPP_WARN(get_logger(), "Planner is already planning. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received new goal.");
  startPlanning();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FreespacePlannerNode::handleCancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Cancelling trajectory planning.");
  stopPlanning();
  auto result = std::make_shared<PlanTrajectoryAction::Result>();
  result->result = PlanTrajectoryAction::Result::FAIL;
  goal_handle->canceled(result);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void FreespacePlannerNode::handleAccepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  RCLCPP_DEBUG(this->get_logger(), "Planning.");

  planning_goal_handle_ = goal_handle;

  // acquire start and goal position from action request
  start_pose_.header = goal_handle->get_goal()->sub_route.header;
  start_pose_.pose = goal_handle->get_goal()->sub_route.start_pose;

  goal_pose_.header = goal_handle->get_goal()->sub_route.header;
  goal_pose_.pose = goal_handle->get_goal()->sub_route.goal_pose;

  // request costmap and plan trajectory
  auto action_goal = PlannerCostmapAction::Goal();
  action_goal.route = goal_handle->get_goal()->sub_route;

  auto send_goal_options = rclcpp_action::Client<PlannerCostmapAction>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
    &FreespacePlannerNode::goalResponseCallback,
    this,
    _1);
  send_goal_options.feedback_callback = std::bind(
    &FreespacePlannerNode::feedbackCallback,
    this,
    _1,
    _2);
  send_goal_options.result_callback = std::bind(
    &FreespacePlannerNode::resultCallback,
    this,
    _1);

  map_client_->async_send_goal(action_goal, send_goal_options);
  RCLCPP_INFO(get_logger(), "Costmap generator action goal sent.");
}

void FreespacePlannerNode::goalResponseCallback(
  std::shared_future<PlannerCostmapGoalHandle::SharedPtr> future)
{
  if (!future.get()) {
    RCLCPP_ERROR(get_logger(), "Costmap generator goal was rejected by server.");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(result);
    stopPlanning();
    return;
  }

  RCLCPP_INFO(get_logger(), "Costmap generator action goal accepted.");
}

void FreespacePlannerNode::resultCallback(
  const PlannerCostmapGoalHandle::WrappedResult & costmap_result)
{
  auto planning_result = std::make_shared<PlanTrajectoryAction::Result>();

  RCLCPP_INFO(get_logger(), "Costmap received, planning started.");

  occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap_result.result->costmap);

  bool success = planTrajectory();

  if (success) {
    visualizeTrajectory();
    planning_result->result = PlanTrajectoryAction::Result::SUCCESS;
    planning_result->trajectory = trajectory_;
    planning_goal_handle_->succeed(planning_result);
  } else {
    planning_result->result = PlanTrajectoryAction::Result::FAIL;
    planning_goal_handle_->abort(planning_result);
  }

  stopPlanning();
}

bool FreespacePlannerNode::planTrajectory()
{
  // reset inner state
  reset();

  // Supply latest costmap to planning algorithm
  astar_->setOccupancyGrid(*occupancy_grid_.get());

  // Calculate poses in costmap frame
  const auto start_pose_in_costmap_frame = transformPose(
    start_pose_.pose,
    getTransform(occupancy_grid_->header.frame_id, start_pose_.header.frame_id));

  const auto goal_pose_in_costmap_frame = transformPose(
    goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));

  // execute astar search
  const rclcpp::Time start = get_clock()->now();
  auto search_status = astar_->makePlan(start_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Astar planning took %f [s]", (end - start).seconds());

  if (astar_search::isSuccess(search_status)) {
    RCLCPP_INFO(get_logger(), "Plan found.");
    auto waypoints = adjustWaypointsSize(astar_->getWaypoints());
    trajectory_ =
      createTrajectory(start_pose_, waypoints, static_cast<float>(node_param_.waypoints_velocity));
  } else {
    switch (search_status) {
      case astar_search::SearchStatus::FAILURE_COLLISION_AT_START:
        RCLCPP_ERROR(
          get_logger(), "Cannot find plan because collision was detected in start position.");
        break;
      case astar_search::SearchStatus::FAILURE_COLLISION_AT_GOAL:
        RCLCPP_ERROR(
          get_logger(), "Cannot find plan because collision was detected in goal position.");
        break;
      case astar_search::SearchStatus::FAILURE_TIMEOUT_EXCEEDED:
        RCLCPP_ERROR(get_logger(), "Cannot find plan because timeout exceeded.");
        break;
      case astar_search::SearchStatus::FAILURE_NO_PATH_FOUND:
        RCLCPP_ERROR(get_logger(), "Cannot find plan.");
        break;
      default:
        RCLCPP_ERROR(get_logger(), "SearchStatus not handled.");
        break;
    }
  }

  return astar_search::isSuccess(search_status);
}

bool FreespacePlannerNode::isPlanning() const
{
  return state_ == FreespacePlannerState::PLANNING;
}
void FreespacePlannerNode::startPlanning()
{
  state_ = FreespacePlannerState::PLANNING;
}
void FreespacePlannerNode::stopPlanning()
{
  state_ = FreespacePlannerState::IDLE;
}

void FreespacePlannerNode::reset()
{
  trajectory_ = autoware_auto_planning_msgs::msg::Trajectory();
}

geometry_msgs::msg::TransformStamped FreespacePlannerNode::getTransform(
  const std::string & from, const std::string & to)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

void FreespacePlannerNode::visualizeTrajectory()
{
  auto debug_pose_array_trajectory = geometry_msgs::msg::PoseArray();

  debug_pose_array_trajectory.header = trajectory_.header;

  for (const auto & trajectory_point : trajectory_.points) {
    debug_pose_array_trajectory.poses.push_back(trajectory_point.pose);
  }

  // publish visualization friendly pose array
  pose_array_trajectory_debug_pub_->publish(debug_pose_array_trajectory);

  // publish trajectory as-is
  trajectory_debug_pub_->publish(trajectory_);
}

}  // namespace freespace_planner
}  // namespace planning
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning::freespace_planner::FreespacePlannerNode)
