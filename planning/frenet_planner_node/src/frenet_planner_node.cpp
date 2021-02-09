// Copyright 2022 Tier IV, Inc.
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

#include "frenet_planner_node/frenet_planner_node.hpp"

#include "frenet_planner/plot/debug_window.hpp"
#include "frenet_planner/structures.hpp"
#include "frenet_planner/trajectory_reuse.hpp"
#include "frenet_planner/transform/spline_transform.hpp"
#include "frenet_planner_node/prepare_inputs.hpp"
#include "frenet_planner_node/trajectory_generation.hpp"
#include "frenet_planner_node/utils/occupancy_grid_to_polygons.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include <autoware_auto_planning_msgs/msg/detail/path__struct.hpp>
#include <autoware_auto_planning_msgs/msg/detail/trajectory_point__struct.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace frenet_planner_node
{
FrenetPlannerNode::FrenetPlannerNode(const rclcpp::NodeOptions & node_options)
: Node("frenet_planner_node", node_options),
  qapplication_(argc_, argv_.data()),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  w_.show();
  trajectory_pub_ =
    create_publisher<autoware_auto_planning_msgs::msg::Trajectory>("~/output/trajectory", 1);

  path_sub_ = create_subscription<autoware_auto_planning_msgs::msg::Path>(
    "~/input/path", rclcpp::QoS{1},
    std::bind(&FrenetPlannerNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", rclcpp::QoS{1},
    std::bind(&FrenetPlannerNode::odomCallback, this, std::placeholders::_1));
  objects_sub_ = create_subscription<autoware_auto_perception_msgs::msg::PredictedObjects>(
    "~/input/objects", rclcpp::QoS{10},
    std::bind(&FrenetPlannerNode::objectsCallback, this, std::placeholders::_1));

  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>();

  // This is necessary to interact with the GUI even when we are not generating trajectories
  gui_process_timer_ =
    create_wall_timer(std::chrono::milliseconds(100), []() { QCoreApplication::processEvents(); });
}

// ROS callback functions
void FrenetPlannerNode::pathCallback(const autoware_auto_planning_msgs::msg::Path::SharedPtr msg)
{
  frenet_planner::Debug debug;
  const auto current_pose = getCurrentEgoPose();
  if (
    msg->points.size() < 2 || msg->drivable_area.data.empty() || !current_pose ||
    !current_twist_ptr_) {
    RCLCPP_INFO(
      get_logger(),
      "[pathCallback] incomplete inputs: current_pose: %d | drivable_area: %d | path points: %ld",
      current_pose.has_value(), !msg->drivable_area.data.empty(), msg->points.size());
    return;
  }

  // TODO(Maxime CLEMENT): use common StopWatch
  const auto calc_begin = std::chrono::steady_clock::now();

  const auto path_spline = preparePathSpline(*msg);
  const auto constraints = prepareConstraints(msg->drivable_area, *in_objects_ptr_);

  const auto trajectories = generateCandidateTrajectories(
    *current_pose, current_twist_ptr_->twist, *msg, path_spline, prev_trajectory_, constraints,
    debug);
  const auto selected_trajectory = selectBestTrajectory(trajectories);
  if (selected_trajectory) {
    // combine newly computed trajectory with the base trajectory
    publishTrajectory(*selected_trajectory, msg);
    prev_trajectory_ = *selected_trajectory;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "[FrenetPlanner] All candidates rejected: lat_dev=%lu vel=%lu coll=%lu curv=%lu",
      debug.nb_constraint_violations.lateral_deviation, debug.nb_constraint_violations.velocity,
      debug.nb_constraint_violations.collision, debug.nb_constraint_violations.curvature);
    publishTrajectory(prev_trajectory_, msg);
  }
  std::chrono::steady_clock::time_point calc_end = std::chrono::steady_clock::now();

  // Plot //
  const auto plot_begin = calc_end;
  w_.plotter_->plotPolygons(constraints.obstacle_polygons);
  std::vector<double> x;
  std::vector<double> y;
  x.reserve(msg->points.size());
  y.reserve(msg->points.size());
  for (const auto & p : msg->points) {
    x.push_back(p.pose.position.x);
    y.push_back(p.pose.position.y);
  }
  w_.plotter_->plotReferencePath(x, y);
  w_.plotter_->plotTrajectories(trajectories);
  // w_.plotter_->plotCommittedTrajectory(base_trajectory);  TODO(Maxime CLEMENT): fix this
  if (selected_trajectory) w_.plotter_->plotSelectedTrajectory(*selected_trajectory);
  w_.plotter_->plotCurrentPose(path_spline.frenet(*current_pose), *current_pose);
  w_.replot();
  QCoreApplication::processEvents();
  w_.update();
  std::chrono::steady_clock::time_point plot_end = std::chrono::steady_clock::now();
  w_.setStatus(
    trajectories.size(),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(calc_end - calc_begin).count()),
    static_cast<double>(
      std::chrono::duration_cast<std::chrono::milliseconds>(plot_end - plot_begin).count()));
}

std::optional<frenet_planner::Trajectory> FrenetPlannerNode::selectBestTrajectory(
  const std::vector<frenet_planner::Trajectory> & trajectories)
{
  auto min_cost = std::numeric_limits<double>::max();
  std::optional<frenet_planner::Trajectory> traj;
  for (const auto & trajectory : trajectories) {
    if (trajectory.valid && trajectory.cost < min_cost) {
      min_cost = trajectory.cost;
      traj = trajectory;
    }
  }
  return traj;
}

void FrenetPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ptr_->header = msg->header;
  current_twist_ptr_->twist = msg->twist.twist;
}

std::optional<frenet_planner::Point> FrenetPlannerNode::getCurrentEgoPose()
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer_.lookupTransform(
      "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "[FrenetPlannerNode] %s", ex.what());
    return {};
  }

  return frenet_planner::Point(
    tf_current_pose.transform.translation.x, tf_current_pose.transform.translation.y);
}

void FrenetPlannerNode::objectsCallback(
  const autoware_auto_perception_msgs::msg::PredictedObjects::SharedPtr msg)
{
  in_objects_ptr_ = std::make_unique<autoware_auto_perception_msgs::msg::PredictedObjects>(*msg);
}

void FrenetPlannerNode::publishTrajectory(
  const frenet_planner::Trajectory & trajectory,
  const autoware_auto_planning_msgs::msg::Path::SharedPtr path_msg)
{
  if (trajectory.points.size() < 2) {
    return;
  }
  tf2::Quaternion q;  // to convert yaw angle to Quaternion orientation
  autoware_auto_planning_msgs::msg::Trajectory traj_msg;
  traj_msg.header.frame_id = path_msg->header.frame_id;
  traj_msg.header.stamp = now();
  for (size_t i = 0; i < trajectory.points.size() - 2; ++i) {
    autoware_auto_planning_msgs::msg::TrajectoryPoint point;
    point.pose.position.x = trajectory.points[i].x();
    point.pose.position.y = trajectory.points[i].y();
    q.setRPY(0, 0, trajectory.yaws[i]);
    point.pose.orientation = tf2::toMsg(q);
    // TODO(Maxime CLEMENT): option to use/not use velocity profile
    point.longitudinal_velocity_mps =
      15.0;  // static_cast<float>(trajectory.longitudinal_velocities[i]);
    point.acceleration_mps2 = static_cast<float>(trajectory.longitudinal_accelerations[i]);
    point.lateral_velocity_mps = static_cast<float>(trajectory.lateral_velocities[i]);
    point.front_wheel_angle_rad = static_cast<float>(trajectory.curvatures[i]);
    point.rear_wheel_angle_rad = 0.0f;
    traj_msg.points.push_back(point);
  }
  trajectory_pub_->publish(traj_msg);
}

}  // namespace frenet_planner_node

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(frenet_planner_node::FrenetPlannerNode)
