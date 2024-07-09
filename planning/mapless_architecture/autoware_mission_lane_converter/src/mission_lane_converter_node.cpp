// Copyright 2024 driveblocks GmbH, authors: Simon Eisenmann, Thomas Herrmann
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/mission_lane_converter/mission_lane_converter_node.hpp"

#include "autoware/local_mission_planner_common/helper_functions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/path.hpp"
#include "autoware_planning_msgs/msg/path_point.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <functional>

namespace autoware::mapless_architecture
{
using std::placeholders::_1;

MissionLaneConverterNode::MissionLaneConverterNode(const rclcpp::NodeOptions & options)
: Node("mission_lane_converter_node", options)
{
  // Set quality of service to best effort (if transmission fails, do not try to
  // resend but rather use new sensor data)
  // the history_depth is set to 1 (message queue size)
  auto qos_best_effort = rclcpp::QoS(1);
  qos_best_effort.best_effort();

  auto qos_reliability = rclcpp::QoS(1);
  qos_reliability.reliability();

  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "mission_lane_converter/input/odometry", qos_best_effort,
    std::bind(&MissionLaneConverterNode::CallbackOdometryMessages_, this, _1));

  mission_lane_subscriber_ =
    this->create_subscription<autoware_planning_msgs::msg::MissionLanesStamped>(
      "mission_lane_converter/input/mission_lanes", qos_best_effort,
      std::bind(&MissionLaneConverterNode::MissionLanesCallback_, this, _1));

  // Initialize publisher
  trajectory_publisher_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "mission_lane_converter/output/trajectory", qos_reliability);

  trajectory_publisher_global_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "mission_lane_converter/output/global_trajectory", qos_reliability);

  // Artificial publisher to test the trajectory generation, TODO(simon.eisenmann@driveblocks.ai):
  // Remove this publisher later, it is needed to get the vehicle started
  publisher_ = this->create_publisher<autoware_planning_msgs::msg::Trajectory>(
    "mission_lane_converter/output/global_trajectory", qos_reliability);

  // Path publisher
  path_publisher_ = create_publisher<autoware_planning_msgs::msg::Path>(
    "mission_lane_converter/output/path", qos_reliability);

  path_publisher_global_ = create_publisher<autoware_planning_msgs::msg::Path>(
    "mission_lane_converter/output/global_path", qos_reliability);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionLaneConverterNode::TimedStartupTrajectoryCallback, this));

  vis_trajectory_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "mission_lane_converter/output/vis_trajectory", qos_best_effort);

  vis_trajectory_publisher_global_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "mission_lane_converter/output/vis_global_trajectory", qos_best_effort);

  vis_path_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mission_lane_converter/output/vis_path", qos_best_effort);

  vis_odometry_publisher_global_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "mission_lane_converter/output/vis_global_odometry", qos_best_effort);

  // ros parameters (will be overwritten by external param file if exists)
  target_speed_ = declare_parameter<float>("target_speed", 3.0);
  RCLCPP_INFO(this->get_logger(), "Target speed set to: %.2f", target_speed_);
}

void MissionLaneConverterNode::TimedStartupTrajectoryCallback()
{
  if (!mission_lanes_available_once_) {
    // Empty trajectory for controller
    autoware_planning_msgs::msg::Trajectory trj_msg = autoware_planning_msgs::msg::Trajectory();

    // Frame id
    trj_msg.header.frame_id = local_map_frame_;
    trj_msg.header.stamp = rclcpp::Node::now();

    for (int idx_point = 0; idx_point < 100; idx_point++) {
      const double x = -5.0 + idx_point * 1.0;
      const double y = 0.0;
      const double v_x = target_speed_;

      AddTrajectoryPoint_(trj_msg, x, y, v_x);
    }

    // Heading in trajectory path will be overwritten
    this->AddHeadingToTrajectory_(trj_msg);

    trj_msg = TransformToGlobalFrame(trj_msg);

    publisher_->publish(trj_msg);
  }
}

void MissionLaneConverterNode::MissionLanesCallback_(
  const autoware_planning_msgs::msg::MissionLanesStamped & msg_mission)
{
  // FIXME: workaround to get the vehicle driving in autonomous mode until the
  // environment model is available
  if (msg_mission.ego_lane.centerline.size() == 0) {
    // Do not continue to publish empty trajectory
    if (mission_lanes_available_once_) {
      // Do only print warning if full mission lane was already available once
      RCLCPP_WARN(this->get_logger(), "Received empty ego mission lane, aborting conversion!");
    }
    return;
  } else {
    mission_lanes_available_once_ = true;
  }

  std::tuple<
    autoware_planning_msgs::msg::Trajectory, visualization_msgs::msg::Marker,
    autoware_planning_msgs::msg::Path, visualization_msgs::msg::MarkerArray>
    mission_to_trj = ConvertMissionToTrajectory(msg_mission);

  autoware_planning_msgs::msg::Trajectory trj_msg = std::get<0>(mission_to_trj);
  visualization_msgs::msg::Marker trj_vis = std::get<1>(mission_to_trj);
  autoware_planning_msgs::msg::Path path_msg = std::get<2>(mission_to_trj);
  visualization_msgs::msg::MarkerArray path_area = std::get<3>(mission_to_trj);

  autoware_planning_msgs::msg::Trajectory trj_msg_global =
    TransformToGlobalFrame<autoware_planning_msgs::msg::Trajectory>(trj_msg);
  autoware_planning_msgs::msg::Path path_msg_global =
    TransformToGlobalFrame<autoware_planning_msgs::msg::Path>(path_msg);

  visualization_msgs::msg::Marker trj_vis_global = GetGlobalTrjVisualization_(trj_msg_global);
  vis_trajectory_publisher_global_->publish(trj_vis_global);

  // Publish trajectory to motion planner
  trajectory_publisher_->publish(trj_msg);
  trajectory_publisher_global_->publish(trj_msg_global);

  // Publish path to motion planner
  path_publisher_->publish(path_msg);
  path_publisher_global_->publish(path_msg_global);

  // TODO(thomas.herrmann@driveblocks.ai): outsource this to a separate method
  // Clear all markers in scene
  visualization_msgs::msg::Marker msg_marker;
  msg_marker.header = msg_mission.header;
  msg_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

  // This specifies the clear all / delete all action
  msg_marker.action = 3;
  vis_trajectory_publisher_->publish(msg_marker);

  visualization_msgs::msg::MarkerArray msg_marker_array;
  msg_marker_array.markers.push_back(msg_marker);
  vis_path_publisher_->publish(msg_marker_array);

  vis_trajectory_publisher_->publish(trj_vis);
  vis_path_publisher_->publish(path_area);

  return;
}

std::tuple<
  autoware_planning_msgs::msg::Trajectory, visualization_msgs::msg::Marker,
  autoware_planning_msgs::msg::Path, visualization_msgs::msg::MarkerArray>
MissionLaneConverterNode::ConvertMissionToTrajectory(
  const autoware_planning_msgs::msg::MissionLanesStamped & msg)
{
  // Empty trajectory for controller
  autoware_planning_msgs::msg::Trajectory trj_msg = autoware_planning_msgs::msg::Trajectory();

  autoware_planning_msgs::msg::Path path_msg = autoware_planning_msgs::msg::Path();

  // Empty trajectory visualization message
  visualization_msgs::msg::Marker trj_vis, path_center_vis, path_left_vis, path_right_vis;
  visualization_msgs::msg::MarkerArray path_area_vis;

  trj_vis.header.frame_id = msg.header.frame_id;
  trj_vis.header.stamp = msg.header.stamp;
  trj_vis.ns = "mission_trajectory";
  trj_vis.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trj_vis.pose.orientation.w = 1.0;  // Neutral orientation
  trj_vis.scale.x = 0.4;
  trj_vis.color.g = 0.742;  // Green color
  trj_vis.color.b = 0.703;  // Blue color
  trj_vis.color.a = 0.750;
  trj_vis.lifetime.sec = 0;      // Forever
  trj_vis.lifetime.nanosec = 0;  // Forever
  trj_vis.frame_locked = false;  // Always transform into baselink

  path_left_vis.header.frame_id = msg.header.frame_id;
  path_left_vis.header.stamp = msg.header.stamp;
  path_left_vis.ns = "mission_path";
  path_left_vis.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_left_vis.pose.orientation.w = 1.0;  // Neutral orientation
  path_left_vis.scale.x = 0.6;

  path_left_vis.color.g = 0.742;  // Green color
  path_left_vis.color.b = 0.703;  // Blue color
  path_left_vis.color.a = 0.350;
  path_left_vis.lifetime.sec = 0;      // Forever
  path_left_vis.lifetime.nanosec = 0;  // Forever
  path_left_vis.frame_locked = false;  // Always transform into baselink

  path_right_vis = path_left_vis;
  path_center_vis = path_left_vis;

  // Fill output trajectory header
  trj_msg.header = msg.header;
  path_msg.header = msg.header;
  trj_msg.header.frame_id = local_map_frame_;
  path_msg.header.frame_id = local_map_frame_;

  switch (msg.target_lane) {
    case 0:
      // If target == 0, forward ego lane
      CreateMotionPlannerInput_(
        trj_msg, path_msg, trj_vis, path_center_vis, msg.ego_lane.centerline);

      // Fill path bounds left and right
      CreatePathBound_(path_msg.left_bound, path_left_vis, msg.ego_lane.bound_left, 1);
      CreatePathBound_(path_msg.right_bound, path_right_vis, msg.ego_lane.bound_right, 2);
      break;
    case -1:
      // Lane change to the left
      CreateMotionPlannerInput_(
        trj_msg, path_msg, trj_vis, path_center_vis, msg.drivable_lanes_left[0].centerline);

      // Fill path bounds left and right
      CreatePathBound_(
        path_msg.left_bound, path_left_vis, msg.drivable_lanes_left[0].bound_left, 1);
      CreatePathBound_(path_msg.right_bound, path_right_vis, msg.ego_lane.bound_right, 2);
      break;
    case 1:
      // Lane change to the right
      CreateMotionPlannerInput_(
        trj_msg, path_msg, trj_vis, path_center_vis, msg.drivable_lanes_right[0].centerline);

      // Fill path bounds left and right
      CreatePathBound_(path_msg.left_bound, path_left_vis, msg.ego_lane.bound_left, 1);
      CreatePathBound_(
        path_msg.right_bound, path_right_vis, msg.drivable_lanes_right[0].bound_right, 2);
      break;
    case -2:
      // Take exit left
      CreateMotionPlannerInput_(
        trj_msg, path_msg, trj_vis, path_center_vis, msg.drivable_lanes_left.back().centerline);

      // Fill path bounds left and right
      CreatePathBound_(
        path_msg.left_bound, path_left_vis, msg.drivable_lanes_left.back().bound_left, 1);
      CreatePathBound_(path_msg.right_bound, path_right_vis, msg.ego_lane.bound_right, 2);
      break;
    case 2:
      // Take exit right
      CreateMotionPlannerInput_(
        trj_msg, path_msg, trj_vis, path_center_vis, msg.drivable_lanes_right.back().centerline);

      // Fill path bounds left and right
      CreatePathBound_(path_msg.left_bound, path_left_vis, msg.ego_lane.bound_left, 1);
      CreatePathBound_(
        path_msg.right_bound, path_right_vis, msg.drivable_lanes_right.back().bound_right, 2);
      break;

    default:
      RCLCPP_WARN(
        this->get_logger(), "Unknown target lane specified in input mission lanes! (%d)",
        msg.target_lane);
      break;
  }

  path_area_vis.markers.push_back(path_center_vis);
  path_area_vis.markers.push_back(path_left_vis);
  path_area_vis.markers.push_back(path_right_vis);

  this->AddHeadingToTrajectory_(trj_msg);

  return std::make_tuple(trj_msg, trj_vis, path_msg, path_area_vis);
}

void MissionLaneConverterNode::CreateMotionPlannerInput_(
  autoware_planning_msgs::msg::Trajectory & trj_msg, autoware_planning_msgs::msg::Path & path_msg,
  visualization_msgs::msg::Marker & trj_vis, visualization_msgs::msg::Marker & path_vis,
  const std::vector<geometry_msgs::msg::Point> & centerline_mission_lane)
{
  // Add a mission lane's centerline to the output trajectory and path messages
  for (size_t idx_point = 0; idx_point < centerline_mission_lane.size(); idx_point++) {
    // Check if trajectory message is empty
    if (trj_msg.points.size() > 0) {
      // Check if last trajectory point equals the one we want to add now
      if (
        trj_msg.points.back().pose.position.x != centerline_mission_lane[idx_point].x &&
        trj_msg.points.back().pose.position.y != centerline_mission_lane[idx_point].y) {
        AddTrajectoryPoint_(
          trj_msg, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y,
          target_speed_);

        // Similar point has to be added to the path message
        AddPathPoint_(
          path_msg, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y,
          target_speed_);

        // Add visualization marker to trajectory vis message
        AddPointVisualizationMarker_(
          trj_vis, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y, 0);

        // Add visualization marker to path vis message
        AddPointVisualizationMarker_(
          path_vis, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y, 0);
      }
    } else {  // Add first point
      // Add a trajectory point
      AddTrajectoryPoint_(
        trj_msg, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y,
        target_speed_);

      // Similar point has to be added to the path message
      AddPathPoint_(
        path_msg, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y,
        target_speed_);

      AddPointVisualizationMarker_(
        trj_vis, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y, 0);

      // Add visualization marker to path vis message
      AddPointVisualizationMarker_(
        path_vis, centerline_mission_lane[idx_point].x, centerline_mission_lane[idx_point].y, 0);
    }
  }

  return;
}

void MissionLaneConverterNode::CreatePathBound_(
  std::vector<geometry_msgs::msg::Point> & bound_path, visualization_msgs::msg::Marker & path_vis,
  const std::vector<geometry_msgs::msg::Point> & bound_mission_lane, const int id_marker)
{
  for (size_t idx_point = 0; idx_point < bound_mission_lane.size(); idx_point++) {
    geometry_msgs::msg::Point pt_path;
    pt_path.x = bound_mission_lane[idx_point].x;
    pt_path.y = bound_mission_lane[idx_point].y;
    // Check if path bound is empty
    if (bound_path.size() > 0) {
      // Check if last path point equals the one we want to add now
      if (
        bound_path.back().x != bound_mission_lane[idx_point].x &&
        bound_path.back().y != bound_mission_lane[idx_point].y) {
        // Finally add the path point after successful checks
        bound_path.push_back(pt_path);
      }
    } else {  // Add first point to path
      bound_path.push_back(pt_path);
    }

    // Add last added point to a marker message for debugging
    // FIXME: probably no unique ids for multiple calls?
    AddPointVisualizationMarker_(path_vis, bound_path.back().x, bound_path.back().y, id_marker);
  }

  return;
}

void MissionLaneConverterNode::AddPathPoint_(
  autoware_planning_msgs::msg::Path & pth_msg, const double x, const double y, const double v_x)
{
  // Add a trajectory point
  pth_msg.points.push_back(autoware_planning_msgs::msg::PathPoint());

  // Fill trajectory points with meaningful data
  pth_msg.points.back().pose.position.x = x;
  pth_msg.points.back().pose.position.y = y;

  // Constant velocity
  pth_msg.points.back().longitudinal_velocity_mps = v_x;

  return;
}

void MissionLaneConverterNode::AddTrajectoryPoint_(
  autoware_planning_msgs::msg::Trajectory & trj_msg, const double x, const double y,
  const double v_x)
{
  // Add a trajectory point
  trj_msg.points.push_back(autoware_planning_msgs::msg::TrajectoryPoint());

  // Fill trajectory points with meaningful data
  trj_msg.points.back().pose.position.x = x;
  trj_msg.points.back().pose.position.y = y;

  // Constant velocity
  trj_msg.points.back().longitudinal_velocity_mps = v_x;

  return;
}

void MissionLaneConverterNode::AddPointVisualizationMarker_(
  visualization_msgs::msg::Marker & trj_vis, const double x, const double y, const int id_marker)
{
  // Fill visualization message
  trj_vis.points.push_back(geometry_msgs::msg::Point());

  trj_vis.points.back().x = x;
  trj_vis.points.back().y = y;
  trj_vis.id = id_marker;

  return;
}

void MissionLaneConverterNode::AddHeadingToTrajectory_(
  autoware_planning_msgs::msg::Trajectory & trj_msg)
{
  std::vector<geometry_msgs::msg::Point> points;

  for (size_t idx_point = 0; idx_point < trj_msg.points.size(); idx_point++) {
    points.push_back(geometry_msgs::msg::Point());
    points.back().x = trj_msg.points[idx_point].pose.position.x;
    points.back().y = trj_msg.points[idx_point].pose.position.y;
  }

  // Only execute if we have at least 2 points
  if (points.size() > 1) {
    std::vector<double> psi_vec = GetPsiForPoints(points);

    tf2::Quaternion tf2_quat;
    for (size_t idx_point = 0; idx_point < trj_msg.points.size(); idx_point++) {
      tf2_quat.setRPY(0.0, 0.0, psi_vec[idx_point]);

      trj_msg.points.back().pose.orientation.x = tf2_quat.getX();
      trj_msg.points.back().pose.orientation.y = tf2_quat.getY();
      trj_msg.points.back().pose.orientation.z = tf2_quat.getZ();
      trj_msg.points.back().pose.orientation.w = tf2_quat.getW();
    }
  }

  return;
}

// TODO(thomas.herrmann@driveblocks.ai): store the latest odometry message here and then re-use in
// the output conversion
void MissionLaneConverterNode::CallbackOdometryMessages_(const nav_msgs::msg::Odometry & msg)
{
  // Store current odometry information
  last_odom_msg_ = msg;

  if (!received_motion_update_once_) {
    initial_odom_msg_ = msg;
  }

  received_motion_update_once_ = true;

  visualization_msgs::msg::Marker odom_vis;
  odom_vis.header.frame_id = msg.header.frame_id;
  odom_vis.header.stamp = msg.header.stamp;
  odom_vis.ns = "odometry";
  odom_vis.type = visualization_msgs::msg::Marker::POINTS;
  odom_vis.pose.orientation.w = 1.0;  // Neutral orientation
  odom_vis.scale.x = 5.0;
  odom_vis.color.g = 0.0;  // Green color
  odom_vis.color.b = 0.7;  // Blue color
  odom_vis.color.a = 0.7;
  odom_vis.lifetime.sec = 0;      // Forever
  odom_vis.lifetime.nanosec = 0;  // Forever
  odom_vis.frame_locked = false;  // Always transform into baselink

  odom_vis.points.push_back(geometry_msgs::msg::Point());

  odom_vis.points.back().x = msg.pose.pose.position.x;
  odom_vis.points.back().y = msg.pose.pose.position.y;
  odom_vis.id = 100;

  vis_odometry_publisher_global_->publish(odom_vis);

  return;
}

template <typename T>
T MissionLaneConverterNode::TransformToGlobalFrame(const T & msg_input)
{
  // Define output message
  T msg_output = msg_input;
  msg_output.header.frame_id = local_map_frame_;

  // Construct raw odometry pose
  geometry_msgs::msg::PoseStamped odometry_pose_raw;
  odometry_pose_raw.header = last_odom_msg_.header;
  odometry_pose_raw.pose = last_odom_msg_.pose.pose;

  if (received_motion_update_once_) {
    // If the incoming odometry signal is properly filled, i.e. if the frame ids
    // are given and report an odometry signal, do nothing, else we assume the
    // odometry signal stems from the GNSS (and is therefore valid in the odom
    // frame)
    if (!(last_odom_msg_.header.frame_id == local_map_frame_ &&
          last_odom_msg_.child_frame_id == "base_link")) {
      if (!b_input_odom_frame_error_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Your odometry signal doesn't match the expectation to be a "
          "transformation from frame <%s> to <base_link>! The node will continue spinning but the "
          "odometry signal should be checked! This error is printed only "
          "once.",
          local_map_frame_.c_str());
        b_input_odom_frame_error_ = true;
      }
    }

    const double psi_initial = GetYawFromQuaternion(
      initial_odom_msg_.pose.pose.orientation.x, initial_odom_msg_.pose.pose.orientation.y,
      initial_odom_msg_.pose.pose.orientation.z, initial_odom_msg_.pose.pose.orientation.w);
    const Pose2D pose_initial(
      initial_odom_msg_.pose.pose.position.x, initial_odom_msg_.pose.pose.position.y, psi_initial);

    const double psi_cur = GetYawFromQuaternion(
      last_odom_msg_.pose.pose.orientation.x, last_odom_msg_.pose.pose.orientation.y,
      last_odom_msg_.pose.pose.orientation.z, last_odom_msg_.pose.pose.orientation.w);
    const Pose2D pose_cur(
      last_odom_msg_.pose.pose.position.x, last_odom_msg_.pose.pose.position.y, psi_cur);

    // Get relationship from current odom frame to global map frame origin
    const Pose2D d_current_to_map_origin = TransformToNewCosy2D(pose_cur, Pose2D{0.0, 0.0});

    // Convert all the input points to the global map frame
    for (size_t i = 0; i < msg_input.points.size(); i++) {
      // convert input pose
      const double psi_point = GetYawFromQuaternion(
        msg_input.points[i].pose.orientation.x, msg_input.points[i].pose.orientation.y,
        msg_input.points[i].pose.orientation.z, msg_input.points[i].pose.orientation.w);
      const Pose2D pose(
        msg_input.points[i].pose.position.x, msg_input.points[i].pose.position.y, psi_point);

      // Express point in global map frame
      Pose2D pose_map = TransformToNewCosy2D(d_current_to_map_origin, pose);

      msg_output.points[i].pose.position.x = pose_map.get_x();
      msg_output.points[i].pose.position.y = pose_map.get_y();
    }

    // Convert the path area's bounds
    if constexpr (std::is_same<T, autoware_planning_msgs::msg::Path>::value) {
      for (size_t ib = 0; ib < 2; ib++) {
        std::vector<geometry_msgs::msg::Point> bound;
        if (ib == 0)
          bound = msg_input.left_bound;
        else
          bound = msg_input.right_bound;
        for (size_t i = 0; i < bound.size(); i++) {
          // Convert input pose
          const Pose2D pose(bound[i].x, bound[i].y);

          // Express point in global map frame
          Pose2D pose_map = TransformToNewCosy2D(d_current_to_map_origin, pose);

          if (ib == 0) {
            msg_output.left_bound[i].x = pose_map.get_x();
            msg_output.left_bound[i].y = pose_map.get_y();
          } else {
            msg_output.right_bound[i].x = pose_map.get_x();
            msg_output.right_bound[i].y = pose_map.get_y();
          }
        }
      }
    }

    // Add heading if type is a trajectory
    if constexpr (std::is_same<T, autoware_planning_msgs::msg::Trajectory>::value)
      AddHeadingToTrajectory_(msg_output);
  }

  return msg_output;
}

visualization_msgs::msg::Marker MissionLaneConverterNode::GetGlobalTrjVisualization_(
  const autoware_planning_msgs::msg::Trajectory & trj_msg)
{
  // Empty trajectory visualization message
  visualization_msgs::msg::Marker trj_vis_global;
  trj_vis_global.header.frame_id = trj_msg.header.frame_id;
  trj_vis_global.header.stamp = trj_msg.header.stamp;
  trj_vis_global.ns = "mission_trajectory_global";
  trj_vis_global.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trj_vis_global.pose.orientation.w = 1.0;  // Neutral orientation
  trj_vis_global.scale.x = 0.4;
  trj_vis_global.color.g = 0.0;    // Green color
  trj_vis_global.color.b = 0.703;  // Blue color
  trj_vis_global.color.a = 0.0;
  trj_vis_global.lifetime.sec = 0;      // Forever
  trj_vis_global.lifetime.nanosec = 0;  // Forever
  trj_vis_global.frame_locked = false;  // Always transform into baselink

  for (size_t i = 0; i < trj_msg.points.size(); i++) {
    AddPointVisualizationMarker_(
      trj_vis_global, trj_msg.points[i].pose.position.x, trj_msg.points[i].pose.position.y, 10);
  }

  return trj_vis_global;
}
}  // namespace autoware::mapless_architecture

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mapless_architecture::MissionLaneConverterNode)
