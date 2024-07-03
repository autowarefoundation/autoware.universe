// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "autoware/local_mission_planner/mission_planner_node.hpp"

#include "autoware/local_mission_planner_common/helper_functions.hpp"
#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_core/geometry/Lanelet.h"
#include "lanelet2_core/geometry/LineString.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/driving_corridor.hpp"
#include "autoware_planning_msgs/msg/mission.hpp"
#include "autoware_planning_msgs/msg/mission_lanes_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <chrono>
#include <limits>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace autoware::mapless_architecture
{
using std::placeholders::_1;

MissionPlannerNode::MissionPlannerNode() : Node("mission_planner_node")
{
  // Set quality of service to best effort (if transmission fails, do not try to
  // resend but rather use new sensor data)
  // the history_depth is set to 1 (message queue size)
  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  // Initialize publisher for visualization markers
  visualizationPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "mission_planner_node/output/marker", 1);

  // Initialize publisher to visualize the centerline of a lane
  visualization_publisher_centerline_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "mission_planner_node/output/centerline", 1);

  // Initialize publisher for visualization of the distance
  visualizationDistancePublisher_ =
    this->create_publisher<autoware_planning_msgs::msg::VisualizationDistance>(
      "mission_planner_node/output/visualization_distance", 1);

  // Initialize publisher for goal point marker
  visualizationGoalPointPublisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "mission_planner_node/output/marker_goal_point", 1);

  // Initialize publisher for mission lanes
  missionLanesStampedPublisher_ =
    this->create_publisher<autoware_planning_msgs::msg::MissionLanesStamped>(
      "mission_planner_node/output/mission_lanes_stamped", 1);

  // Initialize subscriber to lanelets stamped messages
  mapSubscriber_ = this->create_subscription<autoware_planning_msgs::msg::LocalMap>(
    "mission_planner_node/input/local_map", qos,
    std::bind(&MissionPlannerNode::CallbackLocalMapMessages_, this, _1));

  // Initialize subscriber to mission messages
  missionSubscriber_ = this->create_subscription<autoware_planning_msgs::msg::Mission>(
    "mission_planner/input/mission", qos,
    std::bind(&MissionPlannerNode::CallbackMissionMessages_, this, _1));

  // Initialize subscriber to odometry messages
  OdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "mission_planner/input/state_estimate", qos,
    std::bind(&MissionPlannerNode::CallbackOdometryMessages_, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Set ros parameters (DEFAULT values will be overwritten by external
  // parameter file if exists)
  distance_to_centerline_threshold_ =
    declare_parameter<float>("distance_to_centerline_threshold", 0.2);
  RCLCPP_INFO(
    this->get_logger(), "Threshold distance to centerline for successful lane change: %.2f",
    distance_to_centerline_threshold_);

  projection_distance_on_goallane_ =
    declare_parameter<float>("projection_distance_on_goallane", 30.0);
  RCLCPP_INFO(
    this->get_logger(), "Projection distance for goal point in mission: %.1f",
    projection_distance_on_goallane_);

  retrigger_attempts_max_ = declare_parameter<int>("retrigger_attempts_max", 10);
  RCLCPP_INFO(
    this->get_logger(), "Number of attempts for triggering a lane change: %d",
    retrigger_attempts_max_);
}

void MissionPlannerNode::CallbackLocalMapMessages_(
  const autoware_planning_msgs::msg::LocalMap & msg)
{
  // Used for output
  std::vector<LaneletConnection> lanelet_connections;
  std::vector<lanelet::Lanelet> converted_lanelets;

  ConvertInput2LaneletFormat(msg.road_segments, converted_lanelets, lanelet_connections);
  VisualizeLanes(msg.road_segments, converted_lanelets);

  // Get the lanes
  Lanes result = MissionPlannerNode::CalculateLanes_(converted_lanelets, lanelet_connections);

  // Get the ego lane
  ego_lane_ = result.ego;

  std::vector<std::vector<int>> left_lanes = result.left;
  if (!left_lanes.empty()) {
    lane_left_ = left_lanes[0];  // Store the first left lane (needed for lane change)
  }

  std::vector<std::vector<int>> right_lanes = result.right;
  if (!right_lanes.empty()) {
    lane_right_ = right_lanes[0];  // Store the first right lane (needed for lane change)
  }

  // Save current converted_lanelets
  current_lanelets_ = converted_lanelets;

  // Re-trigger lane change if necessary
  if (lane_change_trigger_success_ == false && retry_attempts_ <= retrigger_attempts_max_) {
    if (lane_change_direction_ == left) {
      // Lane change to the left
      InitiateLaneChange_(left, lane_left_);
    } else if (lane_change_direction_ == right) {
      // Lane change to the right
      InitiateLaneChange_(right, lane_right_);
    }
  }

  if (retry_attempts_ > retrigger_attempts_max_) {
    // Lane change has failed, must be re-triggered manually
    RCLCPP_WARN(
      this->get_logger(), "Lane change failed! Number of attempts: (%d/%d)", retry_attempts_,
      retrigger_attempts_max_);

    // Reset variable
    lane_change_trigger_success_ = true;
    retry_attempts_ = 0;
  }

  // Check if goal point should be reset, if yes -> reset goal point
  CheckIfGoalPointShouldBeReset_(converted_lanelets, lanelet_connections);

  // Check if lane change was successful, if yes -> reset mission
  if (mission_ != stay) {
    int egoLaneletIndex = FindEgoOccupiedLaneletID(converted_lanelets);  // Returns -1 if no match
    lanelet::BasicPoint2d pointEgo(0,
                                   0);  // Vehicle is always located at (0, 0)

    if (egoLaneletIndex >= 0) {
      // Check if successful lane change
      if (
        IsOnGoalLane_(egoLaneletIndex, goal_point_, converted_lanelets, lanelet_connections) &&
        CalculateDistanceBetweenPointAndLineString(
          converted_lanelets[egoLaneletIndex].centerline2d(), pointEgo) <=
          distance_to_centerline_threshold_) {
        // Reset mission to lane keeping
        mission_ = stay;
        target_lane_ = stay;
      }

      // Check if we are on the target lane, if yes -> update target_lane
      if (IsOnGoalLane_(egoLaneletIndex, goal_point_, converted_lanelets, lanelet_connections)) {
        target_lane_ = stay;
      } else {
        target_lane_ = mission_;
      }
    }
  }

  // Create a MarkerArray for clearing old markers
  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  visualization_msgs::msg::MarkerArray clear_marker_array;
  clear_marker_array.markers.push_back(clear_marker);

  // Publish the clear marker array to delete old markers
  visualization_publisher_centerline_->publish(clear_marker_array);

  // Publish mission lanes
  autoware_planning_msgs::msg::MissionLanesStamped lanes;
  lanes.header.frame_id = msg.road_segments.header.frame_id;  // Same frame_id as msg
  lanes.header.stamp = rclcpp::Node::now();

  // Add target lane
  switch (target_lane_) {
    case stay:
      lanes.target_lane = 0;
      break;
    case left:
      lanes.target_lane = -1;
      break;
    case right:
      lanes.target_lane = +1;
      break;
    case left_most:
      lanes.target_lane = -2;
      break;
    case right_most:
      lanes.target_lane = +2;
      break;

    default:
      break;
  }

  lanes.deadline_target_lane =
    std::numeric_limits<double>::infinity();  // TODO(simon.eisenmann@driveblocks.ai):
                                              // Change this value

  // Create driving corridors and add them to the MissionLanesStamped message
  lanes.ego_lane = CreateDrivingCorridor(ego_lane_, converted_lanelets);
  VisualizeCenterlineOfDrivingCorridor(msg.road_segments, lanes.ego_lane);

  // Initialize driving corridor
  autoware_planning_msgs::msg::DrivingCorridor driving_corridor;

  if (!left_lanes.empty()) {
    for (const std::vector<int> & lane : left_lanes) {
      driving_corridor = CreateDrivingCorridor(lane, converted_lanelets);
      lanes.drivable_lanes_left.push_back(driving_corridor);
      VisualizeCenterlineOfDrivingCorridor(msg.road_segments, driving_corridor);
    }
  }

  if (!right_lanes.empty()) {
    for (const std::vector<int> & lane : right_lanes) {
      driving_corridor = CreateDrivingCorridor(lane, converted_lanelets);
      lanes.drivable_lanes_right.push_back(driving_corridor);
      VisualizeCenterlineOfDrivingCorridor(msg.road_segments, driving_corridor);
    }
  }

  // Publish MissionLanesStamped message
  missionLanesStampedPublisher_->publish(lanes);
}

void MissionPlannerNode::CallbackOdometryMessages_(const nav_msgs::msg::Odometry & msg)
{
  // NOTE: We assume that the odometry message is the GNSS signal

  // Construct raw odometry pose
  geometry_msgs::msg::PoseStamped odometry_pose_raw, pose_base_link_in_odom_frame,
    pose_base_link_in_map_frame;
  odometry_pose_raw.header = msg.header;
  odometry_pose_raw.pose = msg.pose.pose;

  // If the incoming odometry signal is properly filled, i.e. if the frame ids
  // are given and report an odometry signal , do nothing, else we assume the
  // odometry signal stems from the GNSS (and is therefore valid in the odom
  // frame)
  if (msg.header.frame_id == "map" && msg.child_frame_id == "base_link") {
    pose_base_link_in_map_frame = odometry_pose_raw;
  } else {
    if (!b_global_odometry_deprecation_warning_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Your odometry signal doesn't match the expectation to be a "
        "transformation from frame <map> to <base_link>! We assume the "
        "input "
        "signal to be a GNSS raw signal being a transform from <map> to "
        "<odom>. The support for this feature will be deprecated in a "
        "future "
        "release, please check you odometry signal or use a driveblocks "
        "local odometry signal instead! This warning is printed only "
        "once.");
      b_global_odometry_deprecation_warning_ = true;
    }
    // Prepare map to odom transform
    // TODO(thomas.herrmann@driveblocks.ai): Can be removed when the state
    // estimator publishes this information in the correct frames
    geometry_msgs::msg::TransformStamped trafo_map2odom;
    trafo_map2odom.header.stamp = msg.header.stamp;
    trafo_map2odom.header.frame_id = "map";
    trafo_map2odom.child_frame_id = "odom";
    trafo_map2odom.transform.translation.x = msg.pose.pose.position.x;
    trafo_map2odom.transform.translation.y = msg.pose.pose.position.y;
    trafo_map2odom.transform.translation.z = msg.pose.pose.position.z;
    trafo_map2odom.transform.rotation.x = msg.pose.pose.orientation.x;
    trafo_map2odom.transform.rotation.y = msg.pose.pose.orientation.y;
    trafo_map2odom.transform.rotation.z = msg.pose.pose.orientation.z;
    trafo_map2odom.transform.rotation.w = msg.pose.pose.orientation.w;

    geometry_msgs::msg::TransformStamped trafo_base_link_in_odom_frame;
    try {
      // constant trafo from gnss receiver to base_link
      trafo_base_link_in_odom_frame =
        tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Transform not yet available from <odom> to <base_link>");
    }

    // Extract the trafo from the odom frame to the base_link frame
    pose_base_link_in_odom_frame.header.frame_id = "odom";
    pose_base_link_in_odom_frame.pose.position.x =
      trafo_base_link_in_odom_frame.transform.translation.x;
    pose_base_link_in_odom_frame.pose.position.y =
      trafo_base_link_in_odom_frame.transform.translation.y;
    pose_base_link_in_odom_frame.pose.position.z =
      trafo_base_link_in_odom_frame.transform.translation.z;
    pose_base_link_in_odom_frame.pose.orientation.x =
      trafo_base_link_in_odom_frame.transform.rotation.x;
    pose_base_link_in_odom_frame.pose.orientation.y =
      trafo_base_link_in_odom_frame.transform.rotation.y;
    pose_base_link_in_odom_frame.pose.orientation.z =
      trafo_base_link_in_odom_frame.transform.rotation.z;
    pose_base_link_in_odom_frame.pose.orientation.w =
      trafo_base_link_in_odom_frame.transform.rotation.w;

    // Transform base_link origin from odom frame to map frame
    tf2::doTransform(pose_base_link_in_odom_frame, pose_base_link_in_map_frame, trafo_map2odom);
  }

  // Calculate yaw for received pose
  double psi_cur_corrected = GetYawFromQuaternion(
    pose_base_link_in_map_frame.pose.orientation.x, pose_base_link_in_map_frame.pose.orientation.y,
    pose_base_link_in_map_frame.pose.orientation.z, pose_base_link_in_map_frame.pose.orientation.w);

  RCLCPP_DEBUG(
    this->get_logger(),
    "GNSS pose raw: x,y,z | quaternion: %.3f, %.3f, %.3f | %.3f, %.3f, "
    "%.3f, "
    "%.3f\t "
    "GNSS pose in base_link: x,y,z | quaternion: %.3f, %.3f, %.3f | %.3f, "
    "%.3f, "
    "%.3f, %.3f",
    odometry_pose_raw.pose.position.x, odometry_pose_raw.pose.position.y,
    odometry_pose_raw.pose.position.z, odometry_pose_raw.pose.orientation.x,
    odometry_pose_raw.pose.orientation.y, odometry_pose_raw.pose.orientation.z,
    odometry_pose_raw.pose.orientation.w, pose_base_link_in_map_frame.pose.position.x,
    pose_base_link_in_map_frame.pose.position.y, pose_base_link_in_map_frame.pose.position.z,
    pose_base_link_in_map_frame.pose.orientation.x, pose_base_link_in_map_frame.pose.orientation.y,
    pose_base_link_in_map_frame.pose.orientation.z, pose_base_link_in_map_frame.pose.orientation.w);

  RCLCPP_DEBUG(
    this->get_logger(), "Received pose (x: %.2f, y: %.2f, psi %.2f)",
    pose_base_link_in_map_frame.pose.position.x, pose_base_link_in_map_frame.pose.position.y,
    psi_cur_corrected);

  if (pose_prev_init_) {
    // Calculate and forward relative motion update to driving corridor model
    const Pose2D pose_cur(
      pose_base_link_in_map_frame.pose.position.x, pose_base_link_in_map_frame.pose.position.y,
      psi_cur_corrected);
    const Pose2D d_pose = TransformToNewCosy2D(pose_prev_, pose_cur);

    // Transform the target pose into the new cosy which is given in relation
    // to the previous origin
    Pose2D target_pose = {goal_point_.x(), goal_point_.y()};
    target_pose = TransformToNewCosy2D(d_pose, target_pose);

    // Overwrite goal point
    goal_point_.x() = target_pose.get_x();
    goal_point_.y() = target_pose.get_y();

    // Re-center updated goal point to lie on centerline (to get rid of issues
    // with a less accurate odometry update which could lead to loosing the
    // goal lane)
    // TODO(thomas.herrmann@driveblocks.ai): Reduction of calling
    // frequency of this method (since not needed at a high frequency and
    // probably computationally expensive)
    const lanelet::BasicPoint2d target_point_2d = RecenterGoalPoint(goal_point_, current_lanelets_);

    // Overwrite goal point
    goal_point_.x() = target_point_2d.x();
    goal_point_.y() = target_point_2d.y();

    // --- Start of debug visualization
    // Create marker and publish it
    visualization_msgs::msg::Marker goal_marker;  // Create a new marker

    goal_marker.header.frame_id = msg.header.frame_id;
    goal_marker.header.stamp = msg.header.stamp;
    goal_marker.ns = "goal_point";
    goal_marker.type = visualization_msgs::msg::Marker::POINTS;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.orientation.w = 1.0;  // Neutral orientation
    goal_marker.scale.x = 6.0;
    goal_marker.color.r = 1.0;  // Red color
    goal_marker.color.a = 1.0;  // Full opacity

    // Add goal point to the marker
    geometry_msgs::msg::Point p_marker;

    p_marker.x = goal_point_.x();
    p_marker.y = goal_point_.y();

    goal_marker.points.push_back(p_marker);

    // Clear all markers in scene
    visualization_msgs::msg::Marker msg_marker;
    msg_marker.header = msg.header;
    msg_marker.type = visualization_msgs::msg::Marker::POINTS;
    // This specifies the clear all / delete all action
    msg_marker.action = 3;
    visualizationGoalPointPublisher_->publish(msg_marker);

    visualizationGoalPointPublisher_->publish(goal_marker);
    // --- End of debug visualization

    last_odom_msg_ = msg;
  } else {
    pose_prev_init_ = true;
    last_odom_msg_ = msg;
  }

  // Update pose storage for next iteration
  pose_prev_.set_x(pose_base_link_in_map_frame.pose.position.x);
  pose_prev_.set_y(pose_base_link_in_map_frame.pose.position.y);
  pose_prev_.set_psi(psi_cur_corrected);

  received_motion_update_once_ = true;

  return;
}

void MissionPlannerNode::CallbackMissionMessages_(const autoware_planning_msgs::msg::Mission & msg)
{
  // Initialize variables
  lane_change_trigger_success_ = false;
  retry_attempts_ = 0;

  switch (msg.mission_type) {
    case autoware_planning_msgs::msg::Mission::LANE_KEEP:
      // Keep the lane
      mission_ = stay;
      target_lane_ = stay;
      break;
    case autoware_planning_msgs::msg::Mission::LANE_CHANGE_LEFT:
      // Initiate left lane change
      RCLCPP_INFO(this->get_logger(), "Lane change to the left initiated.");
      lane_change_direction_ = left;
      InitiateLaneChange_(lane_change_direction_, lane_left_);
      break;
    case autoware_planning_msgs::msg::Mission::LANE_CHANGE_RIGHT:
      // Initiate right lane change
      RCLCPP_INFO(this->get_logger(), "Lane change to the right initiated.");
      lane_change_direction_ = right;
      InitiateLaneChange_(lane_change_direction_, lane_right_);
      break;
    case autoware_planning_msgs::msg::Mission::TAKE_NEXT_EXIT_LEFT:
      // Initiate take next exit
      RCLCPP_INFO(this->get_logger(), "Take next exit (left) initiated.");
      target_lane_ = left_most;  // Set target lane
      break;
    case autoware_planning_msgs::msg::Mission::TAKE_NEXT_EXIT_RIGHT:
      // Initiate take next exit
      RCLCPP_INFO(this->get_logger(), "Take next exit (right) initiated.");
      target_lane_ = right_most;  // Set target lane
      break;
    default:
      // Nothing happens if mission does not match!
      RCLCPP_INFO(this->get_logger(), "Mission does not match.");
  }

  return;
}

void MissionPlannerNode::InitiateLaneChange_(
  const Direction direction, const std::vector<int> & neighboring_lane)
{
  retry_attempts_++;  // Increment retry attempts counter
  if (neighboring_lane.size() == 0) {
    // Neighbor lane is empty
    RCLCPP_WARN(this->get_logger(), "Empty neighbor lane!");
  } else {
    // Neighbor lane is available, initiate lane change
    RCLCPP_WARN(this->get_logger(), "Lane change successfully triggered!");
    lane_change_trigger_success_ = true;
    mission_ = direction;
    target_lane_ = direction;
    goal_point_ =
      GetPointOnLane(neighboring_lane, projection_distance_on_goallane_, current_lanelets_);
  }
}

// Determine the lanes
Lanes MissionPlannerNode::CalculateLanes_(
  const std::vector<lanelet::Lanelet> & converted_lanelets,
  std::vector<LaneletConnection> & lanelet_connections)
{
  // Calculate centerlines, left and right bounds
  std::vector<lanelet::ConstLineString3d> centerlines;
  std::vector<lanelet::ConstLineString3d> left_bounds;
  std::vector<lanelet::ConstLineString3d> right_bounds;

  int ego_lanelet_index =
    FindEgoOccupiedLaneletID(converted_lanelets);  // Finds the ID of the ego vehicle occupied
                                                   // lanelet (returns -1 if no match)

  // Initialize variables
  std::vector<std::vector<int>> ego_lane;
  std::vector<int> ego_lane_stripped_idx = {};
  std::vector<std::vector<int>> left_lanes = {};
  std::vector<std::vector<int>> right_lanes = {};

  if (ego_lanelet_index >= 0) {
    // Get ego lane
    ego_lane = GetAllSuccessorSequences(lanelet_connections, ego_lanelet_index);

    // Extract the first available ego lane
    if (ego_lane.size() > 0) {
      ego_lane_stripped_idx = ego_lane[0];

      // Get all neighbor lanelets to the ego lanelet on the left side
      std::vector<int> left_neighbors =
        GetAllNeighboringLaneletIDs(lanelet_connections, ego_lanelet_index, VehicleSide::kLeft);

      // Initialize current_lane and next_lane
      std::vector<int> current_lane = ego_lane_stripped_idx;
      std::vector<int> neighbor_lane;

      for (size_t i = 0; i < left_neighbors.size(); ++i) {
        neighbor_lane =
          GetAllNeighborsOfLane(current_lane, lanelet_connections, VehicleSide::kLeft);

        left_lanes.push_back(neighbor_lane);

        current_lane = neighbor_lane;
      }

      // Get all neighbor lanelets to the ego lanelet on the right side
      std::vector<int> right_neighbors =
        GetAllNeighboringLaneletIDs(lanelet_connections, ego_lanelet_index, VehicleSide::kRight);

      // Reinitialize current_lane
      current_lane = ego_lane_stripped_idx;

      for (size_t i = 0; i < right_neighbors.size(); ++i) {
        neighbor_lane =
          GetAllNeighborsOfLane(current_lane, lanelet_connections, VehicleSide::kRight);

        right_lanes.push_back(neighbor_lane);

        current_lane = neighbor_lane;
      }
    }
  }

  // Add one predecessor lanelet to the ego lane
  InsertPredecessorLanelet(ego_lane_stripped_idx, lanelet_connections);

  // Add one predecessor lanelet to each of the left lanes
  for (std::vector<int> & lane : left_lanes) {
    InsertPredecessorLanelet(lane, lanelet_connections);
  }

  // Add one predecessor lanelet to each of the right lanes
  for (std::vector<int> & lane : right_lanes) {
    InsertPredecessorLanelet(lane, lanelet_connections);
  }

  // Return lanes
  Lanes lanes;
  lanes.ego = ego_lane_stripped_idx;
  lanes.left = left_lanes;
  lanes.right = right_lanes;

  return lanes;
}

bool MissionPlannerNode::IsOnGoalLane_(
  const int ego_lanelet_index, const lanelet::BasicPoint2d & goal_point,
  const std::vector<lanelet::Lanelet> & converted_lanelets,
  const std::vector<LaneletConnection> & lanelet_connections)
{
  bool result = false;

  // Find the index of the lanelet containing the goal point
  int goal_index = FindOccupiedLaneletID(converted_lanelets, goal_point);  // Returns -1 if no match

  if (goal_index >= 0) {  // Check if -1
    std::vector<std::vector<int>> goal_lane = GetAllPredecessorSequences(
      lanelet_connections,
      goal_index);  // Get goal lane

    // Check if vehicle is on goal lane
    if (ego_lanelet_index == goal_index) {
      result = true;
    }

    for (const auto & innerVec : goal_lane) {
      for (int value : innerVec) {
        if (value == ego_lanelet_index) {
          result = true;
        }
      }
    }
  }
  return result;
}

void MissionPlannerNode::CheckIfGoalPointShouldBeReset_(
  const lanelet::Lanelets & converted_lanelets,
  const std::vector<LaneletConnection> & lanelet_connections)
{
  // Check if goal point should be reset: If the x value of the goal point is
  // negative, then the point is behind the vehicle and must be therefore reset.
  if (goal_point_.x() < 0 && mission_ != stay) {  // TODO(simon.eisenmann@driveblocks.ai): Maybe
                                                  // remove condition mission_ != stay
    // Find the index of the lanelet containing the goal point
    int goal_index =
      FindOccupiedLaneletID(converted_lanelets, goal_point_);  // Returns -1 if no match

    if (goal_index >= 0) {  // Check if -1
      // Reset goal point
      goal_point_ = GetPointOnLane(
        GetAllSuccessorSequences(lanelet_connections, goal_index)[0],
        projection_distance_on_goallane_, converted_lanelets);
    } else {
      // Reset of goal point not successful -> reset mission and target lane
      RCLCPP_WARN(this->get_logger(), "Lanelet of goal point cannot be determined, mission reset!");

      target_lane_ = 0;
      mission_ = 0;
    }
  }
}

// Getter for goal_point_
lanelet::BasicPoint2d MissionPlannerNode::goal_point()
{
  return goal_point_;
}

// Setter for goal_point_
void MissionPlannerNode::goal_point(const lanelet::BasicPoint2d & goal_point)
{
  goal_point_ = goal_point;
}

void MissionPlannerNode::ConvertInput2LaneletFormat(
  const autoware_planning_msgs::msg::RoadSegments & msg,
  std::vector<lanelet::Lanelet> & out_lanelets,
  std::vector<LaneletConnection> & out_lanelet_connections)
{
  // Local variables
  const unsigned int n_linestrings_per_lanelet = 2;
  // Left/right boundary of a lanelet
  std::vector<lanelet::LineString3d> la_linestrings(n_linestrings_per_lanelet);
  // Points per linestring
  std::vector<lanelet::Point3d> ls_points = {};

  // Store mapping from original lanelet ids to new (index-based) ids
  std::unordered_map<uint, size_t> map_original_to_new;

  out_lanelets.reserve(msg.segments.size());
  out_lanelet_connections.reserve(msg.segments.size());

  for (size_t idx_segment = 0; idx_segment < msg.segments.size(); idx_segment++) {
    for (size_t idx_linestring = 0; idx_linestring < n_linestrings_per_lanelet; idx_linestring++) {
      ls_points.clear();
      ls_points.reserve(msg.segments[idx_segment].linestrings[idx_linestring].poses.size());
      for (size_t id_pose = 0;
           id_pose < msg.segments[idx_segment].linestrings[idx_linestring].poses.size();
           id_pose++) {
        double p1p =
          msg.segments[idx_segment].linestrings[idx_linestring].poses[id_pose].position.x;
        double p2p =
          msg.segments[idx_segment].linestrings[idx_linestring].poses[id_pose].position.y;
        double p3p =
          msg.segments[idx_segment].linestrings[idx_linestring].poses[id_pose].position.z;

        lanelet::Point3d p{lanelet::utils::getId(), p1p, p2p, p3p};
        ls_points.push_back(p);
      }

      // Create a linestring from the collected points
      lanelet::LineString3d linestring(lanelet::utils::getId(), ls_points);
      la_linestrings[idx_linestring] = linestring;
    }

    // One lanelet consists of 2 boundaries
    lanelet::Lanelet lanelet(lanelet::utils::getId(), la_linestrings[0], la_linestrings[1]);

    out_lanelets.push_back(lanelet);

    // Add empty lanelet connection
    out_lanelet_connections.push_back(LaneletConnection());

    // Set origin lanelet ID (and store it in translation map)
    out_lanelet_connections[idx_segment].original_lanelet_id = msg.segments[idx_segment].id;
    map_original_to_new[msg.segments[idx_segment].id] = idx_segment;

    // Get successor/neighbor lanelet information
    // Write lanelet neighbors
    for (std::size_t i = 0; i < msg.segments[idx_segment].neighboring_segment_id.size(); i++) {
      out_lanelet_connections[idx_segment].neighbor_lanelet_ids.push_back(
        msg.segments[idx_segment].neighboring_segment_id[i]);
    }

    // Write lanelet successors
    for (std::size_t i = 0; i < msg.segments[idx_segment].successor_segment_id.size(); i++) {
      out_lanelet_connections[idx_segment].successor_lanelet_ids.push_back(
        msg.segments[idx_segment].successor_segment_id[i]);
    }

    // The goal_information is not needed in this context, we set it to true for now
    out_lanelet_connections[idx_segment].goal_information = true;
  }

  // Define lambda function to replace old id with a new one
  auto ReplaceAndWarnIfNotFound = [&](auto & segment_ids) {
    for (auto & segment_id : segment_ids) {
      if (segment_id >= 0) {
        auto it = map_original_to_new.find(segment_id);
        if (it != map_original_to_new.end()) {
          segment_id = it->second;
        } else {
          // Key %i seems to be not present in the provided lanelet.
        }
      }
    }
  };

  // Update each entity in all lanelets
  for (auto & lanelet_connection : out_lanelet_connections) {
    ReplaceAndWarnIfNotFound(lanelet_connection.predecessor_lanelet_ids);
    ReplaceAndWarnIfNotFound(lanelet_connection.successor_lanelet_ids);
    ReplaceAndWarnIfNotFound(lanelet_connection.neighbor_lanelet_ids);
  }

  // Fill predecessor field for each lanelet
  CalculatePredecessors(out_lanelet_connections);

  return;
}

lanelet::BasicPoint2d MissionPlannerNode::GetPointOnLane(
  const std::vector<int> & lane, const float x_distance,
  const std::vector<lanelet::Lanelet> & converted_lanelets)
{
  lanelet::BasicPoint2d return_point;  // return value

  if (lane.size() > 0) {
    lanelet::ConstLineString2d linestring =
      CreateLineString(CreateDrivingCorridor(lane, converted_lanelets)
                         .centerline);  // Create linestring for the lane

    // Create point that is float meters in front (x axis)
    lanelet::BasicPoint2d point(x_distance, 0.0);

    // Get projected point on the linestring
    lanelet::BasicPoint2d projected_point = lanelet::geometry::project(linestring, point);

    // Overwrite p (return value)
    return_point.x() = projected_point.x();
    return_point.y() = projected_point.y();
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Overwriting of point may not have occurred properly. Lane is "
      "probably empty.");
  }

  // Return point p
  return return_point;
}

double MissionPlannerNode::CalculateDistanceBetweenPointAndLineString(
  const lanelet::ConstLineString2d & linestring, const lanelet::BasicPoint2d & point)
{
  // Get projected point on the linestring
  lanelet::BasicPoint2d projected_point = lanelet::geometry::project(linestring, point);

  // Calculate the distance between the two points
  double distance = lanelet::geometry::distance2d(point, projected_point);

  // Publish distance
  autoware_planning_msgs::msg::VisualizationDistance d;
  d.distance = distance;
  visualizationDistancePublisher_->publish(d);

  return distance;
}

void MissionPlannerNode::VisualizeLanes(
  const autoware_planning_msgs::msg::RoadSegments & msg,
  const std::vector<lanelet::Lanelet> & converted_lanelets)
{
  // Calculate centerlines, left and right bounds
  std::vector<lanelet::ConstLineString3d> centerlines;
  std::vector<lanelet::ConstLineString3d> left_bounds;
  std::vector<lanelet::ConstLineString3d> right_bounds;

  // Go through every lanelet
  for (const lanelet::Lanelet & l : converted_lanelets) {
    auto centerline = l.centerline();
    auto bound_left = l.leftBound();
    auto bound_right = l.rightBound();

    centerlines.push_back(centerline);
    left_bounds.push_back(bound_left);
    right_bounds.push_back(bound_right);
  }

  auto marker_array = CreateMarkerArray(centerlines, left_bounds, right_bounds, msg);

  // Publish centerlines, left and right bounds
  visualizationPublisher_->publish(marker_array);
}

void MissionPlannerNode::VisualizeCenterlineOfDrivingCorridor(
  const autoware_planning_msgs::msg::RoadSegments & msg,
  const autoware_planning_msgs::msg::DrivingCorridor & driving_corridor)
{
  // Create a marker for the centerline
  visualization_msgs::msg::Marker centerline_marker;
  centerline_marker.header.frame_id = msg.header.frame_id;
  centerline_marker.header.stamp = msg.header.stamp;
  centerline_marker.ns = "centerline";

  // Unique ID
  if (centerline_marker_id_ == std::numeric_limits<int>::max()) {
    // Handle overflow
    centerline_marker_id_ = 0;
  } else {
    centerline_marker.id = centerline_marker_id_++;
  }

  centerline_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  centerline_marker.action = visualization_msgs::msg::Marker::ADD;

  // Set the scale of the marker
  centerline_marker.scale.x = 0.1;  // Line width

  // Set the color of the marker (red, green, blue, alpha)
  centerline_marker.color.r = 1.0;
  centerline_marker.color.g = 0.0;
  centerline_marker.color.b = 0.0;
  centerline_marker.color.a = 1.0;

  // Add points to the marker
  for (const geometry_msgs::msg::Point & p : driving_corridor.centerline) {
    centerline_marker.points.push_back(p);
  }

  // Create a MarkerArray to hold the marker
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(centerline_marker);

  // Publish the marker array
  visualization_publisher_centerline_->publish(marker_array);
}

}  // namespace autoware::mapless_architecture
