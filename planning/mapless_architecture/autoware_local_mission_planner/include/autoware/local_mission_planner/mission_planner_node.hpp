// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef AUTOWARE__LOCAL_MISSION_PLANNER__MISSION_PLANNER_NODE_HPP_
#define AUTOWARE__LOCAL_MISSION_PLANNER__MISSION_PLANNER_NODE_HPP_

#include "autoware/local_mission_planner_common/helper_functions.hpp"
#include "lanelet2_core/geometry/LineString.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_planning_msgs/msg/driving_corridor.hpp"
#include "autoware_planning_msgs/msg/local_map.hpp"
#include "autoware_planning_msgs/msg/mission.hpp"
#include "autoware_planning_msgs/msg/mission_lanes_stamped.hpp"
#include "autoware_planning_msgs/msg/visualization_distance.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

namespace autoware::mapless_architecture
{

// Direction data type
typedef int Direction;
const Direction stay = 0;
const Direction left = 1;
const Direction right = 2;
const Direction left_most = 3;
const Direction right_most = 4;

// Define Lanes
struct Lanes
{
  std::vector<int> ego;
  std::vector<std::vector<int>> left;
  std::vector<std::vector<int>> right;
};

/**
 * Node for mission planner.
 */
class MissionPlannerNode : public rclcpp::Node
{
public:
  MissionPlannerNode();

  /**
   * @brief Get a point on the given lane that is x meters away in x direction
   * (using a projection).
   *
   * @param lane The given lane (std::vector<int>) on which the point is
   * created.
   * @param x_distance The point is created x_distance meters (float) away from
   * the vehicle (in x direction using a projection).
   * @param converted_lanelets The lanelets (std::vector<lanelet::Lanelet>) from
   * the road model.
   * @return lanelet::BasicPoint2d
   */
  lanelet::BasicPoint2d GetPointOnLane_(
    const std::vector<int> & lane, const float x_distance,
    const std::vector<lanelet::Lanelet> & converted_lanelets);

  /**
   * @brief Calculate the distance between a point and a LineString (Euclidean
   * distance).
   *
   * @param linestring The LineString.
   * @param point The point.
   * @return double
   */
  double CalculateDistanceBetweenPointAndLineString_(
    const lanelet::ConstLineString2d & linestring, const lanelet::BasicPoint2d & point);

  /**
   * @brief Recenter a point in a lanelet to its closest point on the centerline
   *
   * @param goal_point The input point which should be re-centered
   * @param road_model The road model which contains the point to be re-centered
   * @return lanelet::BasicPoint2d The re-centered point (which lies on the
   * centerline of its lanelet)
   */
  lanelet::BasicPoint2d RecenterGoalPoint(
    const lanelet::BasicPoint2d & goal_point, const std::vector<lanelet::Lanelet> & road_model);

  /**
    * @brief Function which checks if the vehicle is on the goal lane.
    * This functions returns a bool depending on whether the vehicle is on the
    goal lane or not.
    *
    * @param ego_lanelet_index The index of the ego lanelet (int).
    * @param goal_point The goal point (lanelet::BasicPoint2d).
    * @param converted_lanelets The lanelets from the road model
    (std::vector<lanelet::Lanelet>).
    * @param lanelet_connections The lanelet connections from the road model
    (std::vector<LaneletConnection>).
    * @return bool (is on goal lane or not).
    */
  bool IsOnGoalLane_(
    const int ego_lanelet_index, const lanelet::BasicPoint2d & goal_point,
    const std::vector<lanelet::Lanelet> & converted_lanelets,
    const std::vector<LaneletConnection> & lanelet_connections);

  /**
   * @brief Function which checks if the goal point has a negative x value und
   * must be therefore reset. If the x value is negative the goal point is reset
   * with GetPointOnLane_().
   *
   * @param converted_lanelets The lanelets from the road model
    (std::vector<lanelet::Lanelet>).
   * @param lanelet_connections The lanelet connections from the road model
    (std::vector<LaneletConnection>).
   */
  void CheckIfGoalPointShouldBeReset_(
    const lanelet::Lanelets & converted_lanelets,
    const std::vector<LaneletConnection> & lanelet_connections);

  /**
   * @brief Function for calculating lanes
   *
   * @param converted_lanelets The lanelets given from the road model.
   * @param lanelet_connections The lanelet connections given from the road
   * model.
   * @return Lanes: ego lane, all left lanes, all right lanes
   */
  Lanes CalculateLanes_(
    const std::vector<lanelet::Lanelet> & converted_lanelets,
    std::vector<LaneletConnection> & lanelet_connections);

  /**
    * @brief Function for creating a marker array.
    * This functions creates a visualization_msgs::msg::MarkerArray from the
    given input.
    *
    * @param centerline The centerline which is a LineString.
    * @param left The left boundary which is a LineString.
    * @param right The right boundary which is a LineString.
    * @param msg The LaneletsStamped message.
    * @return MarkerArray (visualization_msgs::msg::MarkerArray).
    */
  visualization_msgs::msg::MarkerArray CreateMarkerArray_(
    const std::vector<lanelet::ConstLineString3d> & centerline,
    const std::vector<lanelet::ConstLineString3d> & left,
    const std::vector<lanelet::ConstLineString3d> & right,
    const autoware_planning_msgs::msg::RoadSegments & msg);

  /**
   * @brief Getter for goal_point_
   *
   * @return lanelet::BasicPoint2d
   */
  lanelet::BasicPoint2d goal_point();

  /**
   * @brief Setter for goal_point_
   *
   * @param goal_point The new value for the goal_point_.
   */
  void goal_point(const lanelet::BasicPoint2d & goal_point);

  /**
   * @brief Create a DrivingCorridor object.
   *
   * @param lane The lane which is a std::vector<int> containing all the indices
   * of the lane.
   * @param converted_lanelets The lanelets (std::vector<lanelet::Lanelet>).
   * @return autoware_planning_msgs::msg::DrivingCorridor
   */
  autoware_planning_msgs::msg::DrivingCorridor CreateDrivingCorridor_(
    const std::vector<int> & lane, const std::vector<lanelet::Lanelet> & converted_lanelets);

  /**
   * @brief The callback for the Mission messages.
   *
   * @param msg The autoware_planning_msgs::msg::Mission message.
   */
  void CallbackMissionMessages_(const autoware_planning_msgs::msg::Mission & msg);

  /**
   * @brief The callback for the LocalMap messages.
   *
   * @param msg The autoware_planning_msgs::msg::LocalMap message.
   */
  void CallbackLocalMapMessages_(const autoware_planning_msgs::msg::LocalMap & msg);

  /**
   * @brief Convert RoadSegments into lanelets.
   *
   * @param msg The message (autoware_planning_msgs::msg::RoadSegments).
   * @param out_lanelets The lanelets (output).
   * @param out_lanelet_connections The lanelet connections (output).
   */
  void ConvertInput2LaneletFormat(
    const autoware_planning_msgs::msg::RoadSegments & msg,
    std::vector<lanelet::Lanelet> & out_lanelets,
    std::vector<LaneletConnection> & out_lanelet_connections);

private:
  /**
   * @brief Function for the visualization of lanes.
   *
   * @param msg The autoware_planning_msgs::msg::RoadSegments message.
   * @param converted_lanelets The lanelets (std::vector<lanelet::Lanelet>).
   */
  void VisualizeLanes_(
    const autoware_planning_msgs::msg::RoadSegments & msg,
    const std::vector<lanelet::Lanelet> & converted_lanelets);

  /**
   * @brief Function for the visualization of the centerline of a driving corridor.
   *
   * @param msg The autoware_planning_msgs::msg::RoadSegments message.
   * @param driving_corridor The considered driving corridor for which the centerline is visualized.
   */
  void VisualizeCenterlineOfDrivingCorridor_(
    const autoware_planning_msgs::msg::RoadSegments & msg,
    const autoware_planning_msgs::msg::DrivingCorridor & driving_corridor);

  /**
   * @brief Function for creating a lanelet::LineString2d.
   *
   * @param points The considered points
   * (std::vector<geometry_msgs::msg::Point>).
   * @return lanelet::LineString2d
   */
  lanelet::LineString2d CreateLineString_(const std::vector<geometry_msgs::msg::Point> & points);

  /**
   * @brief Callback for the odometry messages.
   *
   * @param msg The odometry message (nav_msgs::msg::Odometry).
   */
  void CallbackOdometryMessages_(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief Initiate a lane change.
   *
   * @param direction The direction of the lane change (-1 for left and +1 for
   * right).
   * @param neighboring_lane The neighboring lane.
   */
  void InitiateLaneChange_(const Direction direction, const std::vector<int> & neighboring_lane);

  /**
   * @brief Get all the neighbor lanelets (neighbor lane) of a specific lane on one side.
   *
   * @param lane The considered lane.
   * @param lanelet_connections The lanelet connections.
   * @param vehicle_side The side of the vehicle that is considered (enum).
   * @return std::vector<int>
   */
  std::vector<int> GetAllNeighborsOfLane(
    const std::vector<int> & lane, const std::vector<LaneletConnection> & lanelet_connections,
    const int vehicle_side);

  /**
   * @brief Add the predecessor lanelet to a lane.
   *
   * @param lane_idx The considered lane. The predecessor lanelet is added to
   * the front of the lane.
   * @param lanelet_connections The lanelet connections.
   *
   */
  void InsertPredecessorLanelet(
    std::vector<int> & lane_idx, const std::vector<LaneletConnection> & lanelet_connections);

  /**
   * @brief Calculate the predecessors.
   *
   * @param lanelet_connections The lanelet connections.
   */
  void CalculatePredecessors(std::vector<LaneletConnection> & lanelet_connections);

  //  Declare ROS2 publisher and subscriber
  rclcpp::Subscription<autoware_planning_msgs::msg::LocalMap>::SharedPtr mapSubscriber_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Mission>::SharedPtr missionSubscriber_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualizationPublisher_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    visualization_publisher_centerline_;

  rclcpp::Publisher<autoware_planning_msgs::msg::VisualizationDistance>::SharedPtr
    visualizationDistancePublisher_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualizationGoalPointPublisher_;

  rclcpp::Publisher<autoware_planning_msgs::msg::MissionLanesStamped>::SharedPtr
    missionLanesStampedPublisher_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr OdometrySubscriber_;

  // ROS buffer interface (for TF transforms)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Store previous odometry message
  nav_msgs::msg::Odometry last_odom_msg_;

  // Initialize some variables
  Pose2D pose_prev_;
  bool pose_prev_init_ = false;
  bool b_global_odometry_deprecation_warning_ = false;
  bool received_motion_update_once_ = false;
  Direction target_lane_ = stay;
  Direction mission_ = stay;
  int retry_attempts_ = 0;
  bool lane_change_trigger_success_ = true;
  Direction lane_change_direction_ = stay;

  lanelet::BasicPoint2d goal_point_;
  std::vector<int> ego_lane_;
  std::vector<int> lane_left_;
  std::vector<int> lane_right_;
  std::vector<lanelet::Lanelet> current_lanelets_;

  // ROS parameters
  float distance_to_centerline_threshold_;
  float projection_distance_on_goallane_;
  int retrigger_attempts_max_;

  // Unique ID for each marker
  int centerline_marker_id_ = 0;
};
}  // namespace autoware::mapless_architecture

#endif  // AUTOWARE__LOCAL_MISSION_PLANNER__MISSION_PLANNER_NODE_HPP_
