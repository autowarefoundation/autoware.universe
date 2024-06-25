// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef AUTOWARE__MISSION_LANE_CONVERTER__MISSION_LANE_CONVERTER_NODE_HPP_
#define AUTOWARE__MISSION_LANE_CONVERTER__MISSION_LANE_CONVERTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_planning_msgs/msg/path.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/mission_lanes_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <string>
#include <tuple>
#include <vector>

namespace autoware::mapless_architecture
{

/**
 * Node to convert the mission lane to an autoware trajectory type.
 */
class MissionLaneConverterNode : public rclcpp::Node
{
public:
  MissionLaneConverterNode();

  /**
   * @brief Converts the mission message into a reference trajectory which is
   * forwarded to a local trajectory planner for refinement.
   *
   * @param msg The mission lanes
   * @return std::tuple<autoware_auto_planning_msgs::msg::Trajectory,
   * visualization_msgs::msg::Marker, autoware_auto_planning_msgs::msg::Path,
   * visualization_msgs::msg::Marker>
   */
  std::tuple<
    autoware_auto_planning_msgs::msg::Trajectory, visualization_msgs::msg::Marker,
    autoware_auto_planning_msgs::msg::Path, visualization_msgs::msg::MarkerArray>
  ConvertMissionToTrajectory(const autoware_planning_msgs::msg::MissionLanesStamped & msg);

private:
  /**
   * @brief Computes a trajectory based on the mission planner input.
   *
   * @param msg Mission lanes from the mission planner module
   */
  void MissionLanesCallback_(const autoware_planning_msgs::msg::MissionLanesStamped & msg);

  /**
   * @brief Adds a trajectory point to the pre-allocated ROS message.
   *
   * @param trj_msg The pre-allocated trajectory message
   * @param x The x position of the point to be added
   * @param y The y position of the point to be added
   * @param v_x The longitudinal velocity of the point to be added
   */
  void AddTrajectoryPoint_(
    autoware_auto_planning_msgs::msg::Trajectory & trj_msg, const double x, const double y,
    const double v_x);

  /**
   * @brief Adds a visualization point to the pre-allocated ROS message.
   *
   * @param trj_vis The pre-allocated visualization message
   * @param x The x position of the marker to be added
   * @param y The y position of the marker to be added
   * @param v_x The id of the marker to be added
   */
  void AddPointVisualizationMarker_(
    visualization_msgs::msg::Marker & trj_vis, const double x, const double y, const int id_marker);

  /**
   * @brief Computes and adds a heading information to the pre-allocated input
   * trajectory based on the x/y positions given in the input argument
   *
   * @param trj_msg
   */
  void AddHeadingToTrajectory_(autoware_auto_planning_msgs::msg::Trajectory & trj_msg);

  /**
   * @brief Timed callback which shall be executed until a first valid local
   * road model (where the vehicle can be located on) could have been computed
   * once. This can be considered to be a workaround during startup of the local
   * environment generation which starts in a certain distance in front of the
   * vehicle.
   *
   */
  void TimedStartupTrajectoryCallback();

  /**
   *@brief Create a path bound.
   *
   *@param bound_path The path.
   *@param path_vis The visualization marker.
   *@param bound_mission_lane The mission lane.
   *@param id_marker The ID marker.
   */
  void CreatePathBound_(
    std::vector<geometry_msgs::msg::Point> & bound_path, visualization_msgs::msg::Marker & path_vis,
    const std::vector<geometry_msgs::msg::Point> & bound_mission_lane, const int id_marker);

  /**
   *@brief Add a path point.
   *
   *@param pth_msg The path.
   *@param x The x value.
   *@param y The y value.
   *@param v_x The v_x value.
   */
  void AddPathPoint_(
    autoware_auto_planning_msgs::msg::Path & pth_msg, const double x, const double y,
    const double v_x);

  /**
   *@brief Create a motion planner input.
   *
   *@param trj_msg The trajectory.
   *@param path_msg The path.
   *@param trj_vis The visualization marker for the trajectory.
   *@param path_vis The visualization marker for the path.
   *@param centerline_mission_lane The centerline of the mission lane.
   */
  void CreateMotionPlannerInput_(
    autoware_auto_planning_msgs::msg::Trajectory & trj_msg,
    autoware_auto_planning_msgs::msg::Path & path_msg, visualization_msgs::msg::Marker & trj_vis,
    visualization_msgs::msg::Marker & path_vis,
    const std::vector<geometry_msgs::msg::Point> & centerline_mission_lane);

  /**
   * @brief Callback to store the most recent odometry update.
   *
   * @param msg Odometry input message
   */
  void CallbackOdometryMessages_(const nav_msgs::msg::Odometry & msg);

  /**
   * @brief Template to transform both Autoware::Path and Autoware::Trajectory into a global map
   * frame.
   *
   * @tparam T autoware_auto_planning_msgs::msg::Path, autoware_auto_planning_msgs::msg::Trajectory
   * @param input Input ROS message which content must be transformed into the global map frame
   * @return T Same as input message with the content being valid in the global map
   */
  template <typename T>
  T TransformToGlobalFrame(const T & input);

  /**
   * @brief Visualization helper for the global trajectory
   *
   * @param trj_msg Trajectory in global map frame
   * @return visualization_msgs::msg::Marker
   */
  visualization_msgs::msg::Marker GetGlobalTrjVisualization_(
    const autoware_auto_planning_msgs::msg::Trajectory & trj_msg);

  // Declare ROS2 publisher and subscriber

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  rclcpp::Subscription<autoware_planning_msgs::msg::MissionLanesStamped>::SharedPtr
    mission_lane_subscriber_;

  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr trajectory_publisher_,
    trajectory_publisher_global_;

  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Path>::SharedPtr path_publisher_,
    path_publisher_global_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vis_trajectory_publisher_,
    vis_trajectory_publisher_global_, vis_odometry_publisher_global_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_path_publisher_;

  rclcpp::Publisher<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;

  // Switch to print a warning about wrongly configured odometry frames
  bool b_global_odometry_deprecation_warning_ = false;
  bool received_motion_update_once_ = false;

  // Store initial and last available odom messages
  nav_msgs::msg::Odometry last_odom_msg_, initial_odom_msg_;

  // Workaround to start the vehicle driving into the computed local road model
  bool mission_lanes_available_once_ = false;

  // ROS parameters
  float target_speed_;
};
}  // namespace autoware::mapless_architecture

#endif  // AUTOWARE__MISSION_LANE_CONVERTER__MISSION_LANE_CONVERTER_NODE_HPP_
