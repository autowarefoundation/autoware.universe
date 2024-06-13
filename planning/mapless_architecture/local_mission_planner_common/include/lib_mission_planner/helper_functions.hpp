// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#ifndef LIB_MISSION_PLANNER__HELPER_FUNCTIONS_HPP_
#define LIB_MISSION_PLANNER__HELPER_FUNCTIONS_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "mission_planner_messages/msg/road_segments.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "db_msgs/msg/lanelets_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <vector>

namespace lib_mission_planner
{

/**
 * @brief A class for a 2D pose.
 *
 */
class Pose2D
{
public:
  Pose2D();
  Pose2D(const double x, const double y, const double psi = 0.0);

  // accessors and mutators
  double get_x() const;
  double get_y() const;
  Eigen::Vector2d get_xy() const;
  double get_psi() const;
  geometry_msgs::msg::Point get_point() const;
  void set_x(const double x);
  void set_y(const double y);
  void set_xy(const double x, const double y);
  void set_xy(const Eigen::Vector2d xy);
  void set_psi(const double psi);

private:
  // data variables
  Eigen::Vector2d xy_;
  double psi_;
};

/**
 * Represent a 2D pose (pose_prev) in a new origin / coordinate system, which
 * is given in relation to the previous coordinate system / origin
 * (cosy_rel).
 *
 * If this relation is not known, it can be calculated with this function by
 * providing the absolute pose of the new cosy as "pose_prev" and the
 * absolute pose of the old/current cosy as "cosy_rel".
 *
 * @param cosy_rel translation and rotation between the current/old cosy and
 * a new/go-to cosy
 * @param pose_prev coordinates and heading of a pose in the current/old cosy
 * @return Pose coordinates and heading of pose_prev in the new cosy
 * (defined by the shift cosy_rel between previous and current cosy)
 */
Pose2D TransformToNewCosy2D(const Pose2D cosy_rel, const Pose2D pose_prev);

/**
 * @brief Get the yaw value from a quaternion.
 *
 * @param x The x value.
 * @param y The y value.
 * @param z The z value.
 * @param w The w value.
 * @return double
 */
double GetYawFromQuaternion(const double x, const double y, const double z, const double w);

/**
 * @brief Normalize the psi value.
 *
 * @param psi The psi value.
 * @return double
 */
double NormalizePsi(const double psi);

/**
 * @brief Get the psi value given some points.
 *
 * @param points The points (std::vector<geometry_msgs::msg::Point).
 * @return std::vector<double>
 */
std::vector<double> GetPsiForPoints(const std::vector<geometry_msgs::msg::Point> & points);

/**
 * @brief Convert the LaneletsStamped message into a RoadSegments message.
 *
 * @param msg The message (db_msgs::msg::LaneletsStamped).
 * @return mission_planner_messages::msg::RoadSegments.
 */
mission_planner_messages::msg::RoadSegments ConvertLaneletsStamped2RoadSegments(
  const db_msgs::msg::LaneletsStamped & msg);

}  // namespace lib_mission_planner

#endif  // LIB_MISSION_PLANNER__HELPER_FUNCTIONS_HPP_
