// Copyright 2024 driveblocks GmbH
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

#ifndef AUTOWARE__LOCAL_MISSION_PLANNER_COMMON__HELPER_FUNCTIONS_HPP_
#define AUTOWARE__LOCAL_MISSION_PLANNER_COMMON__HELPER_FUNCTIONS_HPP_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "lanelet2_core/primitives/Lanelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "autoware_planning_msgs/msg/driving_corridor.hpp"
#include "autoware_planning_msgs/msg/local_map.hpp"
#include "autoware_planning_msgs/msg/mission.hpp"
#include "autoware_planning_msgs/msg/mission_lanes_stamped.hpp"
#include "autoware_planning_msgs/msg/road_segments.hpp"
#include "autoware_planning_msgs/msg/visualization_distance.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <tuple>
#include <vector>

namespace autoware::mapless_architecture
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

  // Accessors and mutators
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
  // Data variables
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
 * @param cosy_rel Translation and rotation between the current/old cosy and
 * a new/go-to cosy
 * @param pose_prev Coordinates and heading of a pose in the current/old cosy
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
 * @brief Holds
 * the origin lanelet Id,
 * the predecessor lanelet Ids,
 * the successor lanelet Ids,
 * the neighboring lanelet Ids
 * the goal information
 *
 */
struct LaneletConnection
{
  int original_lanelet_id;
  std::vector<int> predecessor_lanelet_ids;
  std::vector<int> successor_lanelet_ids;
  std::vector<int> neighbor_lanelet_ids;
  bool goal_information;
};

/**
 * @brief The adjacent lane type.
 *
 */
enum AdjacentLaneType { kSuccessors = 0, kPredecessors = 1 };

/**
 * @brief Get all sequences of successor lanelets to the initial lanelet.
          This function wraps GetAllLaneletSequences()

 * @param lanelet_connections   Relation between individual lanelets
 *                              (successors/predecessors/neighbors)
 * @param id_initial_lanelet    ID of lanelet from where neighbor search is
                                started
 * @return Collection of sequences of all successor lanelets
 */

std::vector<std::vector<int>> GetAllSuccessorSequences(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet);

/**
* @brief Get all sequences of adjacent (either successors or predecessors)
        lanelets to the initial lanelet.

* @param lanelet_connections   Relation between individual lanelets
*                              (successors/predecessors/neighbors)
* @param id_initial_lanelet    ID of lanelet from where neighbor search is
                              started
* @param adjacent_lane_type    Specifies whether predecessors or successors
should be targeted (e.g. lanelet_tools::AdjacentLaneType::kPredecessors)
*
* @return Collection of sequences of all adjacent lanelets
*/
std::vector<std::vector<int>> GetAllLaneletSequences(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet,
  const AdjacentLaneType adjacent_lane_type);

/**
* @brief Find relevant adjacent (successors or predecessors) lanelets (currently relevant means
leading towards goal) among a set of provided adjacent lanelets (ids_adjacent_lanelets)
*
* @param lanelet_connections   Relation between individual lanelets
                              (successors/neighbors)
* @param ids_adjacent_lanelets IDs of all available adjacent lanelets
                              (either successors or predecessors)
* @param do_include_navigation_info  Whether to use navigation info to
*            determine relevant successors (true) or not (false); if navigation info is not used,
the ID of the first direct successors will be returned
* @return    ID of relevant successor lanelet
*/
std::vector<int> GetRelevantAdjacentLanelets(
  const std::vector<LaneletConnection> & lanelet_connections,
  const std::vector<int> ids_adjacent_lanelets, const bool do_include_navigation_info);

/**
 * @brief Get a complete lanelet ID sequence starting from an initial lanelet.
 *
 * @param lanelet_id_sequence_current  Current lanelet ID sequence (of
 * previous iteration); this is the start for the search in the current iteration
 * @param lanelets_already_visited      List of already visited lanelet IDs
 * @param ids_relevant_lanelets         IDs of the relevant adjacent (successor or predecessor)
 * lanelets
 * @param id_initial_lanelet            ID of lanelet from which search was
 *                                      started initially
 * @return  - Vector containing IDs of a completed lanelet sequence; is empty
 *            when the sequence is not completed yet (i.e. there are still some unvisited adjacent
 * lanelets left)
 *          - Flag to indicate whether outer for loop should be exited; this is necessary when no
 * unvisited lanelets are left from the initial lanelet
 */
std::tuple<std::vector<int>, bool> GetCompletedLaneletSequence(
  std::vector<int> & lanelet_id_sequence_current, std::vector<int> & lanelets_already_visited,
  const std::vector<int> ids_relevant_lanelets, const int id_initial_lanelet);

/**
 * @brief The vehicle side (left or right).
 *
 */
enum VehicleSide { kLeft = 0, kRight = 1 };

/**
 * @brief Get all neighboring lanelet IDs on one side.
 *
 * @param lanelet_connections   Relation between individual lanelets
 *                              (successors/neighbors)
 * @param id_initial_lanelet    ID of lanelet from where neighbor search is
                                started
 * @param side                  Side of initial lanelet where neighboring
                                lanelet ID should get searched
 * @return IDs of all neighboring lanelets (returns -1 if no neighbor
 available)
 */
std::vector<int> GetAllNeighboringLaneletIDs(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet,
  const VehicleSide side);

/**
 * @brief Get the ID of a specified neighboring lanelet.
 *
 * @param lanelet_connections   Relation between individual lanelets
 *                              (successors/neighbors)
 * @param id_initial_lanelet    ID of lanelet from where neighbor search is
                                started
 * @param side                  Side of initial lanelet where neighboring
                                lanelet ID should get searched
 * @param do_include_navigation_info  Whether to use navigation info to
 *                                    determine relevant neighbors (true) or
 *                                    not (false)
 * @param recursiveness (defaults to 1)
 *    - level of recursiveness for neighbor search
 *    - recursivenes=-1 means that the most outside lanes are requested (returns initial lanelet ID
 if no neighbors are available -> initial lanelet is the most outside lane)
 *    - recursiveness>=0 means that the direct neighbors (1), neighbors of neighbors (2), ..., are
 searched (returns -1 if no lanelet is available at the specified level of recursiveness)
 *
 * @return ID of neighboring lanelet (returns -1 if no neighbor available)
 */
int GetNeighboringLaneletID(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet,
  const VehicleSide side, const bool do_include_navigation_info, const int recursiveness = 1);

/**
 * @brief Get all sequences of predecessor lanelets to the initial lanelet.
          This function wraps GetAllLaneletSequences()

 * @param lanelet_connections   Relation between individual lanelets
 *                              (successors/predecessors/neighbors)
 * @param id_initial_lanelet    ID of lanelet from where neighbor search is
                                started
 * @return Collection of sequences of all predecessor lanelets
 */
std::vector<std::vector<int>> GetAllPredecessorSequences(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet);

/**
 * @brief Finds the ID of lanelet where a given position is located in
 *
 * @param lanelets  Collection of lanelets to search
 * @param position  Position of which the corresponding lanelet ID is wanted
 * @return          lanelet ID (returns -1 if no match)
 */
int FindOccupiedLaneletID(
  const std::vector<lanelet::Lanelet> & lanelets, const lanelet::BasicPoint2d & position);

/**
 * @brief Finds the ID of the ego vehicle occupied lanelet
 *
 * @param lanelets  Collection of lanelets to search
 * @return          lanelet ID (returns -1 if no match)
 */
int FindEgoOccupiedLaneletID(const std::vector<lanelet::Lanelet> & lanelets);

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
visualization_msgs::msg::MarkerArray CreateMarkerArray(
  const std::vector<lanelet::ConstLineString3d> & centerline,
  const std::vector<lanelet::ConstLineString3d> & left,
  const std::vector<lanelet::ConstLineString3d> & right,
  const autoware_planning_msgs::msg::RoadSegments & msg);

/**
 * @brief Create a DrivingCorridor object.
 *
 * @param lane The lane which is a std::vector<int> containing all the indices
 * of the lane.
 * @param converted_lanelets The lanelets (std::vector<lanelet::Lanelet>).
 * @return autoware_planning_msgs::msg::DrivingCorridor
 */
autoware_planning_msgs::msg::DrivingCorridor CreateDrivingCorridor(
  const std::vector<int> & lane, const std::vector<lanelet::Lanelet> & converted_lanelets);

/**
 * @brief Function for creating a lanelet::LineString2d.
 *
 * @param points The considered points
 * (std::vector<geometry_msgs::msg::Point>).
 * @return lanelet::LineString2d
 */
lanelet::LineString2d CreateLineString(const std::vector<geometry_msgs::msg::Point> & points);

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
  std::vector<int> & lane, const std::vector<LaneletConnection> & lanelet_connections);

/**
 * @brief Calculate the predecessors.
 *
 * @param lanelet_connections The lanelet connections.
 */
void CalculatePredecessors(std::vector<LaneletConnection> & lanelet_connections);

}  // namespace autoware::mapless_architecture

#endif  // AUTOWARE__LOCAL_MISSION_PLANNER_COMMON__HELPER_FUNCTIONS_HPP_
