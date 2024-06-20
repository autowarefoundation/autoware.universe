// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "autoware/local_mission_planner_common/helper_functions.hpp"

#include "lanelet2_core/geometry/Lanelet.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/road_segments.hpp"
#include "db_msgs/msg/lanelets_stamped.hpp"

namespace autoware::mapless_architecture
{
namespace lib_mission_planner
{

double NormalizePsi(const double psi)
{
  // remove multiples of 2 PI by using modulo on absolute value and apply sign
  double psi_out = copysign(fmod(fabs(psi), M_PI * 2.0), psi);

  // restrict psi to [-pi, pi[
  if (psi_out >= M_PI)
    psi_out -= 2.0 * M_PI;
  else if (psi_out < -M_PI)
    psi_out += 2.0 * M_PI;

  return psi_out;
}

std::vector<double> GetPsiForPoints(const std::vector<geometry_msgs::msg::Point> & points)
{
  int num_points = points.size();
  std::vector<double> tang_vecs(num_points * 2);

  // get heading vector for first point
  tang_vecs[0] = points[1].x - points[0].x;
  tang_vecs[1] = points[1].y - points[0].y;

  // use one point before and after the targeted one for heading vector (more
  // stable/robust than relying on a single pair of points)
  for (int i = 1; i < num_points - 1; i++) {
    tang_vecs[2 * i] = points[i + 1].x - points[i - 1].x;
    tang_vecs[2 * i + 1] = points[i + 1].y - points[i - 1].y;
  }

  // get heading vector for last point
  tang_vecs[2 * (num_points - 1)] = points[num_points - 1].x - points[num_points - 2].x;
  tang_vecs[2 * (num_points - 1) + 1] = points[num_points - 1].y - points[num_points - 2].y;

  // calculate heading angles with atan2
  std::vector<double> psi_points(num_points);
  for (int i = 0; i < num_points; i++) {
    psi_points[i] = NormalizePsi(std::atan2(tang_vecs[2 * i + 1], tang_vecs[2 * i]));
  }

  return psi_points;
}

Pose2D::Pose2D()
{
  this->set_xy(0.0, 0.0);
  this->set_psi(0.0);
}
Pose2D::Pose2D(const double x, const double y, const double psi /* = 0.0 */)
{
  this->set_xy(x, y);
  this->set_psi(psi);
}
double Pose2D::get_x() const
{
  return this->xy_(0);
}
double Pose2D::get_y() const
{
  return this->xy_(1);
}
Eigen::Vector2d Pose2D::get_xy() const
{
  return this->xy_;
}
double Pose2D::get_psi() const
{
  return this->psi_;
}
geometry_msgs::msg::Point Pose2D::get_point() const
{
  geometry_msgs::msg::Point tmp_point;
  tmp_point.x = this->xy_(0);
  tmp_point.y = this->xy_(1);
  return tmp_point;
}
void Pose2D::set_x(const double x)
{
  this->xy_(0) = x;
}
void Pose2D::set_y(const double y)
{
  this->xy_(1) = y;
}
void Pose2D::set_xy(const double x, const double y)
{
  this->xy_ = {x, y};
}
void Pose2D::set_xy(const Eigen::Vector2d xy)
{
  this->xy_ = xy;
}
void Pose2D::set_psi(const double psi)
{
  this->psi_ = psi;
}

Pose2D TransformToNewCosy2D(const Pose2D cosy_rel, const Pose2D pose_prev)
{
  Pose2D pose_out;

  pose_out.set_xy(
    Eigen::Rotation2D<double>(-cosy_rel.get_psi()) * Eigen::Translation2d(-cosy_rel.get_xy()) *
    pose_prev.get_xy());
  pose_out.set_psi(NormalizePsi(pose_prev.get_psi() - cosy_rel.get_psi()));

  return pose_out;
}

double GetYawFromQuaternion(const double x, const double y, const double z, const double w)
{
  tf2::Quaternion tmp_quat(x, y, z, w);
  tf2::Matrix3x3 m(tmp_quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

// Declare a static logger
static rclcpp::Logger static_logger = rclcpp::get_logger("static_logger");

autoware_planning_msgs::msg::RoadSegments ConvertLaneletsStamped2RoadSegments(
  const db_msgs::msg::LaneletsStamped & msg)
{
  // Initialize road segments message
  autoware_planning_msgs::msg::RoadSegments road_segments;

  // Fill message header and pose
  road_segments.header = msg.header;
  road_segments.pose = msg.pose;

  // Convert lanelets to segments if lanelets are not empty
  if (!msg.lanelets.empty()) {
    for (const db_msgs::msg::Lanelet & lanelet : msg.lanelets) {
      // Initialize a segment
      autoware_planning_msgs::msg::Segment segment;

      // Fill the segment with basic information
      segment.id = lanelet.id;
      segment.successor_lanelet_id = lanelet.successor_lanelet_id;
      segment.neighboring_lanelet_id = lanelet.neighboring_lanelet_id;

      // Copy linestrings data
      for (int i = 0; i < 2; ++i) {
        // Copy points from the original linestring to the new one if points are not empty
        if (!lanelet.linestrings[i].points.empty()) {
          segment.linestrings[i].poses.reserve(lanelet.linestrings[i].points.size());

          for (const db_msgs::msg::DBPoint & point : lanelet.linestrings[i].points) {
            segment.linestrings[i].poses.push_back(point.pose);
          }
        } else {
          RCLCPP_WARN(
            static_logger,
            "Linestring does not contain points (ConvertLaneletsStamped2RoadSegments)!");
        }
      }

      // Add the filled segment to the road_segments message
      road_segments.segments.push_back(segment);
    }
  }

  return road_segments;
}

std::vector<std::vector<int>> GetAllSuccessorSequences(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet)
{
  AdjacentLaneType adjacent_lane_type = AdjacentLaneType::kSuccessors;

  return GetAllLaneletSequences(lanelet_connections, id_initial_lanelet, adjacent_lane_type);
}

std::vector<std::vector<int>> GetAllLaneletSequences(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet,
  const AdjacentLaneType adjacent_lane_type)
{
  // initialize helper variables
  bool do_include_navigation_info = false;

  std::vector<int> lanelets_already_visited;
  std::vector<int> lanelet_id_sequence_temp{id_initial_lanelet};

  std::vector<std::vector<int>> lanelet_sequences;

  // loop as long as all successors of the initial lanelet have been searched
  // maximum iteration depth is (number_of_lanelets - 1) * 2
  for (size_t i = 0; i < lanelet_connections.size() * 2; i++) {
    // IDs which are relevant for searching adjacent lanelets (either successors
    // or predecessors)
    std::vector<int> ids_adjacent_lanelets;

    if (adjacent_lane_type == AdjacentLaneType::kPredecessors) {
      ids_adjacent_lanelets =
        lanelet_connections[lanelet_id_sequence_temp.back()].predecessor_lanelet_ids;
    } else {
      ids_adjacent_lanelets =
        lanelet_connections[lanelet_id_sequence_temp.back()].successor_lanelet_ids;
    }

    const std::vector<int> ids_relevant_adjacent_lanelets = GetRelevantAdjacentLanelets(
      lanelet_connections, ids_adjacent_lanelets, do_include_navigation_info);

    auto [lanelet_id_sequence_completed, do_exit_outer_for_loop] = GetCompletedLaneletSequence(
      lanelet_id_sequence_temp, lanelets_already_visited, ids_relevant_adjacent_lanelets,
      id_initial_lanelet);

    if (do_exit_outer_for_loop) break;

    // store returned complete lanelet id sequence
    if (!lanelet_id_sequence_completed.empty()) {
      lanelet_sequences.push_back(lanelet_id_sequence_completed);
    }
  }
  return lanelet_sequences;
}

std::vector<int> GetRelevantAdjacentLanelets(
  const std::vector<LaneletConnection> & lanelet_connections,
  const std::vector<int> ids_adjacent_lanelets, const bool do_include_navigation_info)
{
  std::vector<int> ids_relevant_successors;

  // return all successors if navigation info is not relevant
  if (do_include_navigation_info) {
    // loop through all successors and check if they lead towards the navigation
    // goal
    if (ids_adjacent_lanelets.front() >= 0) {
      for (std::size_t i = 0; i < ids_adjacent_lanelets.size(); i++) {
        // check if successor leads to goal
        if (lanelet_connections[ids_adjacent_lanelets[i]].goal_information) {
          ids_relevant_successors.push_back(ids_adjacent_lanelets[i]);
        }
      }
    }
    // default return: valid successor lanelet not available
    if (ids_relevant_successors.empty()) {
      ids_relevant_successors.push_back(-1);
    }
  } else {
    ids_relevant_successors = ids_adjacent_lanelets;
  }

  // FIXME: avoid that list is empty -> has to be -1 if no relevant lanelet
  // exists
  // this fixes an issue that originates in the lanelet converter; should be
  // fixed there!
  if (ids_relevant_successors.empty()) {
    ids_relevant_successors.push_back(-1);
  }

  return ids_relevant_successors;
}

std::tuple<std::vector<int>, bool> GetCompletedLaneletSequence(
  std::vector<int> & lanelet_id_sequence_current, std::vector<int> & lanelets_already_visited,
  const std::vector<int> ids_relevant_lanelets, const int id_initial_lanelet)
{
  std::vector<int> lanelet_id_sequence_completed;
  bool do_exit_outer_for_loop = false;

  // check if an adjacent lanelet is even available
  if (ids_relevant_lanelets.front() >= 0) {
    bool has_relevant_successor = false;

    // loop though all relevant adjacent IDs and check whether or not they
    // have already been visited
    for (const int & successor_id : ids_relevant_lanelets) {
      if (
        std::find(lanelets_already_visited.begin(), lanelets_already_visited.end(), successor_id) ==
        lanelets_already_visited.end()) {
        // add new ID to already visited vector
        lanelets_already_visited.push_back(successor_id);
        // add currently visited ID to temporary adjacent lanelet sequence
        lanelet_id_sequence_current.push_back(successor_id);
        has_relevant_successor = true;
        break;
      }
    }

    // if all available adjacent lanelets were already visited, go back to
    // parent lanelet or exit loop
    if (!has_relevant_successor) {
      // pop parent lanelet except it is already the initial lanelet
      if (lanelet_id_sequence_current.back() != id_initial_lanelet) {
        lanelet_id_sequence_current.pop_back();
      } else {
        // exit loop if initial lanelet has no relevant adjacent lanelets left
        do_exit_outer_for_loop = true;
      }
    }

  } else {
    // exit loop if initial lanelet has already no relevant adjacent lanelets
    if (lanelet_id_sequence_current.back() == id_initial_lanelet) {
      do_exit_outer_for_loop = true;
    } else {
      // store lanelet sequence when it is completed (no unvisited adjacent
      // lanelets left from current lanelet)
      lanelet_id_sequence_completed = lanelet_id_sequence_current;

      // go back to parent lanelet ID and continue search from there
      lanelet_id_sequence_current.pop_back();
    }
  }
  return {lanelet_id_sequence_completed, do_exit_outer_for_loop};
}

std::vector<int> GetAllNeighboringLaneletIDs(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet,
  const VehicleSide side)
{
  int id_current_lanelet = id_initial_lanelet;
  std::vector<int> lanelet_id_neighbors;

  // this function is only intended to return all neighboring lanelets, not only
  // the ones leading towards goal. Therefore, this flag is set false for the
  // sub-function.
  const bool do_include_navigation_info = false;

  // loop as long as all left or right neighbors of the initial lanelet have
  // been searched
  // maximum iteration depth is: number_of_lanelets
  for (size_t i = 0; i < lanelet_connections.size(); i++) {
    int idx_tmp = GetNeighboringLaneletID(
      lanelet_connections, id_current_lanelet, side, do_include_navigation_info);

    // if ID >= 0, continue search, else break
    if (idx_tmp >= 0) {
      id_current_lanelet = idx_tmp;
      lanelet_id_neighbors.push_back(id_current_lanelet);
    } else {
      // if return vector is still empty because no neighbors are available,
      // return -1
      if (lanelet_id_neighbors.empty()) {
        lanelet_id_neighbors.push_back(-1);
      }
      break;
    }
  }

  return lanelet_id_neighbors;
}

int GetNeighboringLaneletID(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet,
  const VehicleSide side, const bool do_include_navigation_info, const int recursiveness)
{
  // set default return id if initial lane has already no neighbors
  int id_neighbor_lanelet = -1;

  // check if most outside lane is searched
  // here: return current lanelet's ID when no neighbors are available
  if (recursiveness == -1) {
    id_neighbor_lanelet = id_initial_lanelet;
  }

  // get ID of current lanelet's neighboring lanelet
  int id_neighbor_lanelet_tmp = lanelet_connections[id_initial_lanelet].neighbor_lanelet_ids[side];

  // init counter to track current level of recursiveness in loop
  int recursiveness_counter = 0;

  // loop as long as all neighbors of the initial lanelet have been searched
  // maximum iteration depth: number_of_lanelets
  for (size_t i = 0; i < lanelet_connections.size(); i++) {
    // break while loop if desired recursiveness is reached
    if (recursiveness != -1 && recursiveness_counter >= recursiveness) break;

    // check if neighbor is available
    if (id_neighbor_lanelet_tmp >= 0) {
      // break if navigation info is considered AND the lanelet does not lead
      // towards goal
      if (
        do_include_navigation_info &&
        !lanelet_connections[id_neighbor_lanelet_tmp].goal_information) {
        break;
      }

      // store current neighbor lanelet ID
      id_neighbor_lanelet = id_neighbor_lanelet_tmp;
      // query neighbor of neighbor and use as new start lanelet
      id_neighbor_lanelet_tmp =
        lanelet_connections[id_neighbor_lanelet_tmp].neighbor_lanelet_ids[side];

    } else {
      // return -1 if lanelet ID at specific recursiveness is requested
      if (recursiveness >= 0) {
        id_neighbor_lanelet = -1;
      }
      break;
    }
    recursiveness_counter++;
  }
  return id_neighbor_lanelet;
}

std::vector<std::vector<int>> GetAllPredecessorSequences(
  const std::vector<LaneletConnection> & lanelet_connections, const int id_initial_lanelet)
{
  AdjacentLaneType adjacent_lane_type = AdjacentLaneType::kPredecessors;

  return GetAllLaneletSequences(lanelet_connections, id_initial_lanelet, adjacent_lane_type);
}

int FindOccupiedLaneletID(
  const std::vector<lanelet::Lanelet> & lanelets, const lanelet::BasicPoint2d & position)
{
  int id_occupied_lanelet = -1;

  // check if position is within one of the available lanelet
  for (size_t i = 0; i < lanelets.size(); i++) {
    if (lanelet::geometry::inside(lanelets[i], position)) {
      id_occupied_lanelet = i;
      break;
    }
  }

  return id_occupied_lanelet;
}

int FindEgoOccupiedLaneletID(const std::vector<lanelet::Lanelet> & lanelets)
{
  const lanelet::BasicPoint2d position_ego = lanelet::BasicPoint2d(0.0, 0.0);

  return FindOccupiedLaneletID(lanelets, position_ego);
}

}  // namespace lib_mission_planner

}  // namespace autoware::mapless_architecture
