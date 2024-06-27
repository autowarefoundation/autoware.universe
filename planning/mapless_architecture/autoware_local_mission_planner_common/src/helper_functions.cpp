// Copyright 2024 driveblocks GmbH
// driveblocks proprietary license
#include "autoware/local_mission_planner_common/helper_functions.hpp"

#include "lanelet2_core/geometry/Lanelet.h"
#include "rclcpp/rclcpp.hpp"

#include "autoware_planning_msgs/msg/road_segments.hpp"

namespace autoware::mapless_architecture
{

double NormalizePsi(const double psi)
{
  // Remove multiples of 2 PI by using modulo on absolute value and apply sign
  double psi_out = copysign(fmod(fabs(psi), M_PI * 2.0), psi);

  // Restrict psi to [-pi, pi[
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

  // Get heading vector for first point
  tang_vecs[0] = points[1].x - points[0].x;
  tang_vecs[1] = points[1].y - points[0].y;

  // Use one point before and after the targeted one for heading vector (more
  // stable/robust than relying on a single pair of points)
  for (int i = 1; i < num_points - 1; i++) {
    tang_vecs[2 * i] = points[i + 1].x - points[i - 1].x;
    tang_vecs[2 * i + 1] = points[i + 1].y - points[i - 1].y;
  }

  // Get heading vector for last point
  tang_vecs[2 * (num_points - 1)] = points[num_points - 1].x - points[num_points - 2].x;
  tang_vecs[2 * (num_points - 1) + 1] = points[num_points - 1].y - points[num_points - 2].y;

  // Calculate heading angles with atan2
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
  // Initialize helper variables
  bool do_include_navigation_info = false;

  std::vector<int> lanelets_already_visited;
  std::vector<int> lanelet_id_sequence_temp{id_initial_lanelet};

  std::vector<std::vector<int>> lanelet_sequences;

  // Loop as long as all successors of the initial lanelet have been searched
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

    // Store returned complete lanelet id sequence
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

  // Return all successors if navigation info is not relevant
  if (do_include_navigation_info) {
    // Loop through all successors and check if they lead towards the navigation
    // goal
    if (ids_adjacent_lanelets.front() >= 0) {
      for (std::size_t i = 0; i < ids_adjacent_lanelets.size(); i++) {
        // Check if successor leads to goal
        if (lanelet_connections[ids_adjacent_lanelets[i]].goal_information) {
          ids_relevant_successors.push_back(ids_adjacent_lanelets[i]);
        }
      }
    }
    // Default return: valid successor lanelet not available
    if (ids_relevant_successors.empty()) {
      ids_relevant_successors.push_back(-1);
    }
  } else {
    ids_relevant_successors = ids_adjacent_lanelets;
  }

  // FIXME: avoid that list is empty -> has to be -1 if no relevant lanelet
  // exists, this fixes an issue that originates in the lanelet converter; should be fixed there!
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

  // Check if an adjacent lanelet is even available
  if (ids_relevant_lanelets.front() >= 0) {
    bool has_relevant_successor = false;

    // Loop though all relevant adjacent IDs and check whether or not they
    // have already been visited
    for (const int & successor_id : ids_relevant_lanelets) {
      if (
        std::find(lanelets_already_visited.begin(), lanelets_already_visited.end(), successor_id) ==
        lanelets_already_visited.end()) {
        // Add new ID to already visited vector
        lanelets_already_visited.push_back(successor_id);
        // Add currently visited ID to temporary adjacent lanelet sequence
        lanelet_id_sequence_current.push_back(successor_id);
        has_relevant_successor = true;
        break;
      }
    }

    // If all available adjacent lanelets were already visited, go back to
    // parent lanelet or exit loop
    if (!has_relevant_successor) {
      // Pop parent lanelet except it is already the initial lanelet
      if (lanelet_id_sequence_current.back() != id_initial_lanelet) {
        lanelet_id_sequence_current.pop_back();
      } else {
        // Exit loop if initial lanelet has no relevant adjacent lanelets left
        do_exit_outer_for_loop = true;
      }
    }

  } else {
    // Exit loop if initial lanelet has already no relevant adjacent lanelets
    if (lanelet_id_sequence_current.back() == id_initial_lanelet) {
      do_exit_outer_for_loop = true;
    } else {
      // Store lanelet sequence when it is completed (no unvisited adjacent
      // lanelets left from current lanelet)
      lanelet_id_sequence_completed = lanelet_id_sequence_current;

      // Go back to parent lanelet ID and continue search from there
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

  // This function is only intended to return all neighboring lanelets, not only
  // the ones leading towards goal. Therefore, this flag is set false for the
  // sub-function.
  const bool do_include_navigation_info = false;

  // Loop as long as all left or right neighbors of the initial lanelet have
  // been searched, maximum iteration depth is: number_of_lanelets
  for (size_t i = 0; i < lanelet_connections.size(); i++) {
    int idx_tmp = GetNeighboringLaneletID(
      lanelet_connections, id_current_lanelet, side, do_include_navigation_info);

    // If ID >= 0, continue search, else break
    if (idx_tmp >= 0) {
      id_current_lanelet = idx_tmp;
      lanelet_id_neighbors.push_back(id_current_lanelet);
    } else {
      // If return vector is still empty because no neighbors are available,
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
  // Set default return id if initial lane has already no neighbors
  int id_neighbor_lanelet = -1;

  // Check if most outside lane is searched
  // Here: return current lanelet's ID when no neighbors are available
  if (recursiveness == -1) {
    id_neighbor_lanelet = id_initial_lanelet;
  }

  // Get ID of current lanelet's neighboring lanelet
  int id_neighbor_lanelet_tmp = lanelet_connections[id_initial_lanelet].neighbor_lanelet_ids[side];

  // Init counter to track current level of recursiveness in loop
  int recursiveness_counter = 0;

  // Loop as long as all neighbors of the initial lanelet have been searched, maximum iteration
  // depth: number_of_lanelets
  for (size_t i = 0; i < lanelet_connections.size(); i++) {
    // Break while loop if desired recursiveness is reached
    if (recursiveness != -1 && recursiveness_counter >= recursiveness) break;

    // Check if neighbor is available
    if (id_neighbor_lanelet_tmp >= 0) {
      // Break if navigation info is considered AND the lanelet does not lead
      // towards goal
      if (
        do_include_navigation_info &&
        !lanelet_connections[id_neighbor_lanelet_tmp].goal_information) {
        break;
      }

      // Store current neighbor lanelet ID
      id_neighbor_lanelet = id_neighbor_lanelet_tmp;
      // Query neighbor of neighbor and use as new start lanelet
      id_neighbor_lanelet_tmp =
        lanelet_connections[id_neighbor_lanelet_tmp].neighbor_lanelet_ids[side];

    } else {
      // Return -1 if lanelet ID at specific recursiveness is requested
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

  // Check if position is within one of the available lanelet
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

lanelet::BasicPoint2d RecenterGoalPoint(
  const lanelet::BasicPoint2d & goal_point, const std::vector<lanelet::Lanelet> & road_model)
{
  // Return value
  lanelet::BasicPoint2d projected_goal_point;

  // Get current lanelet index of goal point
  const int lanelet_idx_goal_point = FindOccupiedLaneletID(road_model, goal_point);

  if (lanelet_idx_goal_point >= 0) {
    // Get the centerline of the goal point's lanelet
    lanelet::ConstLineString2d centerline_curr_lanelet =
      road_model[lanelet_idx_goal_point].centerline2d();

    // Project goal point to the centerline of its lanelet
    projected_goal_point = lanelet::geometry::project(centerline_curr_lanelet, goal_point);
  } else {
    // Return untouched input point if index is not valid
    projected_goal_point = goal_point;
  }

  return projected_goal_point;
}

visualization_msgs::msg::MarkerArray CreateMarkerArray(
  const std::vector<lanelet::ConstLineString3d> & centerline,
  const std::vector<lanelet::ConstLineString3d> & left,
  const std::vector<lanelet::ConstLineString3d> & right,
  const autoware_planning_msgs::msg::RoadSegments & msg)
{
  visualization_msgs::msg::MarkerArray markerArray;

  // Centerline
  for (size_t i = 0; i < centerline.size(); ++i) {
    visualization_msgs::msg::Marker marker;  // Create a new marker in each iteration

    // Adding points to the marker
    for (const auto & point : centerline[i]) {
      geometry_msgs::msg::Point p;

      p.x = point.x();
      p.y = point.y();
      p.z = point.z();

      marker.points.push_back(p);
    }
    markerArray.markers.push_back(marker);
  }

  // Left bound
  for (size_t i = 0; i < left.size(); ++i) {
    visualization_msgs::msg::Marker marker;  // Create a new marker in each iteration

    marker.header.frame_id = msg.header.frame_id;  // Adjust frame_id as needed
    marker.header.stamp = msg.header.stamp;        // rclcpp::Node::now();
    marker.ns = "linestring";
    marker.id = i + 1000;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;  // Neutral orientation
    marker.scale.x = 5.0;
    marker.color.b = 1.0;  // Blue color
    marker.color.a = 1.0;  // Full opacity

    // Adding points to the marker
    for (const auto & point : left[i]) {
      geometry_msgs::msg::Point p;

      p.x = point.x();
      p.y = point.y();
      p.z = point.z();

      marker.points.push_back(p);
    }
    markerArray.markers.push_back(marker);
  }

  // Right bound
  for (size_t i = 0; i < right.size(); ++i) {
    visualization_msgs::msg::Marker marker;  // Create a new marker in each iteration

    marker.header.frame_id = msg.header.frame_id;  // Adjust frame_id as needed
    marker.header.stamp = msg.header.stamp;        // rclcpp::Node::now();
    marker.ns = "linestring";
    marker.id = i + 2000;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;  // Neutral orientation
    marker.scale.x = 5.0;
    marker.color.b = 1.0;  // Blue color
    marker.color.a = 1.0;  // Full opacity

    // Adding points to the marker
    for (const auto & point : right[i]) {
      geometry_msgs::msg::Point p;

      p.x = point.x();
      p.y = point.y();
      p.z = point.z();

      marker.points.push_back(p);
    }
    markerArray.markers.push_back(marker);
  }

  return markerArray;
}

autoware_planning_msgs::msg::DrivingCorridor CreateDrivingCorridor(
  const std::vector<int> & lane, const std::vector<lanelet::Lanelet> & converted_lanelets)
{
  // Create driving corridor
  autoware_planning_msgs::msg::DrivingCorridor driving_corridor;

  for (int id : lane) {
    if (id >= 0) {
      auto current_lanelet = converted_lanelets.at(id);

      auto centerline = current_lanelet.centerline();
      auto bound_left = current_lanelet.leftBound();
      auto bound_right = current_lanelet.rightBound();

      // Adding elements of centerline
      for (const auto & point : centerline) {
        geometry_msgs::msg::Point p;

        p.x = point.x();
        p.y = point.y();
        p.z = point.z();

        driving_corridor.centerline.push_back(p);
      }

      // Adding elements of bound_left
      for (const auto & point : bound_left) {
        geometry_msgs::msg::Point p;

        p.x = point.x();
        p.y = point.y();
        p.z = point.z();

        driving_corridor.bound_left.push_back(p);
      }

      // Adding elements of bound_right
      for (const auto & point : bound_right) {
        geometry_msgs::msg::Point p;

        p.x = point.x();
        p.y = point.y();
        p.z = point.z();

        driving_corridor.bound_right.push_back(p);
      }
    }
  }
  return driving_corridor;
}

lanelet::LineString2d CreateLineString(const std::vector<geometry_msgs::msg::Point> & points)
{
  // Create a Lanelet2 linestring
  lanelet::LineString2d linestring;

  // Iterate through the vector of points and add them to the linestring
  for (const auto & point : points) {
    lanelet::Point2d p(0, point.x,
                       point.y);  // First argument is ID (set it to 0 for now)
    linestring.push_back(p);
  }

  // Return the created linestring
  return linestring;
}

std::vector<int> GetAllNeighborsOfLane(
  const std::vector<int> & lane, const std::vector<LaneletConnection> & lanelet_connections,
  const int vehicle_side)
{
  // Initialize vector
  std::vector<int> neighbor_lane_idx = {};

  if (!lane.empty()) {
    // Loop through all the lane indices to get the neighbors
    int neighbor_tmp;

    for (const int id : lane) {
      neighbor_tmp = lanelet_connections[id].neighbor_lanelet_ids[vehicle_side];
      if (neighbor_tmp >= 0) {
        // Only add neighbor if lanelet does not exist already (avoid having
        // duplicates)
        if (
          std::find(neighbor_lane_idx.begin(), neighbor_lane_idx.end(), neighbor_tmp) ==
          neighbor_lane_idx.end()) {
          neighbor_lane_idx.push_back(neighbor_tmp);
        }
      } else {
        // If there is a blind spot in the neighbor sequence, break the loop
        break;
      }
    }
  }

  return neighbor_lane_idx;
}

void InsertPredecessorLanelet(
  std::vector<int> & lane_idx, const std::vector<LaneletConnection> & lanelet_connections)
{
  if (!lane_idx.empty()) {
    // Get index of first lanelet
    int first_lanelet_index = lane_idx[0];

    if (!lanelet_connections[first_lanelet_index].predecessor_lanelet_ids.empty()) {
      // Get one predecessor lanelet
      const int predecessor_lanelet = lanelet_connections[first_lanelet_index]
                                        .predecessor_lanelet_ids[0];  // Get one of the predecessors

      // Insert predecessor lanelet in lane_idx
      if (predecessor_lanelet >= 0) {
        lane_idx.insert(lane_idx.begin(), predecessor_lanelet);
      }
    }
  }
}

void CalculatePredecessors(std::vector<LaneletConnection> & lanelet_connections)
{
  // Determine predecessor information from already known information
  for (std::size_t id_lanelet = 0; id_lanelet < lanelet_connections.size(); id_lanelet++) {
    // Write lanelet predecessors (which are the successors of their previous
    // lanelets)
    for (std::size_t n_successor = 0;
         n_successor < lanelet_connections[id_lanelet].successor_lanelet_ids.size();
         n_successor++) {
      // Check if current lanelet has a valid successor, otherwise end of
      // current local environment model has been reached and all the
      // predecessors have been written with the last iteration
      if (lanelet_connections[id_lanelet].successor_lanelet_ids[n_successor] > -1) {
        lanelet_connections[lanelet_connections[id_lanelet].successor_lanelet_ids[n_successor]]
          .predecessor_lanelet_ids.push_back(id_lanelet);
      }
    }
  }

  // Write -1 to lanelets which have no predecessors
  for (LaneletConnection lanelet_connection : lanelet_connections) {
    if (lanelet_connection.predecessor_lanelet_ids.empty()) {
      lanelet_connection.predecessor_lanelet_ids = {-1};
    }
  }
}

}  // namespace autoware::mapless_architecture
