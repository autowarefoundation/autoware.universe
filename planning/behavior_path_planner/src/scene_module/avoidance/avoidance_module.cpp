// Copyright 2021 Tier IV, Inc.
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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"

#include "autoware_planning_msgs/msg/path_with_lane_id.hpp"
#include "autoware_utils/autoware_utils.hpp"
#include "lanelet2_extension/utility/message_conversion.hpp"
#include "lanelet2_extension/utility/utilities.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/scene_module/avoidance/avoidance_module.hpp"
#include "behavior_path_planner/scene_module/avoidance/debug.hpp"
#include "behavior_path_planner/utilities.hpp"

namespace
{
using behavior_path_planner::ObjectData;
using behavior_path_planner::PlannerData;
using geometry_msgs::msg::Pose;

bool isOnRight(const ObjectData & obj) {return obj.lateral < 0.0;}

bool isOnLeft(const ObjectData & obj) {return !isOnRight(obj);}

lanelet::ConstLanelets calcLaneAroundPose(
  const std::shared_ptr<const PlannerData> & planner_data, const Pose & pose,
  const double backward_length)
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!route_handler->getClosestLaneletWithinRoute(pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("avoidance"),
      "failed to find closest lanelet within route!!!");
    return {};  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes =
    route_handler->getLaneletSequence(current_lane, pose, backward_length, p.forward_path_length);

  return current_lanes;
}

}  // namespace

namespace behavior_path_planner
{
using autoware_utils::calcLateralDeviation;
using autoware_utils::calcSignedArcLength;
using autoware_utils::createPoint;
using autoware_utils::findNearestIndex;

AvoidanceModule::AvoidanceModule(
  const std::string & name, rclcpp::Node & node,
  const AvoidanceParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  approval_handler_.waitApproval();
}

bool AvoidanceModule::isExecutionRequested() const
{
  RCLCPP_DEBUG(getLogger(), "AVOIDANCE isExecutionRequested");

  if (current_state_ == BT::NodeStatus::RUNNING) {return true;}

  const auto avoid_data = calcAvoidancePlanningData();

  const bool has_avoidance_target = !avoid_data.objects.empty();
  return has_avoidance_target ? true : false;
}

bool AvoidanceModule::isExecutionReady() const
{
  RCLCPP_DEBUG(getLogger(), "AVOIDANCE isExecutionReady");

  if (current_state_ == BT::NodeStatus::RUNNING) {return true;}

  return true;
}

BT::NodeStatus AvoidanceModule::updateState()
{
  const auto is_plan_running = isAvoidancePlanRunning();

  const auto avoid_data = calcAvoidancePlanningData();
  const bool has_avoidance_target = !avoid_data.objects.empty();

  if (!is_plan_running && !has_avoidance_target) {
    current_state_ = BT::NodeStatus::SUCCESS;
  } else {
    current_state_ = BT::NodeStatus::RUNNING;
  }

  RCLCPP_DEBUG(
    getLogger(),
    "is_plan_running = %d, has_avoidance_target = %d", is_plan_running, has_avoidance_target);

  return current_state_;
}

AvoidancePlanningData AvoidanceModule::calcAvoidancePlanningData() const
{
  AvoidancePlanningData data;

  const auto reference_pose = prev_output_ ? getUnshiftedEgoPose(*prev_output_) : getEgoPose();
  data.reference_pose = reference_pose.pose;
  const auto center_path = calcCenterLinePath(planner_data_, reference_pose);

  // calculate reference path with spline
  constexpr double RESAMPLE_INTERVAL = 0.3;
  data.reference_path = util::resamplePathWithSpline(center_path, RESAMPLE_INTERVAL);

  // velocity filter: only for stopped vehicle
  const auto objects_candidate = util::filterObjectsByVelocity(
    *planner_data_->dynamic_object, parameters_.threshold_speed_object_is_stopped);

  // detection area filter
  const auto current_lanes = calcLaneAroundPose(
    planner_data_, reference_pose.pose, planner_data_->parameters.backward_path_length);
  data.current_lanelets = current_lanes;
  const auto expanded_lanelets = lanelet::utils::getExpandedLanelets(current_lanes, 1.0, 0.0);
  const auto lane_filtered_objects_index =
    util::filterObjectsByLanelets(objects_candidate, expanded_lanelets);

  RCLCPP_DEBUG(
    getLogger(), "dynamic_objects.size() = %lu", planner_data_->dynamic_object->objects.size());
  RCLCPP_DEBUG(getLogger(), "object_candidate.size() = %lu", objects_candidate.objects.size());
  RCLCPP_DEBUG(
    getLogger(), "lane_filtered_objects_index.size() = %lu", lane_filtered_objects_index.size());

  // for goal
  const bool is_goal_included =
    planner_data_->route_handler->isInGoalRouteSection(expanded_lanelets.back());

  // for filtered objects
  for (const auto & i : lane_filtered_objects_index) {
    const auto & object = objects_candidate.objects.at(i);
    const auto & object_pos = object.state.pose_covariance.pose.position;

    if (!isTargetObjectType(object)) {
      RCLCPP_DEBUG(getLogger(), "isTargetObjectType false. Ignore this object.");
      continue;
    }

    ObjectData object_data;
    object_data.object = object;

    // target object CoM in Frenet coordinate
    object_data.longitudinal =
      calcSignedArcLength(data.reference_path.points, getEgoPosition(), object_pos);

    // object is behind ego or too far.
    if (object_data.longitudinal < -parameters_.object_check_backward_distance) {
      RCLCPP_DEBUG(getLogger(), "object < -backward_distance. Ignore this object.");
      continue;
    }
    if (object_data.longitudinal > parameters_.object_check_forward_distance) {
      RCLCPP_DEBUG(getLogger(), "object > forward_distance. Ignore this object.");
      continue;
    }

    // object is ahead goal
    if (is_goal_included) {
      const auto goal_pose = planner_data_->route_handler->getGoalPose();
      const double goal_dist =
        calcSignedArcLength(data.reference_path.points, getEgoPosition(), goal_pose.position);
      if (object_data.longitudinal > goal_dist) {
        RCLCPP_DEBUG(getLogger(), "object > goal_distance. Ignore this object.");
        continue;
      }
    }

    // target object closest-footprint in Frenet coordinate
    const auto object_closest_index = findNearestIndex(data.reference_path.points, object_pos);
    const auto object_closest_pose = data.reference_path.points.at(object_closest_index).point.pose;
    object_data.lateral = calcLateralDeviation(object_closest_pose, object_pos);

    // object is on center
    if (std::abs(object_data.lateral) < parameters_.threshold_distance_object_is_on_center) {
      RCLCPP_DEBUG(getLogger(), "Object is on Center. Ignore this object.");
      continue;
    }

    // calc longest overhang distance
    double most_overhang_dist = isOnRight(object_data) ? -100.0 : 100.0;  // large number
    autoware_utils::Polygon2d object_poly{};
    util::calcObjectPolygon(object, &object_poly);
    for (const auto & p : object_poly.outer()) {
      const auto lat_dist =
        calcLateralDeviation(object_closest_pose, createPoint(p.x(), p.y(), 0.0));
      most_overhang_dist = isOnRight(object_data) ? std::max(most_overhang_dist, lat_dist) :
        std::min(most_overhang_dist, lat_dist);
    }
    object_data.overhang_dist = most_overhang_dist;

    RCLCPP_DEBUG(
      getLogger(),
      "set object_data: longitudinal = %f, lateral = %f, most_overhang_dist = %f",
      object_data.longitudinal, object_data.lateral, most_overhang_dist);

    // set data
    data.objects.push_back(object_data);
  }

  RCLCPP_DEBUG(getLogger(), "target object size = %lu", data.objects.size());

  {
    debug_data_.current_lanelets = std::make_shared<lanelet::ConstLanelets>(current_lanes);
    debug_data_.expanded_lanelets = std::make_shared<lanelet::ConstLanelets>(expanded_lanelets);
  }

  return data;
}

std::vector<ShiftPoint> AvoidanceModule::calcShiftPointsInFrenet(DebugData & debug) const
{
  /*
   * NOTE: this logic is under development, and will be cleaned up after logic fix.
   */

  const auto & reference = avoidance_data_.reference_path;
  const auto ego_closest_idx = findNearestIndex(reference.points, getEgoPosition());
  const auto arclength_v =
    util::calcPathArcLengthArray(reference, ego_closest_idx, reference.points.size() - 1);

  auto sorted_objects = avoidance_data_.objects;
  std::sort(
    sorted_objects.begin(), sorted_objects.end(), [](auto a, auto b) {
      return a.longitudinal < b.longitudinal;
    });

  std::vector<Frenet> shift_lengths;
  {
    // Search nearby objects, and take its shift length.
    const double same_shift_distance = 30.0;
    for (const auto & main_object : sorted_objects) {
      auto shift_length = main_object.overhang_dist;
      debug.raw_shift_points.emplace_back(shift_length, main_object.longitudinal);
      for (const auto & sub_object : sorted_objects) {
        // within threshold?
        if (std::abs(sub_object.longitudinal - main_object.longitudinal) > same_shift_distance) {
          continue;
        }

        // has same direction?
        if (sub_object.lateral * main_object.lateral < 0.0) {continue;}

        // then, take longer one.
        shift_length = isOnRight(main_object) ? std::max(shift_length, sub_object.overhang_dist) :
          std::min(shift_length, sub_object.overhang_dist);
      }
      shift_lengths.emplace_back(shift_length, main_object.longitudinal);
    }

    debug.modified_shift_points = shift_lengths;
  }

  for (const auto & sl : shift_lengths) {
    RCLCPP_DEBUG(getLogger(), "[RAW SHIFT LENGTH] lon = %f, lat = %f", sl.longitudinal, sl.lateral);
  }

  ////////////////////////////////////////////////////////////
  //////// Add lateral avoidance margin on Frenet ////////////
  ////////////////////////////////////////////////////////////
  const double avoid_margin =
    parameters_.lateral_collision_margin + 0.5 * planner_data_->parameters.vehicle_width;
  for (size_t i = 0; i < shift_lengths.size(); ++i) {
    shift_lengths.at(i).lateral += isOnRight(sorted_objects.at(i)) ? avoid_margin : -avoid_margin;
  }

  for (const auto & sl : shift_lengths) {
    RCLCPP_DEBUG(
      getLogger(), "[MARGINED SHIFT LENGTH] lon = %f, lat = %f", sl.longitudinal, sl.lateral);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// If the objects are empty, but still base_offset has a value, set a return-shift point  ///
  //////////////////////////////////////////////////////////////////////////////////////////////

  std::vector<ShiftPoint> shift_points;

  RCLCPP_DEBUG(
    getLogger(),
    "shift_lengths size = %lu, current_last_shift length = %f", shift_lengths.size(),
    path_shifter_.getLastShiftLength());
  if (shift_lengths.empty() && std::fabs(path_shifter_.getLastShiftLength()) > 0.01) {
    // needs to return to center path.

    const double dist_to_start = getNominalDistanceToStartAvoid();
    const double dist_to_end = getNominalAvoidanceDistance();
    const auto start_idx = util::getIdxByArclength(reference, getEgoPosition(), dist_to_start);
    const auto end_idx = util::getIdxByArclength(reference, getEgoPosition(), dist_to_end);

    ShiftPoint sp{};
    sp.length = 0.0;
    sp.start = reference.points.at(start_idx).point.pose;
    sp.end = reference.points.at(end_idx).point.pose;
    shift_points.push_back(sp);
    RCLCPP_DEBUG(
      getLogger(), "add return point: to_start = %f, to_end = %f", dist_to_start, dist_to_end);

    debug.new_shift_points = shift_points;
    return shift_points;
  }

  /////////////////////////////////////////////////////////////////////////
  //////////// generate shift point from Frenet shift distance ////////////
  /////////////////////////////////////////////////////////////////////////

  double current_shift = path_shifter_.getBaseOffset();

  for (size_t i = 0; i < shift_lengths.size(); ++i) {
    const auto frenet = shift_lengths.at(i);
    const double delta_shift = frenet.lateral - current_shift;
    RCLCPP_DEBUG(
      getLogger(),
      "frenet.lateral = %f, current_shift = %f, delta_shift = %f", frenet.lateral, current_shift,
      delta_shift);
    if (std::abs(delta_shift) < 0.5) {
      RCLCPP_DEBUG(
        getLogger(), "delta_shift = %f, it is small. Do NOT add this point.",
        delta_shift);
      continue;
    }

    ShiftPoint sp{};
    sp.length = frenet.lateral;
    const auto end_idx = util::getIdxByArclength(reference, getEgoPosition(), frenet.longitudinal);
    sp.end = reference.points.at(end_idx).point.pose;
    const double nominal_long_dist = std::max(
      parameters_.min_distance_avoiding,
      path_shifter_.calcLongitudinalDistFromJerk(
        delta_shift, parameters_.nominal_lateral_jerk, getEgoSpeed()));

    double dist_to_ego =
      autoware_utils::calcSignedArcLength(reference.points, getEgoPosition(), sp.end.position);
    double max_interval_dist = dist_to_ego;
    if (i != 0) {
      for (size_t j = i - 1; j > 0; --j) {
        const double diff = shift_lengths.at(j).lateral - shift_lengths.at(i).lateral;
        const bool j_has_larger_shift_than_i = (isOnRight(sorted_objects.at(i)) && diff > 0.5) ||
          (isOnLeft(sorted_objects.at(i)) && diff < -0.5);
        if (j_has_larger_shift_than_i) {
          max_interval_dist = shift_lengths.at(i).longitudinal - shift_lengths.at(j).longitudinal;
        }
      }
    }

    const double longitudinal = std::min({nominal_long_dist, dist_to_ego, max_interval_dist});
    const double dist_limit = path_shifter_.calcLongitudinalDistFromJerk(
      delta_shift, parameters_.max_lateral_jerk, getEgoSpeed());
    RCLCPP_DEBUG(
      getLogger(),
      "delta_shift = %f, nominal_long_dist = %f, dist_limit = %f, longitudinal = %f, vel = %f",
      delta_shift, nominal_long_dist, dist_limit, longitudinal, getEgoSpeed());

    // if lateral jerk is too high, not using this object as a target.
    if (longitudinal < dist_limit) {
      RCLCPP_DEBUG(getLogger(), "jerk is too large. not avoid this object");
      continue;
    }

    const auto start_idx = util::getIdxByArclength(reference, sp.end.position, -longitudinal);
    sp.start = reference.points.at(std::max(ego_closest_idx, start_idx)).point.pose;
    shift_points.push_back(sp);
    RCLCPP_DEBUG(
      getLogger(),
      "[NEW SHIFT POINT] start_idx = %lu, end_idx = %lu, shift_length = %f, nominal_long_dist = "
      "%f, dist_to_ego = %f, max_interval_dist = %f, longitudinal = %f",
      start_idx, end_idx, sp.length, nominal_long_dist, dist_to_ego, max_interval_dist,
      longitudinal);

    current_shift = frenet.lateral;
  }
  RCLCPP_DEBUG(getLogger(), "shift_points size = %lu", shift_points.size());

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /////////////// If there is no obstacle around path edge, set a return-shift point ///////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////

  if (!shift_lengths.empty()) {  // if empty, the return-shift point was taken care of in above.
    if (std::abs(shift_lengths.back().lateral) > 0.01) {  // take back, assuming it is sorted.
      const auto dist_ego_to_path_end = autoware_utils::calcSignedArcLength(
        reference.points, getEgoPosition(), reference.points.size() - 1);

      // TODO(Horibe) think the length later
      if (dist_ego_to_path_end - shift_lengths.back().longitudinal > 50.0) {
        const double dist_to_start = shift_lengths.back().longitudinal;
        const double dist_to_end =
          shift_lengths.back().longitudinal + getNominalAvoidanceDistance();
        const auto start_idx = util::getIdxByArclength(reference, getEgoPosition(), dist_to_start);
        const auto end_idx = util::getIdxByArclength(reference, getEgoPosition(), dist_to_end);

        ShiftPoint sp{};
        sp.length = 0.0;
        sp.start = reference.points.at(start_idx).point.pose;
        sp.end = reference.points.at(end_idx).point.pose;
        shift_points.push_back(sp);
        RCLCPP_DEBUG(
          getLogger(), "add return point: to_start = %f, to_end = %f", dist_to_start,
          dist_to_end);
      }
    }
  }
  RCLCPP_DEBUG(getLogger(), "shift_points size = %lu", shift_points.size());

  ////////////////////////////////////////////////////////////////////////////
  //////////////// If there is an intersection, insert. //////////////////////
  ////////////////////////////////////////////////////////////////////////////
  constexpr bool use_intersection = false;  // TODO(Horibe) enable later
  if (use_intersection) {
    const auto intersection_shift_point = calcIntersectionShiftPoint(avoidance_data_);
    if (intersection_shift_point) {
      RCLCPP_ERROR(getLogger(), "ADD INTERSECTION point!!!");
      shift_points.push_back(*intersection_shift_point);
    } else {
      RCLCPP_ERROR(getLogger(), "NO intersection point found");
    }
  }

  debug.new_shift_points = shift_points;
  return shift_points;
}

std::vector<Frenet> AvoidanceModule::endPointsToFrenet(
  const PathWithLaneId & path, const PathShifter & shifter) const
{
  std::vector<Frenet> frenet_points;
  double current_lateral = shifter.getBaseOffset();
  for (const auto & p : shifter.getShiftPoints()) {
    Frenet frenet;
    frenet.longitudinal =
      autoware_utils::calcSignedArcLength(path.points, getEgoPosition(), p.end.position);
    current_lateral += p.length;
    frenet.lateral = current_lateral;
    frenet_points.push_back(frenet);
  }

  return frenet_points;
}

void AvoidanceModule::extendDrivableArea(ShiftedPath * shifted_path, double margin) const
{
  const auto right_extend_elem =
    std::min_element(shifted_path->shift_length.begin(), shifted_path->shift_length.end());
  const auto left_extend_elem =
    std::max_element(shifted_path->shift_length.begin(), shifted_path->shift_length.end());

  double right_extend = std::min(*right_extend_elem, 0.0);
  double left_extend = std::max(*left_extend_elem, 0.0);

  constexpr double THRESHOLD = 0.01;
  right_extend -= (right_extend < -THRESHOLD) ? margin : 0.0;
  left_extend += (left_extend > THRESHOLD) ? margin : 0.0;

  const double resolution = shifted_path->path.drivable_area.info.resolution;
  const double width = shifted_path->path.drivable_area.info.width * resolution;
  const double height = shifted_path->path.drivable_area.info.height * resolution;
  const double vehicle_length = planner_data_->parameters.vehicle_length;

  const auto extended_lanelets = lanelet::utils::getExpandedLanelets(
    avoidance_data_.current_lanelets, left_extend, right_extend);

  shifted_path->path.drivable_area = util::generateDrivableArea(
    extended_lanelets, getEgoPose(), width, height, resolution, vehicle_length,
    *(planner_data_->route_handler));
}

PoseStamped AvoidanceModule::getUnshiftedEgoPose(const ShiftedPath & prev_path) const
{
  const auto ego_pose = getEgoPose();

  // un-shifted fot current ideal pose
  const auto closest = findNearestIndex(prev_path.path.points, ego_pose.pose.position);

  PoseStamped unshifted_pose{};
  unshifted_pose.header = ego_pose.header;
  unshifted_pose.pose = prev_path.path.points.at(closest).point.pose;

  util::shiftPose(&unshifted_pose.pose, -prev_path.shift_length.at(closest));

  return unshifted_pose;
}

// TODO(Horibe) clean up functions: there is a similar code in util as well.
PathWithLaneId AvoidanceModule::calcCenterLinePath(
  const std::shared_ptr<const PlannerData> & planner_data,
  const PoseStamped & pose) const
{
  const auto & p = planner_data->parameters;
  const auto & route_handler = planner_data->route_handler;

  PathWithLaneId centerline_path;

  // special for avoidance: take behind distance upt ot shift-start-point if it exist.
  const auto longest_dist_to_shift_point = [&]() {
      double max_dist = 0.0;
      for (const auto & pnt : path_shifter_.getShiftPoints()) {
        max_dist = std::max(max_dist, autoware_utils::calcDistance2d(getEgoPose(), pnt.start));
      }
      return max_dist;
    } ();
  const auto extra_margin = 10.0;  // Since distance does not consider arclength, but just line.
  const auto backward_length =
    std::max(p.backward_path_length, longest_dist_to_shift_point + extra_margin);

  RCLCPP_DEBUG(
    getLogger(),
    "p.backward_path_length = %f, longest_dist_to_shift_point = %f, backward_length = %f",
    p.backward_path_length, longest_dist_to_shift_point, backward_length);

  const lanelet::ConstLanelets current_lanes =
    calcLaneAroundPose(planner_data, pose.pose, backward_length);
  centerline_path = route_handler->getCenterLinePath(
    current_lanes, pose.pose, backward_length, p.forward_path_length, p);

  // for debug: check if the path backward distance is same as the desired length.
  // {
  //   const auto back_to_ego = autoware_utils::calcSignedArcLength(
  //     centerline_path.points, centerline_path.points.front().point.pose.position,
  //     getEgoPosition());
  //   RCLCPP_INFO(getLogger(), "actual back_to_ego distance = %f", back_to_ego);
  // }

  centerline_path.header = route_handler->getRouteHeader();

  centerline_path.drivable_area = util::generateDrivableArea(
    current_lanes, pose, p.drivable_area_width, p.drivable_area_height, p.drivable_area_resolution,
    p.vehicle_length, *route_handler);

  return centerline_path;
}

boost::optional<ShiftPoint> AvoidanceModule::calcIntersectionShiftPoint(
  const AvoidancePlanningData & data) const
{
  boost::optional<PathPointWithLaneId> intersection_point{};
  for (const auto & p : avoidance_data_.reference_path.points) {
    for (const auto & id : p.lane_ids) {
      const lanelet::ConstLanelet ll = planner_data_->route_handler->getLaneletsFromId(id);
      std::string turn_direction = ll.attributeOr("turn_direction", "else");
      if (turn_direction == "right" || turn_direction == "left") {
        intersection_point = p;
        RCLCPP_INFO(getLogger(), "intersection is found.");
        break;
      }
    }
    if (intersection_point) {break;}
  }

  const auto calcBehindPose = [&data](const Point & p, const double dist) {
      const auto & path = data.reference_path;
      const size_t start = autoware_utils::findNearestIndex(path.points, p);
      double sum = 0.0;
      for (size_t i = start - 1; i > 1; --i) {
        sum += autoware_utils::calcDistance2d(path.points.at(i), path.points.at(i + 1));
        if (sum > dist) {
          return path.points.at(i).point.pose;
        }
      }
      return path.points.at(0).point.pose;
    };

  const auto intersection_shift_point = [&]() {
      boost::optional<ShiftPoint> shift_point{};
      if (!intersection_point) {
        RCLCPP_INFO(getLogger(), "No intersection.");
        return shift_point;
      }

      const double ego_to_intersection_dist = autoware_utils::calcSignedArcLength(
        data.reference_path.points, getEgoPosition(), intersection_point->point.pose.position);

      if (ego_to_intersection_dist <= 5.0) {
        RCLCPP_INFO(getLogger(), "No enough margin to intersection.");
        return shift_point;
      }

      // Search obstacles around the intersection.
      // If it exists, use its shift distance on the intersection.
      constexpr double intersection_obstacle_check_dist = 10.0;
      constexpr double intersection_shift_margin = 1.0;

      double shift_length = 0.0;  // default (no obstacle) is zero.
      for (const auto & obj : avoidance_data_.objects) {
        if (std::abs(obj.longitudinal - ego_to_intersection_dist) >
          intersection_obstacle_check_dist)
        {
          continue;
        }
        if (isOnRight(obj)) {
          continue;                  // TODO(Horibe) Now only think about the left side obstacle.
        }
        shift_length = std::min(shift_length, obj.overhang_dist - intersection_shift_margin);
      }
      RCLCPP_INFO(getLogger(), "Intersection shift_length = %f", shift_length);

      ShiftPoint p{};
      p.length = shift_length;
      p.start =
        calcBehindPose(intersection_point->point.pose.position, intersection_obstacle_check_dist);
      p.end = intersection_point->point.pose;
      shift_point = p;
      return shift_point;
    } ();

  return intersection_shift_point;
}

BehaviorModuleOutput AvoidanceModule::plan()
{
  RCLCPP_DEBUG(getLogger(), "AVOIDANCE plan");

  const auto shift_points = calcShiftPointsInFrenet(debug_data_);

  const auto new_shift_point = findNewShiftPoint(shift_points, path_shifter_);
  if (new_shift_point) {addShiftPointIfApproved(*new_shift_point);}

  // generate path with shift points that have been inserted.
  const auto avoidance_path = generateAvoidancePath(path_shifter_);

  // post processing
  {
    postProcess(path_shifter_);  // remove old shift points
    prev_output_ = std::make_shared<ShiftedPath>(avoidance_path);
    setDebugData(path_shifter_, debug_data_);
  }

  BehaviorModuleOutput output;
  output.path = std::make_shared<PathWithLaneId>(avoidance_path.path);
  output.turn_signal_info = calcTurnSignalInfo(avoidance_path);

  clipPathLength(*output.path);

  RCLCPP_DEBUG(
    getLogger(), "exit plan(): set prev output (back().lat = %f)",
    prev_output_->shift_length.back());

  return output;
}

PathWithLaneId AvoidanceModule::planCandidate() const
{
  RCLCPP_DEBUG(getLogger(), "AVOIDANCE planCandidate start");

  auto path_shifter = path_shifter_;
  auto debug_data = debug_data_;

  const auto shift_points = calcShiftPointsInFrenet(debug_data);
  const auto new_shift_point = findNewShiftPoint(shift_points, path_shifter);
  if (new_shift_point) {path_shifter.addShiftPoint(*new_shift_point);}

  const auto shifted_path = generateAvoidancePath(path_shifter);

  return shifted_path.path;
}

BehaviorModuleOutput AvoidanceModule::planWaitingApproval()
{
  // we can execute the plan() since it handles the approval appropriately.
  BehaviorModuleOutput out = plan();
  out.path_candidate = std::make_shared<PathWithLaneId>(planCandidate());
  return out;
}

void AvoidanceModule::addShiftPointIfApproved(const ShiftPoint & shift_point)
{
  if (approval_handler_.isApproved()) {
    RCLCPP_DEBUG(getLogger(), "We want to add this shift point, and approved. ADD SHIFT POINT!");
    const size_t prev_size = path_shifter_.getShiftPointsSize();
    path_shifter_.addShiftPoint(shift_point);
    RCLCPP_DEBUG(
      getLogger(), "shift_point size: %lu -> %lu", prev_size, path_shifter_.getShiftPointsSize());

    // use this approval.
    approval_handler_.clearApproval();  // TODO(Horibe) will be fixed with service-call?
  } else {
    RCLCPP_DEBUG(getLogger(), "We want to add this shift point, but NOT approved. waiting...");
    approval_handler_.waitApproval();
  }
}

boost::optional<ShiftPoint> AvoidanceModule::findNewShiftPoint(
  const std::vector<ShiftPoint> & points, const PathShifter & shifter) const
{
  if (points.empty()) {
    RCLCPP_DEBUG(getLogger(), "shift points candidate is empty. return None.");
    return {};
  }

  if (shifter.getShiftPoints().empty()) {
    RCLCPP_DEBUG(getLogger(), "current shift point is empty. The first point should be NEW one.");
    return points.front();
  }

  for (const auto & point_candidate : points) {
    bool we_should_add_this = true;
    constexpr double shift_threshold_m = 1.0;         // length [m]
    constexpr double longitudinal_threshold_m = 5.0;  // length [m]
    for (const auto & current_point : shifter.getShiftPoints()) {
      const auto shift_length_diff = std::abs(point_candidate.length - current_point.length);
      const auto dist = autoware_utils::calcSignedArcLength(
        avoidance_data_.reference_path.points, point_candidate.end.position,
        current_point.end.position);

      if (shift_length_diff < shift_threshold_m && std::abs(dist) < longitudinal_threshold_m) {
        RCLCPP_DEBUG(getLogger(), "A similar shift point found. This is NOT new.");
        we_should_add_this = false;
        break;
      }
    }

    if (we_should_add_this) {
      RCLCPP_DEBUG(getLogger(), "No similar shift point found. This is NEW!");
      return point_candidate;
    }
  }

  RCLCPP_DEBUG(getLogger(), "No new shift point found. return None.");
  return {};
}

double AvoidanceModule::getNominalAvoidanceDistance() const
{
  const auto & p = parameters_;
  const auto epsilon_m = 0.1;  // for floating error to pass "has_enough_distance" check.
  const auto total_time =
    p.time_to_start_avoidance + p.time_avoidance_end_to_object + p.time_avoiding;
  const auto min_distance = p.min_distance_to_start_avoidance +
    p.min_distance_avoidance_end_to_object + p.min_distance_avoiding;
  const auto nominal_total_distance = std::max(getEgoSpeed() * total_time, min_distance);
  return nominal_total_distance + epsilon_m;
}

double AvoidanceModule::getNominalDistanceToStartAvoid() const
{
  const auto & p = parameters_;
  const auto epsilon_m = 0.1;  // for floating error to pass "has_enough_distance" check.
  const auto nominal_distance =
    std::max(getEgoSpeed() * p.time_to_start_avoidance, p.min_distance_to_start_avoidance);
  return nominal_distance + epsilon_m;
}

ShiftedPath AvoidanceModule::generateAvoidancePath(PathShifter & path_shifter) const
{
  const auto & reference = avoidance_data_.reference_path;

  ShiftedPath shifted_path;
  if (!path_shifter.generate(&shifted_path)) {
    RCLCPP_ERROR(getLogger(), "failed to generate shifted path.");
    return toShiftedPath(reference);
  }

  // Drivable area.
  constexpr double extend_margin = 0.5;
  extendDrivableArea(&shifted_path, extend_margin);

  return shifted_path;
}

void AvoidanceModule::postProcess(PathShifter & path_shifter) const
{
  path_shifter.removeBehindShiftPointAndSetBaseOffset(getEgoPosition());
}

void AvoidanceModule::updateData()
{
  debug_data_ = DebugData();
  avoidance_data_ = calcAvoidancePlanningData();
  path_shifter_.setPath(avoidance_data_.reference_path);
}

void AvoidanceModule::onEntry()
{
  RCLCPP_DEBUG(getLogger(), "AVOIDANCE onEntry. wait approval!");
  initVariables();
  current_state_ = BT::NodeStatus::SUCCESS;
  approval_handler_.waitApproval();
}

void AvoidanceModule::onExit()
{
  RCLCPP_DEBUG(getLogger(), "AVOIDANCE onExit");
  initVariables();
  current_state_ = BT::NodeStatus::IDLE;
  approval_handler_.clearWaitApproval();
}

void AvoidanceModule::setParameters(const AvoidanceParameters & parameters)
{
  parameters_ = parameters;
}

void AvoidanceModule::initVariables()
{
  prev_output_ = nullptr;
  path_shifter_ = PathShifter{};
}

void AvoidanceModule::clipPathLength(PathWithLaneId & path) const
{
  const double forward = planner_data_->parameters.forward_path_length;
  const double backward = planner_data_->parameters.backward_path_length;

  const auto start_idx = util::getIdxByArclength(path, getEgoPosition(), -backward);
  const auto end_idx = util::getIdxByArclength(path, getEgoPosition(), forward);

  const std::vector<PathPointWithLaneId> clipped_points{path.points.begin() + start_idx,
    path.points.begin() + end_idx};

  path.points = clipped_points;
}

ShiftedPath AvoidanceModule::toShiftedPath(const PathWithLaneId & path) const
{
  ShiftedPath out;
  out.path = path;
  out.shift_length.resize(path.points.size());
  std::fill(out.shift_length.begin(), out.shift_length.end(), 0.0);
  return out;
}

bool AvoidanceModule::isTargetObjectType(
  const DynamicObject & object) const
{
  using autoware_perception_msgs::msg::Semantic;
  const auto type = object.semantic.type;
  if (type == Semantic::CAR || type == Semantic::TRUCK || type == Semantic::BUS) {
    return true;
  }
  return false;
}

void AvoidanceModule::setDebugData(const PathShifter & shifter, const DebugData & debug)
{
  using marker_utils::createFrenetPointMarkerArray;
  using marker_utils::createLaneletsAreaMarkerArray;
  using marker_utils::createObjectsMarkerArray;
  using marker_utils::createShiftPointMarkerArray;

  debug_marker_.markers.clear();

  const auto add = [this](const MarkerArray & added) {
      autoware_utils::appendMarkerArray(added, &debug_marker_);
    };

  const auto & path = avoidance_data_.reference_path;
  const auto & base_offset = shifter.getBaseOffset();

  add(createLaneletsAreaMarkerArray(*debug.current_lanelets, "current_lanes", 0.0, 1.0, 0.0));
  add(createLaneletsAreaMarkerArray(*debug.expanded_lanelets, "expanded_lanelet", 0.8, 0.8, 0.0));
  add(createObjectsMarkerArray(debug.target_objects, "avoidance_object", 0, 0.5, 0.5, 0.0));
  add(
    createFrenetPointMarkerArray(
      debug.raw_shift_points, path, getEgoPosition(), "raw_shift_point", 0, 0.9, 0.9));
  add(
    createFrenetPointMarkerArray(
      debug.modified_shift_points, path, getEgoPosition(), "modified_shift_point", 0.9, 0., 0.9));
  add(
    createShiftPointMarkerArray(
      shifter.getShiftPoints(), base_offset, "current_shift_point", 0.9, 0.9, 0.0));
  add(
    createShiftPointMarkerArray(
      debug.new_shift_points, base_offset, "new_shift_point", 0.9, 0.0, 0.0));
}

TurnSignalInfo AvoidanceModule::calcTurnSignalInfo(const ShiftedPath & path) const
{
  TurnSignalInfo turn_signal;

  const auto shift_points = path_shifter_.getShiftPoints();
  if (shift_points.empty()) {return {};}

  const auto latest_shift_point = shift_points.front();  // assuming it is sorted.

  // Set turn signal if the shift length is larger than threshold.
  // TODO(Horibe) Turn signal should be turned on only when the vehicle across the lane.
  const auto tl_on_threshold = 0.3;  // [m]
  if (!path.shift_length.empty()) {
    if (isAvoidancePlanRunning()) {
      const double diff = path.shift_length.at(latest_shift_point.end_idx) -
        path.shift_length.at(latest_shift_point.start_idx);
      // RCLCPP_INFO(getLogger(), "diff = %f", diff);
      if (diff > tl_on_threshold) {
        turn_signal.turn_signal.data = TurnSignal::LEFT;
      } else if (diff < -tl_on_threshold) {
        turn_signal.turn_signal.data = TurnSignal::RIGHT;
      }
    }
  }

  // calc distance from ego to latest_shift_point end point.
  {
    const double distance = autoware_utils::calcSignedArcLength(
      path.path.points, getEgoPosition(), latest_shift_point.end.position) -
      planner_data_->parameters.base_link2front;
    // RCLCPP_INFO(getLogger(), "distance = %f", distance);
    if (distance >= 0.0) {
      turn_signal.signal_distance = distance;
    }
  }

  return turn_signal;
}

}  // namespace behavior_path_planner
