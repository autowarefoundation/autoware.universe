/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <scene_module/detection_area/scene.h>

#include <tf2_eigen/tf2_eigen.h>

#include <utilization/util.h>

namespace
{
double calcSignedDistance(const geometry_msgs::Pose & p1, const Eigen::Vector2d & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  auto basecoords_p2 = map2p1 * Eigen::Vector3d(p2.x(), p2.y(), p1.position.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}
}  // namespace

namespace bg = boost::geometry;

DetectionAreaModule::DetectionAreaModule(
  const int64_t module_id, const lanelet::autoware::DetectionArea & detection_area_reg_elem,
  const PlannerParam & planner_param)
: SceneModuleInterface(module_id),
  detection_area_reg_elem_(detection_area_reg_elem),
  state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool DetectionAreaModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path, autoware_planning_msgs::StopReason * stop_reason)
{
  const auto input_path = *path;

  debug_data_ = {};
  debug_data_.base_link2front = planner_data_->base_link2front;
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::StopReason::DETECTION_AREA);

  if (state_ == State::PASS) {
    return true;
  }

  // get stop line and detection_area
  lanelet::ConstLineString3d lanelet_stop_line = detection_area_reg_elem_.stopLine();
  lanelet::ConstPolygons3d detection_areas = detection_area_reg_elem_.detectionAreas();

  // get pointcloud
  const auto no_ground_pointcloud_ptr = planner_data_->no_ground_pointcloud;

  if (!isPointsWithinDetectionArea(no_ground_pointcloud_ptr, detection_areas)) {
    return true;
  }

  // get vehicle info and compute pass_judge_line_distance
  geometry_msgs::TwistStamped::ConstPtr self_twist_ptr = planner_data_->current_velocity;

  const double pass_judge_line_distance =
    planning_utils::calcJudgeLineDist(self_twist_ptr->twist.linear.x);

  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;

  // insert stop point
  for (size_t stop_line_id = 0; stop_line_id < lanelet_stop_line.size() - 1; ++stop_line_id) {
    const Line stop_line = {
      {lanelet_stop_line[stop_line_id].x(), lanelet_stop_line[stop_line_id].y()},
      {lanelet_stop_line[stop_line_id + 1].x(), lanelet_stop_line[stop_line_id + 1].y()}};
    Eigen::Vector2d stop_line_point;
    size_t stop_line_point_idx;
    if (!createTargetPoint(
          input_path, stop_line, planner_param_.stop_margin, stop_line_point_idx,
          stop_line_point)) {
      continue;
    }

    if (
      state_ != State::STOP &&
      calcSignedDistance(self_pose.pose, stop_line_point) < pass_judge_line_distance) {
      ROS_WARN_THROTTLE(1.0, "[detection_area] vehicle is over stop border");
      state_ = State::PASS;
      return true;
    }

    // Add Stop WayPoint
    if (!insertTargetVelocityPoint(input_path, stop_line, planner_param_.stop_margin, 0.0, *path)) {
      continue;
    }

    state_ = State::STOP;
    /* get stop point and stop factor */
    autoware_planning_msgs::StopFactor stop_factor;
    stop_factor.stop_pose = debug_data_.first_stop_pose;
    stop_factor.stop_factor_points = debug_data_.detection_points;
    planning_utils::appendStopReason(stop_factor, stop_reason);

    return true;
  }
  return false;
}

bool DetectionAreaModule::isPointsWithinDetectionArea(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  const lanelet::ConstPolygons3d & detection_areas)
{
  for (const auto & da : detection_areas) {
    for (size_t i = 0; i < no_ground_pointcloud_ptr->size(); ++i) {
      // create polygon
      Polygon polygon;
      const auto da_polygon = da.basicPolygon();
      for (const auto & da_point : da_polygon) {
        polygon.outer().push_back(bg::make<Point>(da_point.x(), da_point.y()));
      }
      polygon.outer().push_back(polygon.outer().front());

      Point point(no_ground_pointcloud_ptr->at(i).x, no_ground_pointcloud_ptr->at(i).y);
      if (bg::within(point, polygon)) {
        debug_data_.detection_points.emplace_back(
          planning_utils::toRosPoint(no_ground_pointcloud_ptr->at(i)));
        return true;
      }
    }
  }
  return false;
}

bool DetectionAreaModule::insertTargetVelocityPoint(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
  const double & margin, const double & velocity, autoware_planning_msgs::PathWithLaneId & output)
{
  // create target point
  Eigen::Vector2d target_point;
  size_t insert_target_point_idx;
  autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;

  if (!createTargetPoint(input, stop_line, margin, insert_target_point_idx, target_point))
    return false;
  const int target_velocity_point_idx = std::max(static_cast<int>(insert_target_point_idx - 1), 0);
  target_point_with_lane_id = output.points.at(target_velocity_point_idx);
  target_point_with_lane_id.point.pose.position.x = target_point.x();
  target_point_with_lane_id.point.pose.position.y = target_point.y();
  target_point_with_lane_id.point.twist.linear.x = velocity;
  output = input;

  // insert target point
  output.points.insert(output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

  // insert 0 velocity after target point
  for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
    output.points.at(j).point.twist.linear.x =
      std::min(velocity, output.points.at(j).point.twist.linear.x);
  if (velocity == 0.0 && target_velocity_point_idx < first_stop_path_point_index_) {
    debug_data_.first_stop_pose = target_point_with_lane_id.point.pose;
    first_stop_path_point_index_ = target_velocity_point_idx;
  }
  // -- debug code --
  if (velocity == 0.0) debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);
  // ----------------
  return true;
}

bool DetectionAreaModule::createTargetPoint(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &
    stop_line,
  const double & margin, size_t & target_point_idx, Eigen::Vector2d & target_point)
{
  for (size_t i = 0; i < input.points.size() - 1; ++i) {
    Line path_line = {
      {input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y},
      {input.points.at(i + 1).point.pose.position.x, input.points.at(i + 1).point.pose.position.y}};
    std::vector<Point> collision_points;
    bg::intersection(stop_line, path_line, collision_points);

    if (collision_points.empty()) continue;

    // check nearest collision point
    Point nearest_collision_point;
    double min_dist;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(
        Point(input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y),
        collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
      }
    }

    // search target point index
    target_point_idx = 0;
    const double base_link2front = planner_data_->base_link2front;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    if (0 <= target_length) {
      point1 << nearest_collision_point.x(), nearest_collision_point.y();
      point2 << input.points.at(i).point.pose.position.x, input.points.at(i).point.pose.position.y;
      length_sum += (point2 - point1).norm();
      for (size_t j = i; 0 < j; --j) {
        if (target_length < length_sum) {
          target_point_idx = j + 1;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j - 1).point.pose.position.x,
          input.points.at(j - 1).point.pose.position.y;
        length_sum += (point2 - point1).norm();
      }
    } else {
      point1 << nearest_collision_point.x(), nearest_collision_point.y();
      point2 << input.points.at(i + 1).point.pose.position.x,
        input.points.at(i + 1).point.pose.position.y;
      length_sum -= (point2 - point1).norm();
      for (size_t j = i + 1; j < input.points.size() - 1; ++j) {
        if (length_sum < target_length) {
          target_point_idx = j;
          break;
        }
        point1 << input.points.at(j).point.pose.position.x,
          input.points.at(j).point.pose.position.y;
        point2 << input.points.at(j + 1).point.pose.position.x,
          input.points.at(j + 1).point.pose.position.y;
        length_sum -= (point2 - point1).norm();
      }
    }
    // create target point
    getBackwordPointFromBasePoint(
      point2, point1, point2, std::fabs(length_sum - target_length), target_point);
    return true;
  }
  return false;
}

bool DetectionAreaModule::getBackwordPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}
