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
std::pair<int, double> findWayPointAndDistance(
  const autoware_planning_msgs::PathWithLaneId & input_path, const Eigen::Vector2d & p)
{
  constexpr double max_lateral_dist = 3.0;
  for (size_t i = 0; i < input_path.points.size() - 1; ++i) {
    const double dx = p.x() - input_path.points.at(i).point.pose.position.x;
    const double dy = p.y() - input_path.points.at(i).point.pose.position.y;
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;

    const double theta = std::atan2(dy, dx) - std::atan2(dy_wp, dx_wp);

    const double dist = std::hypot(dx, dy);
    const double dist_wp = std::hypot(dx_wp, dy_wp);

    // check lateral distance
    if (std::fabs(dist * std::sin(theta)) > max_lateral_dist) {
      continue;
    }

    // if the point p is back of the way point, return negative distance
    if (dist * std::cos(theta) < 0) {
      return std::make_pair(static_cast<int>(i), -1.0 * dist);
    }

    if (dist * std::cos(theta) < dist_wp) {
      return std::make_pair(static_cast<int>(i), dist);
    }
  }

  // if the way point is not found, return negative distance from the way point at 0
  const double dx = p.x() - input_path.points.at(0).point.pose.position.x;
  const double dy = p.y() - input_path.points.at(0).point.pose.position.y;
  return std::make_pair(-1, -1.0 * std::hypot(dx, dy));
}

double calcArcLengthFromWayPoint(
  const autoware_planning_msgs::PathWithLaneId & input_path, const int & src, const int & dst)
{
  double length = 0;
  const size_t src_idx = src >= 0 ? static_cast<size_t>(src) : 0;
  const size_t dst_idx = dst >= 0 ? static_cast<size_t>(dst) : 0;
  for (size_t i = src_idx; i < dst_idx; ++i) {
    const double dx_wp = input_path.points.at(i + 1).point.pose.position.x -
                         input_path.points.at(i).point.pose.position.x;
    const double dy_wp = input_path.points.at(i + 1).point.pose.position.y -
                         input_path.points.at(i).point.pose.position.y;
    length += std::hypot(dx_wp, dy_wp);
  }
  return length;
}

double calcSignedArcLength(
  const autoware_planning_msgs::PathWithLaneId & input_path, const geometry_msgs::Pose & p1,
  const Eigen::Vector2d & p2)
{
  std::pair<int, double> src =
    findWayPointAndDistance(input_path, Eigen::Vector2d(p1.position.x, p1.position.y));
  std::pair<int, double> dst = findWayPointAndDistance(input_path, p2);
  if (dst.first == -1) {
    double dx = p1.position.x - p2.x();
    double dy = p1.position.y - p2.y();
    return -1.0 * std::hypot(dx, dy);
  }

  if (src.first < dst.first) {
    return calcArcLengthFromWayPoint(input_path, src.first, dst.first) - src.second + dst.second;
  } else if (src.first > dst.first) {
    return -1.0 *
           (calcArcLengthFromWayPoint(input_path, dst.first, src.first) - dst.second + src.second);
  } else {
    return dst.second - src.second;
  }
}

double calcSignedDistance(const geometry_msgs::Pose & p1, const Eigen::Vector2d & p2)
{
  Eigen::Affine3d map2p1;
  tf2::fromMsg(p1, map2p1);
  auto basecoords_p2 = map2p1.inverse() * Eigen::Vector3d(p2.x(), p2.y(), p1.position.z);
  return basecoords_p2.x() >= 0 ? basecoords_p2.norm() : -basecoords_p2.norm();
}
}  // namespace

namespace bg = boost::geometry;

DetectionAreaModule::DetectionAreaModule(
  const int64_t module_id, const lanelet::autoware::DetectionArea & detection_area_reg_elem,
  const PlannerParam & planner_param)
: SceneModuleInterface(module_id),
  module_id_(module_id),
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
  const double max_acc = planner_data_->max_stop_acceleration_threshold_;
  const double delay_response_time = planner_data_->delay_response_time_;
  const double pass_judge_line_distance =
    planning_utils::calcJudgeLineDist(self_twist_ptr->twist.linear.x, max_acc, delay_response_time);

  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;

  // insert stop point
  for (size_t stop_line_id = 0; stop_line_id < lanelet_stop_line.size() - 1; ++stop_line_id) {
    const Line stop_line = {
      {lanelet_stop_line[stop_line_id].x(), lanelet_stop_line[stop_line_id].y()},
      {lanelet_stop_line[stop_line_id + 1].x(), lanelet_stop_line[stop_line_id + 1].y()}};

    // Check Dead Line
    {
      constexpr double dead_line_range = 5.0;
      Eigen::Vector2d dead_line_point;
      size_t dead_line_point_idx;
      if (!createTargetPoint(
            input_path, stop_line, -2.0 /*overline margin*/, dead_line_point_idx,
            dead_line_point)) {
        continue;
      }

      if (isOverDeadLine(
            self_pose.pose, input_path, dead_line_point_idx, dead_line_point, dead_line_range)) {
        state_ = State::PASS;
        return true;
      }
    }

    // Check Stop Line
    {
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

bool DetectionAreaModule::isOverDeadLine(
  const geometry_msgs::Pose & self_pose, const autoware_planning_msgs::PathWithLaneId & input_path,
  const size_t & dead_line_point_idx, const Eigen::Vector2d & dead_line_point,
  const double dead_line_range)
{
  if (calcSignedArcLength(input_path, self_pose, dead_line_point) > dead_line_range) {
    return false;
  }

  double yaw;
  if (dead_line_point_idx == 0)
    yaw = std::atan2(
      input_path.points.at(dead_line_point_idx + 1).point.pose.position.y - dead_line_point.y(),
      input_path.points.at(dead_line_point_idx + 1).point.pose.position.x - dead_line_point.x());
  else
    yaw = std::atan2(
      dead_line_point.y() - input_path.points.at(dead_line_point_idx - 1).point.pose.position.y,
      dead_line_point.x() - input_path.points.at(dead_line_point_idx - 1).point.pose.position.x);

  // Calculate transform from dead_line_pose to self_pose
  tf2::Transform tf_dead_line_pose2self_pose;
  {
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    tf2::Transform tf_map2dead_line_pose(
      quat, tf2::Vector3(dead_line_point.x(), dead_line_point.y(), self_pose.position.z));
    tf2::Transform tf_map2self_pose;
    tf2::fromMsg(self_pose, tf_map2self_pose);
    tf_dead_line_pose2self_pose = tf_map2dead_line_pose.inverse() * tf_map2self_pose;

    // debug code
    geometry_msgs::Pose dead_line_pose;
    tf2::toMsg(tf_map2dead_line_pose, dead_line_pose);
    debug_data_.dead_line_poses.push_back(dead_line_pose);
  }

  if (0 < tf_dead_line_pose2self_pose.getOrigin().x()) {
    ROS_WARN("[traffic_light] vehicle is over dead line");
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
    double min_dist = 0.0;
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
