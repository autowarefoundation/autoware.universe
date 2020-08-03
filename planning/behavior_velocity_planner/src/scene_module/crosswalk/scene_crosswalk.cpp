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
#include <scene_module/crosswalk/scene_crosswalk.h>
#include <utilization/util.h>

#include <cmath>

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

CrosswalkModule::CrosswalkModule(
  const int64_t module_id, const lanelet::ConstLanelet & crosswalk,
  const PlannerParam & planner_param)
: SceneModuleInterface(module_id), crosswalk_(crosswalk), state_(State::APPROACH)
{
  planner_param_ = planner_param;
}

bool CrosswalkModule::modifyPathVelocity(
  autoware_planning_msgs::PathWithLaneId * path, autoware_planning_msgs::StopReason * stop_reason)
{
  debug_data_ = {};
  debug_data_.base_link2front = planner_data_->base_link2front;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;
  *stop_reason =
    planning_utils::initializeStopReason(autoware_planning_msgs::StopReason::CROSSWALK);

  const auto input = *path;

  // create polygon
  lanelet::CompoundPolygon3d lanelet_polygon = crosswalk_.polygon3d();
  Polygon polygon;
  for (const auto & lanelet_point : lanelet_polygon) {
    polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
  }
  polygon.outer().push_back(polygon.outer().front());

  // check state
  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;
  if (bg::within(Point(self_pose.pose.position.x, self_pose.pose.position.y), polygon))
    state_ = State::INSIDE;
  else if (state_ == State::INSIDE)
    state_ = State::GO_OUT;

  if (state_ == State::APPROACH) {
    // check person in polygon
    const auto objects_ptr = planner_data_->dynamic_objects;
    const auto no_ground_pointcloud_ptr = planner_data_->no_ground_pointcloud;

    autoware_planning_msgs::PathWithLaneId slow_path, stop_path;
    if (!checkSlowArea(input, polygon, objects_ptr, no_ground_pointcloud_ptr, slow_path)) {
      return false;
    }

    bool insert_stop;
    if (!checkStopArea(
          slow_path, polygon, objects_ptr, no_ground_pointcloud_ptr, stop_path, &insert_stop)) {
      return false;
    }
    // stop_path = slow_path;
    *path = stop_path;

    if (insert_stop) {
      /* get stop point and stop factor */
      autoware_planning_msgs::StopFactor stop_factor;
      stop_factor.stop_pose = debug_data_.first_stop_pose;
      stop_factor.stop_factor_points = debug_data_.stop_factor_points;
      planning_utils::appendStopReason(stop_factor, stop_reason);
    }
  }
  return true;
}

bool CrosswalkModule::checkStopArea(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
    crosswalk_polygon,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & objects_ptr,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  autoware_planning_msgs::PathWithLaneId & output, bool * insert_stop)
{
  output = input;
  *insert_stop = false;
  bool pedestrian_found = false;
  bool object_found = false;
  ros::Time current_time = ros::Time::now();

  // create stop area
  std::vector<Point> path_collision_points;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    const auto p0 = output.points.at(i).point.pose.position;
    const auto p1 = output.points.at(i + 1).point.pose.position;
    const Line line{{p0.x, p0.y}, {p1.x, p1.y}};
    std::vector<Point> line_collision_points;
    bg::intersection(crosswalk_polygon, line, line_collision_points);
    if (line_collision_points.empty()) continue;
    for (size_t j = 0; j < line_collision_points.size(); ++j) {
      path_collision_points.push_back(line_collision_points.at(j));
    }
  }
  if (path_collision_points.size() != 2) {
    // ROS_ERROR_THROTTLE(1, "Must be 2. Size is %d", (int)path_collision_points.size());
    return false;
  }

  Polygon stop_polygon;
  {
    constexpr double extension_margin = 0.25;
    const double width = planner_data_->vehicle_width;
    const double d = (width / 2.0) + extension_margin;
    const auto cp0 = path_collision_points.at(0);
    const auto cp1 = path_collision_points.at(1);
    const double yaw = std::atan2(cp1.y() - cp0.y(), cp1.x() - cp0.x()) + M_PI_2;
    const double dcosyaw = d * std::cos(yaw);
    const double dsinyaw = d * std::sin(yaw);
    stop_polygon.outer().push_back(bg::make<Point>(cp0.x() + dcosyaw, cp0.y() + dsinyaw));
    stop_polygon.outer().push_back(bg::make<Point>(cp0.x() - dcosyaw, cp0.y() - dsinyaw));
    stop_polygon.outer().push_back(bg::make<Point>(cp1.x() - dcosyaw, cp1.y() - dsinyaw));
    stop_polygon.outer().push_back(bg::make<Point>(cp1.x() + dcosyaw, cp1.y() + dsinyaw));
    stop_polygon.outer().push_back(stop_polygon.outer().front());
  }

  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < stop_polygon.outer().size(); ++i) {
    const auto p = stop_polygon.outer().at(i);
    points.push_back(Eigen::Vector3d(p.x(), p.y(), planner_data_->current_pose.pose.position.z));
  }
  debug_data_.stop_polygons.push_back(points);
  // ----------------

  // check object pointcloud
  for (size_t i = 0; i < no_ground_pointcloud_ptr->size(); ++i) {
    Point point(no_ground_pointcloud_ptr->at(i).x, no_ground_pointcloud_ptr->at(i).y);
    if (!bg::within(point, crosswalk_polygon)) continue;
    if (bg::within(point, stop_polygon)) {
      object_found = true;
      debug_data_.stop_factor_points.emplace_back(
        planning_utils::toRosPoint(no_ground_pointcloud_ptr->at(i)));
      break;
    }
  }

  // check pedestrian
  for (const auto & object : objects_ptr->objects) {
    if (object_found) break;

    if (isTargetType(object)) {
      Point point(
        object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y);
      if (bg::within(point, stop_polygon)) {
        pedestrian_found = true;
      }
      for (const auto & object_path : object.state.predicted_paths) {
        for (size_t k = 0; k < object_path.path.size() - 1; ++k) {
          if (
            (current_time - object_path.path.at(k).header.stamp).toSec() <
            planner_param_.stop_dynamic_object_prediction_time_margin) {
            const auto op0 = object_path.path.at(k).pose.pose.position;
            const auto op1 = object_path.path.at(k + 1).pose.pose.position;
            const Line line{{op0.x, op0.y}, {op1.x, op1.y}};
            std::vector<Point> line_collision_points;
            bg::intersection(stop_polygon, line, line_collision_points);
            if (!line_collision_points.empty()) pedestrian_found = true;
            if (pedestrian_found) {
              debug_data_.stop_factor_points.emplace_back(
                object.state.pose_covariance.pose.position);
              break;
            }
          }
        }
      }
    }
  }

  if (!pedestrian_found && !object_found) return true;

  // insert stop point
  if (!insertTargetVelocityPoint(
        input, crosswalk_polygon, planner_param_.stop_margin, 0.0, *planner_data_, output,
        debug_data_, first_stop_path_point_index_)) {
    return false;
  }
  *insert_stop = true;
  return true;
}

bool CrosswalkModule::checkSlowArea(
  const autoware_planning_msgs::PathWithLaneId & input, const Polygon & polygon,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & objects_ptr,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  autoware_planning_msgs::PathWithLaneId & output)
{
  output = input;
  bool pedestrian_found = false;
  for (size_t i = 0; i < objects_ptr->objects.size(); ++i) {
    if (isTargetType(objects_ptr->objects.at(i))) {
      Point point(
        objects_ptr->objects.at(i).state.pose_covariance.pose.position.x,
        objects_ptr->objects.at(i).state.pose_covariance.pose.position.y);
      if (bg::within(point, polygon)) {
        pedestrian_found = true;
      }
    }
  }
  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < polygon.outer().size(); ++i) {
    Eigen::Vector3d point;
    point << polygon.outer().at(i).x(), polygon.outer().at(i).y(),
      planner_data_->current_pose.pose.position.z;
    points.push_back(point);
  }
  debug_data_.slow_polygons.push_back(points);
  // ----------------

  if (!pedestrian_found) return true;

  // insert slow point
  if (!insertTargetVelocityPoint(
        input, polygon, planner_param_.slow_margin, planner_param_.slow_velocity, *planner_data_,
        output, debug_data_, first_stop_path_point_index_))
    return false;
  return true;
}

bool CrosswalkModule::isTargetType(const autoware_perception_msgs::DynamicObject & obj)
{
  if (
    obj.semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN ||
    obj.semantic.type == autoware_perception_msgs::Semantic::BICYCLE) {
    return true;
  }
  return false;
}
