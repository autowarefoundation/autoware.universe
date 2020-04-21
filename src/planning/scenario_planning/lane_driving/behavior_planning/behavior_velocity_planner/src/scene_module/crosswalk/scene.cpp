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
#include <scene_module/crosswalk/scene.h>

#include <cmath>

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

CrosswalkModule::CrosswalkModule(const int64_t module_id, const lanelet::ConstLanelet & crosswalk)
: SceneModuleInterface(module_id), crosswalk_(crosswalk), state_(State::APPROARCH)
{
}

bool CrosswalkModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path)
{
  debug_data_ = {};
  debug_data_.base_link2front = planner_data_->base_link2front;

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

  if (state_ == State::APPROARCH) {
    // check person in polygon
    const auto objects_ptr = planner_data_->dynamic_objects;
    const auto no_ground_pointcloud_ptr = planner_data_->no_ground_pointcloud;

    autoware_planning_msgs::PathWithLaneId slow_path, stop_path;
    if (!checkSlowArea(input, polygon, objects_ptr, no_ground_pointcloud_ptr, slow_path)) {
      return false;
    }
    if (!checkStopArea(slow_path, polygon, objects_ptr, no_ground_pointcloud_ptr, stop_path)) {
      return false;
    }
    // stop_path = slow_path;
    *path = stop_path;
  }
  return true;
}

bool CrosswalkModule::checkStopArea(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
    crosswalk_polygon,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & objects_ptr,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  autoware_planning_msgs::PathWithLaneId & output)
{
  output = input;
  bool pedestrian_found = false;
  bool object_found = false;
  ros::Time current_time = ros::Time::now();

  // create stop area
  std::vector<Point> path_collision_points;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    Line line = {
      {output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
      {output.points.at(i + 1).point.pose.position.x,
       output.points.at(i + 1).point.pose.position.y}};
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

  const double width = planner_data_->vehicle_width;

  const double yaw = std::atan2(
                       path_collision_points.at(1).y() - path_collision_points.at(0).y(),
                       path_collision_points.at(1).x() - path_collision_points.at(0).x()) +
                     M_PI_2;
  Polygon stop_polygon;
  const double extension_margin = 0.25;
  stop_polygon.outer().push_back(bg::make<Point>(
    path_collision_points.at(0).x() + std::cos(yaw) * ((width / 2.0) + extension_margin),
    path_collision_points.at(0).y() + std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(bg::make<Point>(
    path_collision_points.at(0).x() - std::cos(yaw) * ((width / 2.0) + extension_margin),
    path_collision_points.at(0).y() - std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(bg::make<Point>(
    path_collision_points.at(1).x() - std::cos(yaw) * ((width / 2.0) + extension_margin),
    path_collision_points.at(1).y() - std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(bg::make<Point>(
    path_collision_points.at(1).x() + std::cos(yaw) * ((width / 2.0) + extension_margin),
    path_collision_points.at(1).y() + std::sin(yaw) * ((width / 2.0) + extension_margin)));
  stop_polygon.outer().push_back(stop_polygon.outer().front());

  // -- debug code --
  std::vector<Eigen::Vector3d> points;
  for (size_t i = 0; i < stop_polygon.outer().size(); ++i) {
    Eigen::Vector3d point;
    point << stop_polygon.outer().at(i).x(), stop_polygon.outer().at(i).y(),
      planner_data_->current_pose.pose.position.z;
    points.push_back(point);
  }
  debug_data_.stop_polygons.push_back(points);
  // ----------------

  // check object pointcloud
  for (size_t i = 0; i < no_ground_pointcloud_ptr->size(); ++i) {
    Point point(no_ground_pointcloud_ptr->at(i).x, no_ground_pointcloud_ptr->at(i).y);
    if (!bg::within(point, crosswalk_polygon)) continue;
    if (bg::within(point, stop_polygon)) {
      object_found = true;
    }
  }

  // check pedestrian
  for (const auto & object : objects_ptr->objects) {
    if (
      object.semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN ||
      object.semantic.type == autoware_perception_msgs::Semantic::BICYCLE) {
      Point point(
        object.state.pose_covariance.pose.position.x, object.state.pose_covariance.pose.position.y);
      if (bg::within(point, stop_polygon)) {
        pedestrian_found = true;
      }
      for (const auto & object_path : object.state.predicted_paths) {
        for (size_t k = 0; k < object_path.path.size() - 1; ++k) {
          if (
            (current_time - object_path.path.at(k).header.stamp).toSec() <
            stop_dynamic_object_prediction_time_margin_) {
            Line line = {{object_path.path.at(k).pose.pose.position.x,
                          object_path.path.at(k).pose.pose.position.y},
                         {object_path.path.at(k + 1).pose.pose.position.x,
                          object_path.path.at(k + 1).pose.pose.position.y}};
            std::vector<Point> line_collision_points;
            bg::intersection(stop_polygon, line, line_collision_points);
            if (!line_collision_points.empty()) pedestrian_found = true;
          }
        }
      }
    }
  }

  if (!pedestrian_found && !object_found) return true;

  // insert stop point
  if (!insertTargetVelocityPoint(input, crosswalk_polygon, stop_margin_, 0.0, output)) return false;
  return true;
}

bool CrosswalkModule::checkSlowArea(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
    polygon,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr & objects_ptr,
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & no_ground_pointcloud_ptr,
  autoware_planning_msgs::PathWithLaneId & output)
{
  const double slow_velocity = 1.39;  // 5kmph
  output = input;
  bool pedestrian_found = false;
  for (size_t i = 0; i < objects_ptr->objects.size(); ++i) {
    if (
      objects_ptr->objects.at(i).semantic.type == autoware_perception_msgs::Semantic::PEDESTRIAN) {
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
  if (!insertTargetVelocityPoint(input, polygon, slow_margin_, slow_velocity, output)) return false;
  return true;
}

bool CrosswalkModule::insertTargetVelocityPoint(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
    polygon,
  const double & margin, const double & velocity, autoware_planning_msgs::PathWithLaneId & output)
{
  output = input;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    Line line = {
      {output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
      {output.points.at(i + 1).point.pose.position.x,
       output.points.at(i + 1).point.pose.position.y}};
    std::vector<Point> collision_points;
    bg::intersection(polygon, line, collision_points);

    if (collision_points.empty()) continue;
    // -- debug code --
    for (const auto & collision_point : collision_points) {
      Eigen::Vector3d point3d;
      point3d << collision_point.x(), collision_point.y(),
        planner_data_->current_pose.pose.position.z;
      debug_data_.collision_points.push_back(point3d);
    }
    std::vector<Eigen::Vector3d> line3d;
    Eigen::Vector3d point3d;
    point3d << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y,
      output.points.at(i).point.pose.position.z;
    line3d.push_back(point3d);
    point3d << output.points.at(i + 1).point.pose.position.x,
      output.points.at(i + 1).point.pose.position.y, output.points.at(i + 1).point.pose.position.z;
    line3d.push_back(point3d);
    debug_data_.collision_lines.push_back(line3d);
    // ----------------

    // check nearest collision point
    Point nearest_collision_point;
    double min_dist;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(
        Point(output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y),
        collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
      }
    }

    // search target point index
    size_t insert_target_point_idx = 0;
    const double base_link2front = planner_data_->base_link2front;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    point1 << nearest_collision_point.x(), nearest_collision_point.y();
    point2 << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y;
    length_sum += (point2 - point1).norm();
    for (size_t j = i; 0 < j; --j) {
      if (target_length < length_sum) {
        insert_target_point_idx = j + 1;
        break;
      }
      point1 << output.points.at(j).point.pose.position.x,
        output.points.at(j).point.pose.position.y;
      point2 << output.points.at(j - 1).point.pose.position.x,
        output.points.at(j - 1).point.pose.position.y;
      length_sum += (point2 - point1).norm();
    }

    // create target point
    Eigen::Vector2d target_point;
    autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;
    getBackwordPointFromBasePoint(point2, point1, point2, length_sum - target_length, target_point);
    target_point_with_lane_id =
      output.points.at(std::max(static_cast<int>(insert_target_point_idx - 1), 0));
    target_point_with_lane_id.point.pose.position.x = target_point.x();
    target_point_with_lane_id.point.pose.position.y = target_point.y();
    target_point_with_lane_id.point.twist.linear.x = velocity;
    // -- debug code --
    if (velocity == 0.0)
      debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);
    else
      debug_data_.slow_poses.push_back(target_point_with_lane_id.point.pose);
    // ----------------

    // insert target point
    output.points.insert(
      output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

    // insert 0 velocity after target point
    for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
      output.points.at(j).point.twist.linear.x =
        std::min(velocity, output.points.at(j).point.twist.linear.x);
    return true;
  }
  return false;
}

bool CrosswalkModule::getBackwordPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}
