/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <lane_change_planner/utilities.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <tf2/utils.h>
#include <opencv2/opencv.hpp>

namespace
{
ros::Duration safeSubtraction(const ros::Time & t1, const ros::Time & t2)
{
  ros::Duration duration;
  try {
    duration = t1 - t2;
  } catch (std::runtime_error) {
    if (t1 > t2)
      duration = ros::DURATION_MIN;
    else
      duration = ros::DURATION_MAX;
  }
  return duration;
}
ros::Time safeAddition(const ros::Time & t1, const double seconds)
{
  ros::Time sum;
  try {
    sum = t1 + ros::Duration(seconds);
  } catch (std::runtime_error & err) {
    if (seconds > 0) sum = ros::TIME_MAX;
    if (seconds < 0) sum = ros::TIME_MIN;
  }
  return sum;
}

cv::Point toCVPoint(
  const geometry_msgs::Point & geom_point, const double width_m, const double height_m,
  const double resolution)
{
  return cv::Point(
    static_cast<int>((height_m - geom_point.y) / resolution),
    static_cast<int>((width_m - geom_point.x) / resolution));
}

void imageToOccupancyGrid(const cv::Mat & cv_image, nav_msgs::OccupancyGrid * occupancy_grid)
{
  occupancy_grid->data.reserve(cv_image.rows * cv_image.cols);
  for (int x = cv_image.cols - 1; x >= 0; x--) {
    for (int y = cv_image.rows - 1; y >= 0; y--) {
      const unsigned char intensity = cv_image.at<unsigned char>(y, x);
      occupancy_grid->data.push_back(intensity);
    }
  }
}

}  // namespace

namespace lane_change_planner
{
namespace util
{
using autoware_perception_msgs::PredictedPath;
using autoware_planning_msgs::PathWithLaneId;

double l2Norm(const geometry_msgs::Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

Eigen::Vector3d convertToEigenPt(const geometry_msgs::Point geom_pt)
{
  return Eigen::Vector3d(geom_pt.x, geom_pt.y, geom_pt.z);
}

// returns false when search point is off the linestring
bool convertToFrenetCoordinate3d(
  const std::vector<geometry_msgs::Point> & linestring,
  const geometry_msgs::Point search_point_geom, FrenetCoordinate3d * frenet_coordinate)
{
  if (linestring.empty()) {
    return false;
  }

  const auto search_pt = convertToEigenPt(search_point_geom);
  bool found = false;
  double min_distance = std::numeric_limits<double>::max();

  // get frenet coordinate based on points
  // this is done because linestring is not differentiable at vertices
  {
    double accumulated_length = 0;

    for (std::size_t i = 0; i < linestring.size(); i++) {
      const auto geom_pt = linestring.at(i);
      const auto current_pt = convertToEigenPt(geom_pt);
      const auto current2search_pt = (search_pt - current_pt);
      // update accumulated length
      if (i != 0) {
        const auto p1 = convertToEigenPt(linestring.at(i - 1));
        const auto p2 = current_pt;
        accumulated_length += (p2 - p1).norm();
      }
      // update frenet coordinate

      const double tmp_distance = current2search_pt.norm();
      if (tmp_distance < min_distance) {
        found = true;
        min_distance = tmp_distance;
        frenet_coordinate->distance = tmp_distance;
        frenet_coordinate->length = accumulated_length;
      }
    }
  }

  // get frenet coordinate based on lines
  {
    auto prev_geom_pt = linestring.front();
    double accumulated_length = 0;
    for (const auto & geom_pt : linestring) {
      const auto start_pt = convertToEigenPt(prev_geom_pt);
      const auto end_pt = convertToEigenPt(geom_pt);

      const auto line_segment = end_pt - start_pt;
      const double line_segment_length = line_segment.norm();
      const auto direction = line_segment / line_segment_length;
      const auto start2search_pt = (search_pt - start_pt);

      double tmp_length = direction.dot(start2search_pt);
      if (tmp_length >= 0 && tmp_length <= line_segment_length) {
        double tmp_distance = direction.cross(start2search_pt).norm();
        if (tmp_distance < min_distance) {
          found = true;
          min_distance = tmp_distance;
          frenet_coordinate->distance = tmp_distance;
          frenet_coordinate->length = accumulated_length + tmp_length;
        }
      }
      accumulated_length += line_segment_length;
      prev_geom_pt = geom_pt;
    }
  }
  return found;
}

std::vector<geometry_msgs::Point> convertToGeometryPointArray(const PathWithLaneId & path)
{
  std::vector<geometry_msgs::Point> converted_path;
  converted_path.reserve(path.points.size());
  for (const auto & point_with_id : path.points) {
    converted_path.push_back(point_with_id.point.pose.position);
  }
  return converted_path;
}

std::vector<geometry_msgs::Point> convertToGeometryPointArray(const PredictedPath & path)
{
  std::vector<geometry_msgs::Point> converted_path;

  converted_path.reserve(path.path.size());
  for (const auto & pose_with_cov_stamped : path.path) {
    converted_path.push_back(pose_with_cov_stamped.pose.pose.position);
  }
  return converted_path;
}

geometry_msgs::PoseArray convertToGeometryPoseArray(const PathWithLaneId & path)
{
  geometry_msgs::PoseArray converted_array;
  converted_array.header = path.header;

  converted_array.poses.reserve(path.points.size());
  for (const auto & point_with_id : path.points) {
    converted_array.poses.push_back(point_with_id.point.pose);
  }
  return converted_array;
}

PredictedPath convertToPredictedPath(
  const PathWithLaneId & path, const geometry_msgs::Twist & vehicle_twist,
  const geometry_msgs::Pose & vehicle_pose)
{
  PredictedPath predicted_path;
  predicted_path.path.reserve(path.points.size());
  if (path.points.empty()) {
    return predicted_path;
  }

  const auto & geometry_points = convertToGeometryPointArray(path);
  FrenetCoordinate3d vehicle_pose_frenet;
  convertToFrenetCoordinate3d(geometry_points, vehicle_pose.position, &vehicle_pose_frenet);
  ros::Time start_time = ros::Time::now();
  double vehicle_speed = std::abs(vehicle_twist.linear.x);
  constexpr double min_speed = 1.0;
  if (vehicle_speed < min_speed) {
    vehicle_speed = min_speed;
    ROS_DEBUG_STREAM_THROTTLE(
      1, "cannot convert PathWithLaneId with zero velocity, using minimum value "
           << min_speed << " [m/s] instead");
  }
  double accumulated_distance = 0;

  auto prev_pt = path.points.front();
  for (size_t i = 0; i < path.points.size(); i++) {
    auto pt = path.points.at(i);
    FrenetCoordinate3d pt_frenet;
    if (!convertToFrenetCoordinate3d(geometry_points, pt.point.pose.position, &pt_frenet)) {
      continue;
    }
    double frenet_distance = pt_frenet.length - vehicle_pose_frenet.length;
    double travel_time = frenet_distance / vehicle_speed;
    const auto time_stamp = safeAddition(start_time, travel_time);

    geometry_msgs::PoseWithCovarianceStamped predicted_pose;
    predicted_pose.header.stamp = time_stamp;
    predicted_pose.pose.pose.position = pt.point.pose.position;
    predicted_pose.pose.pose.orientation = pt.point.pose.orientation;
    predicted_path.path.push_back(predicted_pose);
    prev_pt = pt;
  }
  return predicted_path;
}

PredictedPath resamplePredictedPath(
  const PredictedPath & input_path, const double resolution, const double duration)
{
  PredictedPath resampled_path;

  ros::Duration t_delta(resolution);
  ros::Duration prediction_duration(duration);

  double min_distance = std::numeric_limits<double>::max();
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = ros::Time::now() + prediction_duration;

  for (auto t = start_time; t < end_time; t += t_delta) {
    geometry_msgs::Pose pose;
    if (!lerpByTimeStamp(input_path, t, &pose)) {
      continue;
    }
    geometry_msgs::PoseWithCovarianceStamped predicted_pose;
    predicted_pose.header.frame_id = "map";
    predicted_pose.header.stamp = t;
    predicted_pose.pose.pose = pose;
    resampled_path.path.push_back(predicted_pose);
  }

  return resampled_path;
}

geometry_msgs::Pose lerpByPose(
  const geometry_msgs::Pose & p1, const geometry_msgs::Pose & p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto & tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto & tf_quaternion =
    tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::Pose pose;
  pose.position = tf2::toMsg(tf_point, pose.position);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool lerpByTimeStamp(
  const PredictedPath & path, const ros::Time & t, geometry_msgs::Pose * lerped_pt)
{
  if (lerped_pt == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "failed to lerp by time due to nullptr pt");
    return false;
  }
  if (path.path.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "Empty path. Failed to interpolate path by time!");
    return false;
  }
  if (t < path.path.front().header.stamp) {
    ROS_DEBUG_STREAM(
      "failed to interpolate path by time!"
      << std::endl
      << "path start time: " << path.path.front().header.stamp << std::endl
      << "path end time  : " << path.path.back().header.stamp << std::endl
      << "query time     : " << t);
    *lerped_pt = path.path.front().pose.pose;
    return false;
  }

  if (t > path.path.back().header.stamp) {
    ROS_DEBUG_STREAM(
      "failed to interpolate path by time!"
      << std::endl
      << "path start time: " << path.path.front().header.stamp << std::endl
      << "path end time  : " << path.path.back().header.stamp << std::endl
      << "query time     : " << t);
    *lerped_pt = path.path.back().pose.pose;

    return false;
  }

  for (size_t i = 1; i < path.path.size(); i++) {
    const auto & pt = path.path.at(i);
    const auto & prev_pt = path.path.at(i - 1);
    if (t < pt.header.stamp) {
      const ros::Duration duration = safeSubtraction(pt.header.stamp, prev_pt.header.stamp);
      const auto off_set = t - prev_pt.header.stamp;
      const auto ratio = off_set.toSec() / duration.toSec();
      *lerped_pt = lerpByPose(prev_pt.pose.pose, pt.pose.pose, ratio);
      return true;
    }
  }

  ROS_ERROR_STREAM("Something failed in function: " << __func__);
  return false;
}

double getDistance3d(const geometry_msgs::Point & p1, const geometry_msgs::Point & p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double getDistanceBetweenPredictedPaths(
  const PredictedPath & object_path, const PredictedPath & ego_path, const double start_time,
  const double end_time, const double resolution, const bool use_vehicle_width,
  const double vehicle_width)
{
  ros::Duration t_delta(resolution);
  // ros::Duration prediction_duration(duration);
  double min_distance = std::numeric_limits<double>::max();
  ros::Time ros_start_time = ros::Time::now() + ros::Duration(start_time);
  ros::Time ros_end_time = ros::Time::now() + ros::Duration(end_time);
  const auto ego_path_point_array = convertToGeometryPointArray(ego_path);
  for (auto t = ros_start_time; t < ros_end_time; t += t_delta) {
    geometry_msgs::Pose object_pose, ego_pose;
    if (!lerpByTimeStamp(object_path, t, &object_pose)) {
      continue;
    }
    if (!lerpByTimeStamp(ego_path, t, &ego_pose)) {
      continue;
    }
    if (use_vehicle_width) {
      FrenetCoordinate3d frenet_coordinate;
      if (convertToFrenetCoordinate3d(
            ego_path_point_array, object_pose.position, &frenet_coordinate)) {
        if (frenet_coordinate.distance > vehicle_width) {
          continue;
        }
      }
    }
    double distance = getDistance3d(object_pose.position, ego_pose.position);
    if (distance < min_distance) {
      min_distance = distance;
    }
  }
  return min_distance;
}

std::vector<size_t> filterObjectsByLanelets(
  const autoware_perception_msgs::DynamicObjectArray & objects,
  const lanelet::ConstLanelets & target_lanelets, const double start_arc_length,
  const double end_arc_length)
{
  std::vector<size_t> indices;
  if (target_lanelets.empty()) {
    return indices;
  }
  const auto polygon =
    lanelet::utils::getPolygonFromArcLength(target_lanelets, start_arc_length, end_arc_length);
  const auto polygon2d = lanelet::utils::to2D(polygon).basicPolygon();
  for (size_t i = 0; i < objects.objects.size(); i++) {
    const auto & obj_position = objects.objects.at(i).state.pose_covariance.pose.position;
    lanelet::BasicPoint2d obj_position2d(obj_position.x, obj_position.y);

    double distance = boost::geometry::distance(polygon2d, obj_position2d);
    if (distance < std::numeric_limits<double>::epsilon()) {
      indices.push_back(i);
    }
  }
  return indices;
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId & input_path)
{
  PathWithLaneId filtered_path;
  for (const auto & pt : input_path.points) {
    if (filtered_path.points.empty()) {
      filtered_path.points.push_back(pt);
      continue;
    }
    if (
      getDistance3d(filtered_path.points.back().point.pose.position, pt.point.pose.position) <
      std::numeric_limits<double>::epsilon()) {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
    } else {
      filtered_path.points.push_back(pt);
    }
  }
  filtered_path.drivable_area = input_path.drivable_area;
  return filtered_path;
}

template <typename T>
bool exists(std::vector<T> vec, T element)
{
  return std::find(vec.begin(), vec.end(), element) != vec.end();
}

bool setGoal(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::Pose & goal, const int64_t goal_lane_id, PathWithLaneId * output_ptr)
{
  try {
    if (input.points.empty()) {
      return false;
    }
    size_t min_dist_index;
    double min_dist = std::numeric_limits<double>::max();
    double goal_z;
    {
      bool found = false;
      for (size_t i = 0; i < input.points.size(); ++i) {
        const double x = input.points.at(i).point.pose.position.x - goal.position.x;
        const double y = input.points.at(i).point.pose.position.y - goal.position.y;
        const double z = input.points.at(i).point.pose.position.z - goal.position.z;
        const double dist = sqrt(x * x + y * y);
        if (
          dist < search_radius_range && dist < min_dist &&
          exists(input.points.at(i).lane_ids, goal_lane_id)) {
          min_dist_index = i;
          min_dist = dist;
          found = true;
        }
      }
      if (!found) {
        return false;
      }
    }

    size_t min_dist_out_of_range_index;
    {
      for (size_t i = min_dist_index; 0 <= i; --i) {
        const double x = input.points.at(i).point.pose.position.x - goal.position.x;
        const double y = input.points.at(i).point.pose.position.y - goal.position.y;
        const double z = input.points.at(i).point.pose.position.z - goal.position.z;
        goal_z = input.points.at(i).point.pose.position.z;
        const double dist = sqrt(x * x + y * y);
        min_dist_out_of_range_index = i;
        if (search_radius_range < dist) {
          break;
        }
        if (i == 0) {
          break;
        }
      }
    }
    autoware_planning_msgs::PathPointWithLaneId refined_goal;
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z = goal_z;
    refined_goal.point.twist.linear.x = 0.0;
    refined_goal.lane_ids = input.points.back().lane_ids;

    autoware_planning_msgs::PathPointWithLaneId pre_refined_goal;
    double roll, pitch, yaw;
    pre_refined_goal.point.pose = goal;
    tf2::Quaternion tf2_quaternion(
      goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
    tf2::Matrix3x3 tf2_matrix(tf2_quaternion);
    tf2_matrix.getRPY(roll, pitch, yaw);
    pre_refined_goal.point.pose.position.x -= std::cos(yaw);
    pre_refined_goal.point.pose.position.y -= std::sin(yaw);
    pre_refined_goal.point.pose.position.z = goal_z;
    pre_refined_goal.point.twist.linear.x =
      input.points.at(min_dist_out_of_range_index).point.twist.linear.x;
    pre_refined_goal.lane_ids = input.points.back().lane_ids;

    for (size_t i = 0; i <= min_dist_out_of_range_index; ++i) {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);

    output_ptr->drivable_area = input.drivable_area;
    return true;
  } catch (std::out_of_range & ex) {
    ROS_ERROR_STREAM("failed to set goal" << ex.what() << std::endl);
    return false;
  }
}

const geometry_msgs::Pose refineGoal(
  const geometry_msgs::Pose & goal, const lanelet::ConstLanelet & goal_lanelet)
{
  // return goal;
  const auto lanelet_point = lanelet::utils::conversion::toLaneletPoint(goal.position);
  const double distance = boost::geometry::distance(
    goal_lanelet.polygon2d().basicPolygon(), lanelet::utils::to2D(lanelet_point).basicPoint());
  if (distance < std::numeric_limits<double>::epsilon()) {
    return goal;
  }

  const auto segment = lanelet::utils::getClosestSegment(
    lanelet::utils::to2D(lanelet_point), goal_lanelet.centerline());
  if (segment.empty()) {
    return goal;
  }

  geometry_msgs::Pose refined_goal;
  {
    // find position
    const auto p1 = segment.front().basicPoint();
    const auto p2 = segment.back().basicPoint();
    const auto direction_vector = (p2 - p1).normalized();
    const auto p1_to_goal = lanelet_point.basicPoint() - p1;
    const double s = direction_vector.dot(p1_to_goal);
    const auto refined_point = p1 + direction_vector * s;

    refined_goal.position.x = refined_point.x();
    refined_goal.position.y = refined_point.y();
    refined_goal.position.z = refined_point.z();

    // find orientation
    const double yaw = std::atan2(direction_vector.y(), direction_vector.x());
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw);
    refined_goal.orientation = tf2::toMsg(tf_quat);
  }
  return refined_goal;
}

PathWithLaneId refinePath(
  const double search_radius_range, const double search_rad_range, const PathWithLaneId & input,
  const geometry_msgs::Pose & goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path, path_with_goal;
  filtered_path = removeOverlappingPoints(input);

  // always set zero velocity at the end of path for safety
  if (!filtered_path.points.empty()) {
    filtered_path.points.back().point.twist.linear.x = 0.0;
  }

  if (setGoal(
        search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id,
        &path_with_goal)) {
    return path_with_goal;
  } else {
    return filtered_path;
  }
}

nav_msgs::OccupancyGrid convertLanesToDrivableArea(
  const lanelet::ConstLanelets & lanes, const geometry_msgs::PoseStamped & current_pose,
  const double width, const double height, const double resolution)
{
  nav_msgs::OccupancyGrid occupancy_grid;
  geometry_msgs::PoseStamped grid_origin;

  // calculate grid origin
  {
    grid_origin.header = current_pose.header;
    const double yaw = tf2::getYaw(current_pose.pose.orientation);
    const double origin_offset_x_m = (-width / 4) * cos(yaw) - (-height / 2) * sin(yaw);
    const double origin_offset_y_m = (-width / 4) * sin(yaw) + (-height / 2) * cos(yaw);
    grid_origin.pose.orientation = current_pose.pose.orientation;
    grid_origin.pose.position.x = current_pose.pose.position.x + origin_offset_x_m;
    grid_origin.pose.position.y = current_pose.pose.position.y + origin_offset_y_m;
    grid_origin.pose.position.z = current_pose.pose.position.z;
  }

  // header
  {
    occupancy_grid.header.stamp = current_pose.header.stamp;
    occupancy_grid.header.frame_id = "map";
  }

  // info
  {
    const int width_cell = width / resolution;
    const int height_cell = height / resolution;

    occupancy_grid.info.map_load_time = occupancy_grid.header.stamp;
    occupancy_grid.info.resolution = resolution;
    occupancy_grid.info.width = width_cell;
    occupancy_grid.info.height = height_cell;
    occupancy_grid.info.origin = grid_origin.pose;
  }

  // occupancy_grid.data = image;
  {
    constexpr uint8_t free_space = 0;
    constexpr uint8_t occupied_space = 100;
    // get transform
    tf2::Stamped<tf2::Transform> tf_grid2map, tf_map2grid;
    tf2::fromMsg(grid_origin, tf_grid2map);
    tf_map2grid.setData(tf_grid2map.inverse());
    const auto geom_tf_map2grid = tf2::toMsg(tf_map2grid);

    // convert lane polygons into cv type
    cv::Mat cv_image(
      occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1, cv::Scalar(occupied_space));
    for (std::size_t i = 0; i < lanes.size(); i++) {
      const auto lane = lanes.at(i);

      // skip if it overlaps with past lane
      bool overlaps_with_past_lane = false;
      for (std::size_t j = 0; j < i; j++) {
        const auto past_lane = lanes.at(j);
        if (boost::geometry::overlaps(
              lane.polygon2d().basicPolygon(), past_lane.polygon2d().basicPolygon())) {
          overlaps_with_past_lane = true;
          break;
        }
      }
      if (overlaps_with_past_lane) {
        continue;
      }

      // create drivable area using opencv
      std::vector<std::vector<cv::Point>> cv_polygons;
      std::vector<int> cv_polygon_sizes;
      cv::Mat cv_image_single_lane(
        occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1, cv::Scalar(occupied_space));
      std::vector<cv::Point> cv_polygon;
      for (const auto & llt_pt : lane.polygon3d()) {
        geometry_msgs::Point geom_pt = lanelet::utils::conversion::toGeomMsgPt(llt_pt);
        geometry_msgs::Point transformed_geom_pt;
        tf2::doTransform(geom_pt, transformed_geom_pt, geom_tf_map2grid);
        cv_polygon.push_back(toCVPoint(transformed_geom_pt, width, height, resolution));
      }
      cv_polygons.push_back(cv_polygon);
      cv_polygon_sizes.push_back(cv_polygon.size());
      // fill in drivable area and copy to occupancy grid
      cv::fillPoly(cv_image_single_lane, cv_polygons, cv::Scalar(free_space));
      cv::bitwise_and(cv_image, cv_image_single_lane, cv_image);
    }

    const auto & cv_image_reshaped = cv_image.reshape(1, 1);
    imageToOccupancyGrid(cv_image, &occupancy_grid);
    occupancy_grid.data[0] = 0;
    // cv_image_reshaped.copyTo(occupancy_grid.data);
  }
  return occupancy_grid;
}

double getDistanceToEndOfLane(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);
  const double lanelet_length = lanelet::utils::getLaneletLength3d(lanelets);
  return lanelet_length - arc_coordinates.length;
}

double getDistanceToNextIntersection(
  const geometry_msgs::Pose & current_pose, const lanelet::ConstLanelets & lanelets)
{
  const auto & arc_coordinates = lanelet::utils::getArcCoordinates(lanelets, current_pose);

  lanelet::ConstLanelet current_lanelet;
  if (!lanelet::utils::query::getClosestLanelet(lanelets, current_pose, &current_lanelet)) {
    return std::numeric_limits<double>::max();
  }

  double distance = 0;
  bool is_after_current_lanelet = false;
  for (const auto & llt : lanelets) {
    if (llt == current_lanelet) {
      is_after_current_lanelet = true;
    }
    if (is_after_current_lanelet && llt.hasAttribute("turn_direction")) {
      return distance - arc_coordinates.length;
    }
    distance += lanelet::utils::getLaneletLength3d(llt);
  }

  return std::numeric_limits<double>::max();
}  // namespace util

std::vector<uint64_t> getIds(const lanelet::ConstLanelets & lanelets)
{
  std::vector<uint64_t> ids;
  for (const auto & llt : lanelets) {
    ids.push_back(llt.id());
  }
  return ids;
}

}  // namespace util
}  // namespace lane_change_planner
