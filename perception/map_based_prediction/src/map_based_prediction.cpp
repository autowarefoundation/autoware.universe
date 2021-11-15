// Copyright 2020 Tier IV, Inc.
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

#include <cubic_spline.hpp>
#include <map_based_prediction.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <chrono>
#include <vector>

MapBasedPrediction::MapBasedPrediction(
  double interpolating_resolution, double time_horizon, double sampling_delta_time)
: interpolating_resolution_(interpolating_resolution),
  time_horizon_(time_horizon),
  sampling_delta_time_(sampling_delta_time)
{
}

double calculateEuclideanDistance(
  const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double distance = std::sqrt(dx * dx + dy * dy);
  return distance;
}

bool getNearestPoint(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Point point,
  geometry_msgs::msg::Point & nearest_point)
{
  double min_dist = 1e+10;
  bool flag = false;
  for (const auto & tmp_point : points) {
    double distance = calculateEuclideanDistance(point, tmp_point);
    if (distance < min_dist) {
      min_dist = distance;
      nearest_point = point;
      flag = true;
    }
  }
  return flag;
}

bool getNearestPointIdx(
  const std::vector<geometry_msgs::msg::Point> & points, const geometry_msgs::msg::Point point,
  geometry_msgs::msg::Point & nearest_point, size_t & nearest_index)
{
  double min_dist = 10000000;
  bool flag = false;
  size_t index = 0;
  for (const auto & tmp_point : points) {
    double distance = calculateEuclideanDistance(point, tmp_point);
    if (distance < min_dist) {
      min_dist = distance;
      nearest_point = tmp_point;
      flag = true;
      nearest_index = index;
    }
    index++;
  }
  return flag;
}

bool MapBasedPrediction::doPrediction(
  const DynamicObjectWithLanesArray & in_objects,
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & out_objects,
  std::vector<geometry_msgs::msg::Point> & debug_interpolated_points)
{
  for (auto & object_with_lanes : in_objects.objects) {
    const double min_lon_velocity_ms_for_map_based_prediction = 1;
    if (
      std::fabs(object_with_lanes.object.kinematics.twist_with_covariance.twist.linear.x) <
      min_lon_velocity_ms_for_map_based_prediction) {
      autoware_auto_perception_msgs::msg::PredictedPath predicted_path;
      getLinearPredictedPath(
        object_with_lanes.object.kinematics.pose_with_covariance.pose,
        object_with_lanes.object.kinematics.twist_with_covariance.twist, predicted_path);
      autoware_auto_perception_msgs::msg::PredictedObject tmp_object;
      // convert to predicted object
      tmp_object = convertToPredictedObject(object_with_lanes.object);
      tmp_object.kinematics.predicted_paths.push_back(predicted_path);
      out_objects.push_back(tmp_object);
      continue;
    }
    autoware_auto_perception_msgs::msg::PredictedObject tmp_object;
    tmp_object = convertToPredictedObject(object_with_lanes.object);
    for (const auto & path : object_with_lanes.lanes) {
      std::vector<double> tmp_x;
      std::vector<double> tmp_y;
      std::vector<geometry_msgs::msg::Pose> geometry_points = path;
      for (size_t i = 0; i < path.size(); i++) {
        if (i > 0) {
          double dist = calculateEuclideanDistance(
            geometry_points[i].position, geometry_points[i - 1].position);
          if (dist < interpolating_resolution_) {
            continue;
          }
        }
        tmp_x.push_back(geometry_points[i].position.x);
        tmp_y.push_back(geometry_points[i].position.y);
      }

      Spline2D spline2d(tmp_x, tmp_y);
      std::vector<geometry_msgs::msg::Point> interpolated_points;
      std::vector<double> interpolated_yaws;
      for (float s = 0.0; s < spline2d.s.back(); s += interpolating_resolution_) {
        std::array<double, 2> point1 = spline2d.calc_position(s);
        geometry_msgs::msg::Point g_point;
        g_point.x = point1[0];
        g_point.y = point1[1];
        g_point.z = object_with_lanes.object.kinematics.pose_with_covariance.pose.position.z;
        interpolated_points.push_back(g_point);
        interpolated_yaws.push_back(spline2d.calc_yaw(s));
      }
      debug_interpolated_points = interpolated_points;

      geometry_msgs::msg::Point object_point =
        object_with_lanes.object.kinematics.pose_with_covariance.pose.position;
      geometry_msgs::msg::Point nearest_point;
      size_t nearest_point_idx;
      if (getNearestPointIdx(interpolated_points, object_point, nearest_point, nearest_point_idx)) {
        // calculate initial position in Frenet coordinate
        // Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame
        // Path Planning for Highly Automated Driving on Embedded GPUs
        double current_s_position =
          interpolating_resolution_ * static_cast<double>(nearest_point_idx);
        double current_d_position = calculateEuclideanDistance(nearest_point, object_point);

        double lane_yaw = spline2d.calc_yaw(current_s_position);
        std::vector<double> origin_v = {std::cos(lane_yaw), std::sin(lane_yaw)};
        std::vector<double> object_v = {
          object_point.x - nearest_point.x, object_point.y - nearest_point.y};
        double cross2d = object_v[0] * origin_v[1] - object_v[1] * origin_v[0];
        if (cross2d < 0) {
          current_d_position *= -1;
        }

        // Does not consider orientation of twist since predicting lane-direction
        double current_d_velocity =
          object_with_lanes.object.kinematics.twist_with_covariance.twist.linear.y;
        double current_s_velocity =
          std::fabs(object_with_lanes.object.kinematics.twist_with_covariance.twist.linear.x);
        autoware_auto_perception_msgs::msg::PredictedPath path;

        geometry_msgs::msg::PoseWithCovarianceStamped point;
        point.pose.pose.position = object_point;
        getPredictedPath(
          object_point.z, current_d_position, current_d_velocity, current_s_position,
          current_s_velocity, spline2d, path);
        tmp_object.kinematics.predicted_paths.push_back(path);
        if (
          tmp_object.kinematics.predicted_paths.size() >=
          tmp_object.kinematics.predicted_paths.capacity()) {
          break;
        }
      } else {
        continue;
      }
    }

    normalizeLikelihood(tmp_object.kinematics);
    out_objects.push_back(tmp_object);
  }
  return true;
}

bool MapBasedPrediction::doLinearPrediction(
  const autoware_auto_perception_msgs::msg::PredictedObjects & in_objects,
  std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & out_objects)
{
  for (const auto & object : in_objects.objects) {
    autoware_auto_perception_msgs::msg::PredictedPath path;
    getLinearPredictedPath(
      object.kinematics.initial_pose_with_covariance.pose,
      object.kinematics.initial_twist_with_covariance.twist, path);
    autoware_auto_perception_msgs::msg::PredictedObject tmp_object;
    tmp_object = object;
    tmp_object.kinematics.predicted_paths.push_back(path);
    out_objects.push_back(tmp_object);
  }

  return true;
}

autoware_auto_perception_msgs::msg::PredictedObject MapBasedPrediction::convertToPredictedObject(
  const autoware_auto_perception_msgs::msg::TrackedObject & tracked_object)
{
  autoware_auto_perception_msgs::msg::PredictedObject output;
  output.object_id = tracked_object.object_id;
  output.existence_probability = tracked_object.existence_probability;
  output.classification = tracked_object.classification;
  output.kinematics = convertToPredictedKinematics(tracked_object.kinematics);
  output.shape = tracked_object.shape;
  return output;
}

autoware_auto_perception_msgs::msg::PredictedObjectKinematics
MapBasedPrediction::convertToPredictedKinematics(
  const autoware_auto_perception_msgs::msg::TrackedObjectKinematics & tracked_object)
{
  autoware_auto_perception_msgs::msg::PredictedObjectKinematics output;
  output.initial_pose_with_covariance = tracked_object.pose_with_covariance;
  output.initial_twist_with_covariance = tracked_object.twist_with_covariance;
  output.initial_acceleration_with_covariance = tracked_object.acceleration_with_covariance;
  return output;
}

void MapBasedPrediction::normalizeLikelihood(
  autoware_auto_perception_msgs::msg::PredictedObjectKinematics & predicted_object_kinematics)
{
  // might not be the smartest way
  double sum_confidence = 0;
  for (size_t i = 0; i < predicted_object_kinematics.predicted_paths.size(); ++i) {
    sum_confidence += 1 / predicted_object_kinematics.predicted_paths.at(i).confidence;
  }

  for (size_t i = 0; i < predicted_object_kinematics.predicted_paths.size(); ++i) {
    predicted_object_kinematics.predicted_paths.at(i).confidence =
      (1 / predicted_object_kinematics.predicted_paths.at(i).confidence) / sum_confidence;
  }
}

bool MapBasedPrediction::getPredictedPath(
  const double height, const double current_d_position, const double current_d_velocity,
  const double current_s_position, const double current_s_velocity, Spline2D & spline2d,
  autoware_auto_perception_msgs::msg::PredictedPath & path)
{
  // Quintic polynomial for d
  // A = np.array([[T**3, T**4, T**5],
  //               [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
  //               [6 * T, 12 * T ** 2, 20 * T ** 3]])
  // A_inv = np.matrix([[10/(T**3), -4/(T**2), 1/(2*T)],
  //                    [-15/(T**4), 7/(T**3), -1/(T**2)],
  //                    [6/(T**5), -3/(T**4),  1/(2*T**3)]])
  // b = np.matrix([[xe - self.a0 - self.a1 * T - self.a2 * T**2],
  //                [vxe - self.a1 - 2 * self.a2 * T],
  //                [axe - 2 * self.a2]])
  double target_d_position = 0;

  double t = time_horizon_;
  Eigen::Matrix3d a_3_inv;
  a_3_inv << 10 / std::pow(t, 3), -4 / std::pow(t, 2), 1 / (2 * t), -15 / std::pow(t, 4),
    7 / std::pow(t, 3), -1 / std::pow(t, 2), 6 / std::pow(t, 5), -3 / std::pow(t, 4),
    1 / (2 * std::pow(t, 3));

  double target_d_velocity = current_d_velocity;
  double target_d_acceleration = 0;
  Eigen::Vector3d b_3;
  b_3 << target_d_position - current_d_position - current_d_velocity * t,
    target_d_velocity - current_d_velocity, target_d_acceleration;

  Eigen::Vector3d x_3;
  x_3 = a_3_inv * b_3;

  // Quadric polynomial
  // A_inv = np.matrix([[1/(T**2), -1/(3*T)],
  //                         [-1/(2*T**3), 1/(4*T**2)]])
  // b = np.matrix([[vxe - self.a1 - 2 * self.a2 * T],
  //               [axe - 2 * self.a2]])
  Eigen::Matrix2d a_2_inv;
  a_2_inv << 1 / std::pow(t, 2), -1 / (3 * t), -1 / (2 * std::pow(t, 3)), 1 / (4 * std::pow(t, 2));
  double target_s_velocity = current_s_velocity;
  Eigen::Vector2d b_2;
  b_2 << target_s_velocity - current_s_velocity, 0;
  Eigen::Vector2d x_2;
  x_2 = a_2_inv * b_2;

  // sampling points from calculated path
  double dt = sampling_delta_time_;
  std::vector<double> d_vec;
  double calculated_d, calculated_s;
  for (double i = 0; i < t; i += dt) {
    calculated_d = current_d_position + current_d_velocity * i + 0 * 2 * i * i +
                   x_3(0) * i * i * i + x_3(1) * i * i * i * i + x_3(2) * i * i * i * i * i;
    calculated_s = current_s_position + current_s_velocity * i + 2 * 0 * i * i +
                   x_2(0) * i * i * i + x_2(1) * i * i * i * i;

    geometry_msgs::msg::Pose tmp_point;
    if (calculated_s > spline2d.s.back()) {
      break;
    }
    std::array<double, 2> p = spline2d.calc_position(calculated_s);
    double yaw = spline2d.calc_yaw(calculated_s);
    tmp_point.position.x = p[0] + std::cos(yaw - M_PI / 2.0) * calculated_d;
    tmp_point.position.y = p[1] + std::sin(yaw - M_PI / 2.0) * calculated_d;
    tmp_point.position.z = height;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, yaw);
    tmp_point.orientation = tf2::toMsg(quat);
    path.path.push_back(tmp_point);
    if (path.path.size() >= path.path.max_size()) {
      break;
    }
  }
  path.confidence = calculateLikelihood(current_d_position);
  path.time_step = rclcpp::Duration::from_seconds(dt);

  return false;
}

void MapBasedPrediction::getLinearPredictedPath(
  const geometry_msgs::msg::Pose & object_pose, const geometry_msgs::msg::Twist & object_twist,
  autoware_auto_perception_msgs::msg::PredictedPath & predicted_path)
{
  const double & sampling_delta_time = sampling_delta_time_;
  const double & time_horizon = time_horizon_;
  const double ep = 0.001;

  for (double dt = 0.0; dt < time_horizon + ep; dt += sampling_delta_time) {
    geometry_msgs::msg::Pose tmp_pose;
    geometry_msgs::msg::Pose object_frame_pose;
    geometry_msgs::msg::Pose world_frame_pose;
    object_frame_pose.position.x = object_twist.linear.x * dt;
    object_frame_pose.position.y = object_twist.linear.y * dt;
    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, 0.0);
    object_frame_pose.orientation = tf2::toMsg(quat);
    tf2::Transform tf_object2future;
    tf2::Transform tf_world2object;
    tf2::Transform tf_world2future;

    tf2::fromMsg(object_pose, tf_world2object);
    tf2::fromMsg(object_frame_pose, tf_object2future);
    tf_world2future = tf_world2object * tf_object2future;
    tf2::toMsg(tf_world2future, world_frame_pose);
    tmp_pose = world_frame_pose;
    predicted_path.path.push_back(tmp_pose);
    if (predicted_path.path.size() >= predicted_path.path.max_size()) {
      break;
    }
  }

  predicted_path.confidence = 1.0;
  predicted_path.time_step = rclcpp::Duration::from_seconds(sampling_delta_time);
}

double MapBasedPrediction::calculateLikelihood(const double current_d)
{
  double d_std = 0.5;
  double likelihood = std::abs(current_d) / d_std;
  return likelihood;
}
