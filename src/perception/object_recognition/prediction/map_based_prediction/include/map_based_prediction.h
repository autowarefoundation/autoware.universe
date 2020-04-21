/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef MAP_BASED_PREDICTION_H
#define MAP_BASED_PREDICTION_H

namespace autoware_perception_msgs
{
ROS_DECLARE_MESSAGE(DynamicObjectArray);
ROS_DECLARE_MESSAGE(DynamicObject);
ROS_DECLARE_MESSAGE(PredictedPath);
}  // namespace autoware_perception_msgs

namespace geometry_msgs
{
ROS_DECLARE_MESSAGE(Point);
ROS_DECLARE_MESSAGE(Pose);
}  // namespace geometry_msgs

struct DynamicObjectWithLanes
{
  autoware_perception_msgs::DynamicObject object;
  std::vector<std::vector<geometry_msgs::Pose>> lanes;
};

struct DynamicObjectWithLanesArray
{
  std_msgs::Header header;
  std::vector<DynamicObjectWithLanes> objects;
};

class Spline2D;

class MapBasedPrediction
{
private:
  double interpolating_resolution_;
  double time_horizon_;
  double sampling_delta_time_;

  bool getPredictedPath(
    const double height, const double current_d_position, const double current_d_velocity,
    const double current_s_position, const double current_s_velocity,
    const double target_s_position, const std_msgs::Header & origin_header, Spline2D & spline2d,
    autoware_perception_msgs::PredictedPath & path);

  bool getLinearPredictedPath(
    const geometry_msgs::Pose & object_pose, const geometry_msgs::Twist & object_twist,
    const std_msgs::Header & origin_header,
    autoware_perception_msgs::PredictedPath & predicted_path);

  // double calculateLikelyhood(const double desired_yaw, const double current_d, const double current_yaw);
  double calculateLikelyhood(const double current_d);

  bool normalizeLikelyhood(std::vector<autoware_perception_msgs::PredictedPath> & paths);

public:
  MapBasedPrediction(
    double interpolating_resolution, double time_horizon, double sampling_delta_time);

  bool doPrediction(
    const DynamicObjectWithLanesArray & in_objects,
    std::vector<autoware_perception_msgs::DynamicObject> & out_objects,
    std::vector<geometry_msgs::Point> & debug_interpolated_points);

  bool doLinearPrediction(
    const autoware_perception_msgs::DynamicObjectArray & in_objects,
    std::vector<autoware_perception_msgs::DynamicObject> & out_objects);
};

#endif  // MAP_BASED_PREDICTION_H
