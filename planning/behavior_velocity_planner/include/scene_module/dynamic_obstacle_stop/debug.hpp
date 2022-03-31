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
#ifndef DYNAMIC_OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
#define DYNAMIC_OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_

#include "scene_module/dynamic_obstacle_stop/utils.hpp"

#include <rclcpp/rclcpp.hpp>

#include <tier4_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>

#include <memory>
#include <string>
#include <vector>
namespace behavior_velocity_planner
{
using dynamic_obstacle_stop_utils::TextWithPosition;
using tier4_debug_msgs::msg::Float32MultiArrayStamped;

enum class PointType : int8_t {
  Blue = 0,
  Red,
  Yellow,
};

class DebugValues
{
public:
  enum class TYPE {
    CALCULATION_TIME = 0,
    LATERAL_DIST = 1,
    LONGITUDINAL_DIST_OBSTACLE = 2,
    LONGITUDINAL_DIST_COLLISION = 3,
    COLLISION_POS_FROM_EGO_FRONT = 4,
    STOP_DISTANCE = 5,
    SIZE,  // this is the number of enum elements
  };

  /**
   * @brief get the index corresponding to the given value TYPE
   * @param [in] type the TYPE enum for which to get the index
   * @return index of the type
   */
  int getValuesIdx(const TYPE type) const { return static_cast<int>(type); }
  /**
   * @brief get all the debug values as an std::array
   * @return array of all debug values
   */
  std::array<float, static_cast<int>(TYPE::SIZE)> getValues() const { return values_; }
  /**
   * @brief set the given type to the given value
   * @param [in] type TYPE of the value
   * @param [in] value value to set
   */
  void setValues(const TYPE type, const float val) { values_.at(static_cast<int>(type)) = val; }
  /**
   * @brief set the given type to the given value
   * @param [in] type index of the type
   * @param [in] value value to set
   */
  void setValues(const int type, const float val) { values_.at(type) = val; }

private:
  static constexpr int num_debug_values_ = static_cast<int>(TYPE::SIZE);
  std::array<float, static_cast<int>(TYPE::SIZE)> values_;
};

class DynamicObstacleStopDebug
{
public:
  explicit DynamicObstacleStopDebug(rclcpp::Node & node);
  ~DynamicObstacleStopDebug() {}

  void setDebugValues(const DebugValues::TYPE type, const double val)
  {
    debug_values_.setValues(type, val);
  }

  bool pushObstaclePoint(const geometry_msgs::msg::Point & obstacle_point, const PointType & type);
  bool pushObstaclePoint(const pcl::PointXYZ & obstacle_point, const PointType & type);
  void pushDebugPoints(const pcl::PointXYZ & debug_point);
  void pushDebugPoints(const geometry_msgs::msg::Point & debug_point);
  void pushDebugPoints(const std::vector<geometry_msgs::msg::Point> & debug_points);
  void pushDebugPoints(const geometry_msgs::msg::Point & debug_point, const PointType point_type);
  void pushStopPose(const geometry_msgs::msg::Pose & pose);
  void pushDebugLines(const std::vector<geometry_msgs::msg::Point> & debug_line);
  void pushDebugPolygons(const std::vector<geometry_msgs::msg::Point> & debug_polygon);
  void pushDebugTexts(const TextWithPosition & debug_text);
  void publishDebugValue();
  visualization_msgs::msg::MarkerArray createVisualizationMarkerArray();

private:
  visualization_msgs::msg::MarkerArray createVisualizationMarkerArrayFromDebugData(const builtin_interfaces::msg::Time & current_time);
  void clearDebugMarker();

  rclcpp::Node & node_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_values_;
  std::vector<geometry_msgs::msg::Point> debug_points_;
  std::vector<geometry_msgs::msg::Point> debug_points_red_;
  std::vector<geometry_msgs::msg::Point> debug_points_yellow_;
  boost::optional<geometry_msgs::msg::Pose> stop_pose_{};
  std::vector<std::vector<geometry_msgs::msg::Point>> debug_lines_;
  std::vector<std::vector<geometry_msgs::msg::Point>> debug_polygons_;
  std::vector<TextWithPosition> debug_texts_;
  DebugValues debug_values_;
};

}  // namespace behavior_velocity_planner

#endif  // DYNAMIC_OBSTACLE_STOP_PLANNER__DEBUG_MARKER_HPP_
