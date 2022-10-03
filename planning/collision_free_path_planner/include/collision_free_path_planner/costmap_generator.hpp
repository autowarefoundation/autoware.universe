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

#ifndef COLLISION_FREE_PATH_PLANNER__COSTMAP_GENERATOR_HPP_
#define COLLISION_FREE_PATH_PLANNER__COSTMAP_GENERATOR_HPP_

#include "collision_free_path_planner/common_structs.hpp"
#include "tier4_autoware_utils/system/stop_watch.hpp"

#include "boost/optional.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

namespace collision_free_path_planner
{
class CostmapGenerator
{
public:
  struct AvoidanceObjectParam
  {
    bool is_avoiding_unknown;
    bool is_avoiding_car;
    bool is_avoiding_truck;
    bool is_avoiding_bus;
    bool is_avoiding_bicycle;
    bool is_avoiding_motorbike;
    bool is_avoiding_pedestrian;
    bool is_avoiding_animal;

    double max_avoiding_ego_velocity_ms;
    double max_avoiding_objects_velocity_ms;
  };

  explicit CostmapGenerator(rclcpp::Node * node);
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  CVMaps getMaps(
    const PlannerData & planner_data, const TrajectoryParam & traj_param,
    DebugData & debug_data) const;

private:
  mutable tier4_autoware_utils::StopWatch<
    std::chrono::milliseconds, std::chrono::microseconds, std::chrono::steady_clock>
    stop_watch_;

  bool enable_pub_area_with_objects{false};
  bool enable_pub_object_clearance_map{false};
  bool enable_pub_clearance_map{false};

  AvoidanceObjectParam avoidance_object_param_{};

  rclcpp::Publisher<OccupancyGrid>::SharedPtr debug_clearance_map_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr debug_object_clearance_map_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr debug_area_with_objects_pub_;

  void initializeParam(rclcpp::Node * node);

  cv::Mat getAreaWithObjects(
    const cv::Mat & drivable_area, const cv::Mat & objects_image, DebugData & debug_data) const;

  cv::Mat getClearanceMap(const cv::Mat & drivable_area, DebugData & debug_data) const;

  cv::Mat drawObstaclesOnImage(
    const bool enable_avoidance,
    const std::vector<autoware_auto_perception_msgs::msg::PredictedObject> & objects,
    const std::vector<autoware_auto_planning_msgs::msg::PathPoint> & path_points,
    const nav_msgs::msg::MapMetaData & map_info, [[maybe_unused]] const cv::Mat & drivable_area,
    const cv::Mat & clearance_map, const TrajectoryParam & traj_param,
    std::vector<autoware_auto_perception_msgs::msg::PredictedObject> * debug_avoiding_objects,
    DebugData & debug_data) const;

  cv::Mat getDrivableAreaInCV(
    const nav_msgs::msg::OccupancyGrid & occupancy_grid, DebugData & debug_data) const;

  void publishDebugMaps(const Path & path, const CVMaps & cv_maps, DebugData & debug_data) const;
};
}  // namespace collision_free_path_planner
#endif  // COLLISION_FREE_PATH_PLANNER__COSTMAP_GENERATOR_HPP_
