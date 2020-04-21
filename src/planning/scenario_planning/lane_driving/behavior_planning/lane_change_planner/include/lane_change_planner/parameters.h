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

#ifndef LANE_CHANGE_PLANNER_PARAMETERS_H
#define LANE_CHANGE_PLANNER_PARAMETERS_H

struct LaneChangerParameters
{
  double min_stop_distance;
  double stop_time;
  double hysteresis_buffer_distance;
  double backward_path_length;
  double forward_path_length;
  double lane_change_prepare_duration;
  double lane_changing_duration;
  double minimum_lane_change_length;
  double prediction_duration;
  double prediction_time_resolution;
  double drivable_area_resolution;
  double drivable_area_width;
  double drivable_area_height;
  double vehicle_width;
  double static_obstacle_velocity_thresh;
  bool enable_abort_lane_change;
};

#endif
