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
// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef LANE_DEPARTURE_CHECKER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_
#define LANE_DEPARTURE_CHECKER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/ros/vehicle_info.hpp"

inline autoware_utils::LinearRing2d createVehicleFootprint(
  const VehicleInfo & vehicle_info, const double margin = 0.0)
{
  using autoware_utils::LinearRing2d;
  using autoware_utils::Point2d;

  const auto & i = vehicle_info;

  const double x_front = i.front_overhang + i.wheel_base + margin;
  const double x_center = i.wheel_base / 2.0;
  const double x_rear = -(i.rear_overhang + margin);
  const double y_left = i.wheel_tread / 2.0 + i.left_overhang + margin;
  const double y_right = -(i.wheel_tread / 2.0 + i.right_overhang + margin);

  LinearRing2d footprint;
  footprint.push_back(Point2d{x_front, y_left});
  footprint.push_back(Point2d{x_front, y_right});
  footprint.push_back(Point2d{x_center, y_right});
  footprint.push_back(Point2d{x_rear, y_right});
  footprint.push_back(Point2d{x_rear, y_left});
  footprint.push_back(Point2d{x_center, y_left});
  footprint.push_back(Point2d{x_front, y_left});

  return footprint;
}

#endif  // LANE_DEPARTURE_CHECKER__UTIL__CREATE_VEHICLE_FOOTPRINT_HPP_
