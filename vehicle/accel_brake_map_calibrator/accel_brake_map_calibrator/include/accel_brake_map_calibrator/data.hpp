// Copyright 2023 Tier IV, Inc.
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

#ifndef ACCEL_BRAKE_MAP_CALIBRATOR__DATA_HPP_
#define ACCEL_BRAKE_MAP_CALIBRATOR__DATA_HPP_

#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace accel_brake_map_calibrator
{
struct DataStamped
{
  DataStamped(const double _data, const rclcpp::Time & _data_time)
  : data{_data}, data_time{_data_time}
  {
  }
  double data;
  rclcpp::Time data_time;
};
using DataStampedPtr = std::shared_ptr<DataStamped>;

}  // namespace accel_brake_map_calibrator

#endif  // ACCEL_BRAKE_MAP_CALIBRATOR__DATA_HPP_
