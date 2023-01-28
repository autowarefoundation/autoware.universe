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

#include "accel_brake_map_calibrator/utility.hpp"

namespace accel_brake_map_calibrator
{

bool isTimeout(
  const builtin_interfaces::msg::Time & stamp, const double timeout_sec, const rclcpp::Time & now)
{
  const double dt = now.seconds() - rclcpp::Time(stamp).seconds();
  return dt > timeout_sec;
}

bool isTimeout(
  const DataStampedPtr & data_stamped, const double timeout_sec, const rclcpp::Time & now)
{
  const double dt = (now - data_stamped->data_time).seconds();
  return dt > timeout_sec;
}

double lowpass(const double original, const double current, const double gain)
{
  return current * gain + original * (1.0 - gain);
}

void pushDataToQue(
  const TwistStamped::ConstSharedPtr & data, const std::size_t max_size,
  std::queue<TwistStamped::ConstSharedPtr> * que)
{
  que->push(data);
  while (que->size() > max_size) {
    que->pop();
  }
}

DataStampedPtr getNearestTimeDataFromVec(
  DataStampedPtr base_data, const double back_time, const std::vector<DataStampedPtr> & vec)
{
  double nearest_time = std::numeric_limits<double>::max();
  const double target_time = base_data->data_time.seconds() - back_time;
  DataStampedPtr nearest_time_data;
  for (const auto & data : vec) {
    const double data_time = data->data_time.seconds();
    const auto delta_time = std::abs(target_time - data_time);
    if (nearest_time > delta_time) {
      nearest_time_data = data;
      nearest_time = delta_time;
    }
  }
  return nearest_time_data;
}

double getAverage(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  double sum = 0.0;
  for (const auto num : vec) {
    sum += num;
  }
  return sum / vec.size();
}

double getStandardDeviation(const std::vector<double> & vec)
{
  if (vec.empty()) {
    return 0.0;
  }

  const double ave = getAverage(vec);

  double sum = 0.0;
  for (const auto num : vec) {
    sum += std::pow(num - ave, 2);
  }
  return std::sqrt(sum / vec.size());
}

OccupancyGrid getOccMsg(
  const std::string frame_id, const double height, const double width, const double resolution,
  const std::vector<int8_t> & map_value, const rclcpp::Time & stamp)
{
  OccupancyGrid occ;
  occ.header.frame_id = frame_id;
  occ.header.stamp = stamp;
  occ.info.height = height;
  occ.info.width = width;
  occ.info.map_load_time = stamp;
  occ.info.origin.position.x = 0;
  occ.info.origin.position.y = 0;
  occ.info.origin.position.z = 0;
  occ.info.origin.orientation.x = 0;
  occ.info.origin.orientation.y = 0;
  occ.info.origin.orientation.z = 0;
  occ.info.origin.orientation.w = 1;
  occ.info.resolution = resolution;
  occ.data = map_value;
  return occ;
}

}  // namespace accel_brake_map_calibrator
