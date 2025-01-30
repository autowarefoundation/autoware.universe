// Copyright 2025 Tier IV, Inc.
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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/find_intervals.hpp"

#include <matplotlibcpp17/pyplot.h>

#include <vector>

using Trajectory = autoware::trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId>;

tier4_planning_msgs::msg::PathPointWithLaneId path_point_with_lane_id(
  double x, double y, uint8_t lane_id)
{
  tier4_planning_msgs::msg::PathPointWithLaneId point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.lane_ids.emplace_back(lane_id);
  return point;
}

int main()
{
  pybind11::scoped_interpreter guard{};
  auto plt = matplotlibcpp17::pyplot::import();

  std::vector<tier4_planning_msgs::msg::PathPointWithLaneId> points{
    path_point_with_lane_id(0.41, 0.69, 0), path_point_with_lane_id(0.66, 1.09, 0),
    path_point_with_lane_id(0.93, 1.41, 0), path_point_with_lane_id(1.26, 1.71, 0),
    path_point_with_lane_id(1.62, 1.90, 0), path_point_with_lane_id(1.96, 1.98, 0),
    path_point_with_lane_id(2.48, 1.96, 1), path_point_with_lane_id(3.02, 1.87, 1),
    path_point_with_lane_id(3.56, 1.82, 1), path_point_with_lane_id(4.14, 2.02, 1),
    path_point_with_lane_id(4.56, 2.36, 1), path_point_with_lane_id(4.89, 2.72, 1),
    path_point_with_lane_id(5.27, 3.15, 1), path_point_with_lane_id(5.71, 3.69, 1),
    path_point_with_lane_id(6.09, 4.02, 0), path_point_with_lane_id(6.54, 4.16, 0),
    path_point_with_lane_id(6.79, 3.92, 0), path_point_with_lane_id(7.11, 3.60, 0),
    path_point_with_lane_id(7.42, 3.01, 0)};

  auto trajectory = Trajectory::Builder{}.build(points);

  if (!trajectory) {
    return 1;
  }

  const auto intervals = autoware::trajectory::find_intervals(
    *trajectory, [](const tier4_planning_msgs::msg::PathPointWithLaneId & point) {
      return point.lane_ids[0] == 1;
    });

  std::vector<double> x_id0;
  std::vector<double> y_id0;
  std::vector<double> x_id1;
  std::vector<double> y_id1;
  std::vector<double> x_all;
  std::vector<double> y_all;

  for (const auto & point : points) {
    if (point.lane_ids[0] == 0) {
      x_id0.push_back(point.point.pose.position.x);
      y_id0.push_back(point.point.pose.position.y);
    } else {
      x_id1.push_back(point.point.pose.position.x);
      y_id1.push_back(point.point.pose.position.y);
    }
  }

  for (double s = 0.0; s < trajectory->length(); s += 0.01) {
    auto p = trajectory->compute(s);
    x_all.push_back(p.point.pose.position.x);
    y_all.push_back(p.point.pose.position.y);
  }

  plt.plot(Args(x_all, y_all), Kwargs("color"_a = "blue"));
  plt.scatter(Args(x_id0, y_id0), Kwargs("color"_a = "blue", "label"_a = "lane_id = 0"));
  plt.scatter(Args(x_id1, y_id1), Kwargs("color"_a = "red", "label"_a = "lane_id = 1"));

  std::vector<double> x_cropped;
  std::vector<double> y_cropped;

  trajectory->crop(intervals[0].start, intervals[0].end - intervals[0].start);

  for (double s = 0.0; s < trajectory->length(); s += 0.01) {
    auto p = trajectory->compute(s);
    x_cropped.push_back(p.point.pose.position.x);
    y_cropped.push_back(p.point.pose.position.y);
  }

  plt.plot(
    Args(x_cropped, y_cropped),
    Kwargs("color"_a = "red", "label"_a = "interval between lane_id = 1"));

  plt.legend();
  plt.show();

  return 0;
}
