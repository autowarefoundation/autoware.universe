// Copyright 2023 TIER IV, Inc.
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

#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace yabloc
{
class InitArea
{
public:
  using point = boost::geometry::model::d2::point_xy<double>;
  using polygon = boost::geometry::model::polygon<point>;

  InitArea(const sensor_msgs::msg::PointCloud2 & msg);

  // Return whether it is inside of init/deinit area or not
  bool is_inside(const Eigen::Vector3d & xyz, bool * init_area) const;

private:
  std::vector<polygon> init_areas_;
  std::vector<polygon> deinit_areas_;
};

}  // namespace yabloc