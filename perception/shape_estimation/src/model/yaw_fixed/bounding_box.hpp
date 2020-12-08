// Copyright 2020 TierIV
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

#include "shape_estimation/model_interface.hpp"

namespace yaw_fixed
{
class BoundingBoxModel : public ShapeEstimationModelInterface
{
private:
  double calcClosenessCriterion(const std::vector<double> & C_1, const std::vector<double> & C_2);
  double l_shape_fitting_search_angle_range_;

public:
  BoundingBoxModel()
  : l_shape_fitting_search_angle_range_(3) {}

  BoundingBoxModel(double l_shape_fitting_search_angle_range)
  : l_shape_fitting_search_angle_range_(l_shape_fitting_search_angle_range) {}

  ~BoundingBoxModel() {}

  /*
   * minimum cluster size is 2.
   */
  bool estimate(
    const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output) override;
};
}  // namespace yaw_fixed
