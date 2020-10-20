/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 *
 * v1.0 Yukihiro Saito
 */

#pragma once

#include "shape_estimation/model_interface.hpp"

namespace normal
{
class BoundingBoxModel : public ShapeEstimationModelInterface
{
private:
  double calcClosenessCriterion(const std::vector<double> & C_1, const std::vector<double> & C_2);

public:
  BoundingBoxModel(){};

  ~BoundingBoxModel(){};

  /*
   * minimum cluster size is 2.
   */
  bool estimate(
    const pcl::PointCloud<pcl::PointXYZ> & cluster,
    autoware_perception_msgs::msg::Shape & shape_output,
    geometry_msgs::msg::Pose & pose_output) override;
};
}  // namespace normal
