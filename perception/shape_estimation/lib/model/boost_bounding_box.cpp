// Copyright 2018 Autoware Foundation. All rights reserved.
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

#include "shape_estimation/model/boost_bounding_box.hpp"

#include <boost/math/tools/minima.hpp>

constexpr float epsilon = 0.001;

BoostBoundingBoxShapeModel::BoostBoundingBoxShapeModel() : BoundingBoxShapeModel() {}

BoostBoundingBoxShapeModel::BoostBoundingBoxShapeModel(const boost::optional<ReferenceYawInfo> & ref_yaw_info)
: BoundingBoxShapeModel(ref_yaw_info)
{
}

float BoostBoundingBoxShapeModel::optimize(const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const float min_angle, const float max_angle)
{
  // std::cerr << "Boost ! " << std::endl;
  auto closeness_func = [&](float theta) {
    Eigen::Vector2f e_1;
    e_1 << std::cos(theta), std::sin(theta);  // col.3, Algo.2
    Eigen::Vector2f e_2;
    e_2 << -std::sin(theta), std::cos(theta);  // col.4, Algo.2
    std::vector<float> C_1;                    // col.5, Algo.2
    std::vector<float> C_2;                    // col.6, Algo.2
    for (const auto & point : cluster) {
      C_1.push_back(point.x * e_1.x() + point.y * e_1.y());
      C_2.push_back(point.x * e_2.x() + point.y * e_2.y());
    }
    float q = calcClosenessCriterion(C_1, C_2);
    return -q;
  };

  int bits = 4;
  boost::uintmax_t max_iter = 20;
  std::pair<float, float> min = boost::math::tools::brent_find_minima(closeness_func, min_angle, max_angle+epsilon, bits, max_iter);
  float theta_star = min.first;
  return theta_star;
}
