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

#ifndef SHAPE_ESTIMATION__MODEL__BOOST_BOUNDING_BOX_HPP_
#define SHAPE_ESTIMATION__MODEL__BOOST_BOUNDING_BOX_HPP_

#include "shape_estimation/model/bounding_box.hpp"

class BoostBoundingBoxShapeModel : public BoundingBoxShapeModel
{
private:
  float optimize(
    const pcl::PointCloud<pcl::PointXYZ> & cluster, const float min_angle, const float max_angle);

public:
  BoostBoundingBoxShapeModel();
  explicit BoostBoundingBoxShapeModel(const boost::optional<ReferenceYawInfo> & ref_yaw_info);

  ~BoostBoundingBoxShapeModel() {}
};

#endif  // SHAPE_ESTIMATION__MODEL__BOOST_BOUNDING_BOX_HPP_
