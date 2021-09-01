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

#include "shape_estimation/shape_estimator.hpp"
#include <iostream>
#include <memory>
#include "shape_estimation/corrector/corrector.hpp"
#include "shape_estimation/filter/filter.hpp"
#include "shape_estimation/model/model.hpp"

ShapeEstimator::ShapeEstimator(bool use_corrector, bool use_filter)
: use_corrector_(use_corrector), use_filter_(use_filter)
{
}

bool ShapeEstimator::estimateShapeAndPose(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<float> & yaw,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;
  // estimate shape
  if (!estimateShape(type, cluster, yaw, shape, pose)) {
    return false;
  }

  // rule based filter
  if (use_filter_) {
    if (!applyFilter(type, shape, pose)) {
      return false;
    }
  }

  // rule based corrector
  if (use_corrector_) {
    bool use_reference_yaw = yaw ? true : false;
    if (!applyCorrector(type, use_reference_yaw, shape, pose)) {
      return false;
    }
  }

  shape_output = shape;
  pose_output = pose;
  return true;
}

bool ShapeEstimator::estimateShape(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<float> & yaw, autoware_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  // estimate shape
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  if (
    type == autoware_perception_msgs::msg::Semantic::CAR ||
    type == autoware_perception_msgs::msg::Semantic::TRUCK ||
    type == autoware_perception_msgs::msg::Semantic::BUS)
  {
    model_ptr.reset(new BoundingBoxShapeModel(yaw));
  } else if (type == autoware_perception_msgs::msg::Semantic::PEDESTRIAN) {
    model_ptr.reset(new CylinderShapeModel());
  } else if (type == autoware_perception_msgs::msg::Semantic::MOTORBIKE) {
    model_ptr.reset(new BoundingBoxShapeModel(yaw));
  } else if (type == autoware_perception_msgs::msg::Semantic::BICYCLE) {
    model_ptr.reset(new BoundingBoxShapeModel(yaw));
  } else {
    model_ptr.reset(new ConvexhullShapeModel());
  }

  return model_ptr->estimate(cluster, shape_output, pose_output);
}

bool ShapeEstimator::applyFilter(
  const int type, const autoware_perception_msgs::msg::Shape & shape_output,
  const geometry_msgs::msg::Pose & pose_output)
{
  std::unique_ptr<ShapeEstimationFilterInterface> filter_ptr;
  if (type == autoware_perception_msgs::msg::Semantic::CAR) {
    filter_ptr.reset(new CarFilter);
  } else if (type == autoware_perception_msgs::msg::Semantic::BUS) {
    filter_ptr.reset(new BusFilter);
  } else if (type == autoware_perception_msgs::msg::Semantic::TRUCK) {
    filter_ptr.reset(new TruckFilter);
  } else {
    filter_ptr.reset(new NoFilter);
  }

  return filter_ptr->filter(shape_output, pose_output);
}

bool ShapeEstimator::applyCorrector(
  const int type, const bool use_reference_yaw, autoware_perception_msgs::msg::Shape & shape_output,
  geometry_msgs::msg::Pose & pose_output)
{
  std::unique_ptr<ShapeEstimationCorrectorInterface> corrector_ptr;
  if (type == autoware_perception_msgs::msg::Semantic::CAR) {
    corrector_ptr.reset(new CarCorrector(use_reference_yaw));
  } else if (type == autoware_perception_msgs::msg::Semantic::BUS) {
    corrector_ptr.reset(new BusCorrector(use_reference_yaw));
  } else if (type == autoware_perception_msgs::msg::Semantic::TRUCK) {
    corrector_ptr.reset(new TruckCorrector(use_reference_yaw));
  } else {
    corrector_ptr.reset(new NoCorrector);
  }

  return corrector_ptr->correct(shape_output, pose_output);
}
