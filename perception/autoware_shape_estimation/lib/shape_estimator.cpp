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

#include "autoware/shape_estimation/shape_estimator.hpp"

#include "autoware/shape_estimation/corrector/corrector.hpp"
#include "autoware/shape_estimation/filter/filter.hpp"
#include "autoware/shape_estimation/model/model.hpp"

#include <iostream>
#include <memory>

namespace autoware::shape_estimation
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

ShapeEstimator::ShapeEstimator(bool use_corrector, bool use_filter, bool use_boost_bbox_optimizer)
: use_corrector_(use_corrector),
  use_filter_(use_filter),
  use_boost_bbox_optimizer_(use_boost_bbox_optimizer)
{
}

bool ShapeEstimator::estimateShapeAndPose(
  const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<ReferenceYawInfo> & ref_yaw_info,
  const boost::optional<ReferenceShapeSizeInfo> & ref_shape_size_info,
  const boost::optional<geometry_msgs::msg::Pose> & ref_pose,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  autoware_perception_msgs::msg::Shape shape;
  geometry_msgs::msg::Pose pose;
  // estimate shape
  bool reverse_to_unknown = false;
  if (!estimateOriginalShapeAndPose(label, cluster, ref_yaw_info, shape, pose)) {
    reverse_to_unknown = true;
  }

  // rule based filter
  if (use_filter_) {
    if (!applyFilter(label, shape, pose)) {
      reverse_to_unknown = true;
    }
  }

  // rule based corrector
  if (use_corrector_) {
    bool use_reference_yaw = ref_yaw_info ? true : false;
    if (!applyCorrector(label, use_reference_yaw, ref_shape_size_info, ref_pose, shape, pose)) {
      reverse_to_unknown = true;
    }
  }
  if (reverse_to_unknown) {
    estimateOriginalShapeAndPose(Label::UNKNOWN, cluster, ref_yaw_info, shape, pose);
    shape_output = shape;
    pose_output = pose;
    return false;
  }
  shape_output = shape;
  pose_output = pose;
  return true;
}

bool ShapeEstimator::estimateOriginalShapeAndPose(
  const uint8_t label, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const boost::optional<ReferenceYawInfo> & ref_yaw_info,
  autoware_perception_msgs::msg::Shape & shape_output, geometry_msgs::msg::Pose & pose_output)
{
  // estimate shape
  std::unique_ptr<model::ShapeEstimationModelInterface> model_ptr;
  if (
    label == Label::CAR || label == Label::TRUCK || label == Label::BUS ||
    label == Label::TRAILER || label == Label::MOTORCYCLE || label == Label::BICYCLE) {
    model_ptr.reset(new model::BoundingBoxShapeModel(ref_yaw_info, use_boost_bbox_optimizer_));
  } else if (label == Label::PEDESTRIAN) {
    model_ptr.reset(new model::CylinderShapeModel());
  } else {
    model_ptr.reset(new model::ConvexHullShapeModel());
  }

  return model_ptr->estimate(cluster, shape_output, pose_output);
}

bool ShapeEstimator::applyFilter(
  const uint8_t label, const autoware_perception_msgs::msg::Shape & shape,
  const geometry_msgs::msg::Pose & pose)
{
  std::unique_ptr<filter::ShapeEstimationFilterInterface> filter_ptr;
  if (label == Label::CAR) {
    filter_ptr.reset(new filter::CarFilter);
  } else if (label == Label::BUS) {
    filter_ptr.reset(new filter::BusFilter);
  } else if (label == Label::TRUCK) {
    filter_ptr.reset(new filter::TruckFilter);
  } else if (label == Label::TRAILER) {
    filter_ptr.reset(new filter::TrailerFilter);
  } else {
    filter_ptr.reset(new filter::NoFilter);
  }

  return filter_ptr->filter(shape, pose);
}

bool ShapeEstimator::applyCorrector(
  const uint8_t label, const bool use_reference_yaw,
  const boost::optional<ReferenceShapeSizeInfo> & ref_shape_size_info,
  const boost::optional<geometry_msgs::msg::Pose> & ref_pose,
  autoware_perception_msgs::msg::Shape & shape, geometry_msgs::msg::Pose & pose)
{
  std::unique_ptr<corrector::ShapeEstimationCorrectorInterface> corrector_ptr;

  if (ref_shape_size_info && use_reference_yaw) {
    if (ref_pose) {
      corrector_ptr.reset(new corrector::ReferenceObjectBasedVehicleCorrector(
        ref_shape_size_info.get(), ref_pose.get()));
    } else {
      corrector_ptr.reset(
        new corrector::ReferenceShapeBasedVehicleCorrector(ref_shape_size_info.get()));
    }
  } else if (label == Label::CAR) {
    corrector_ptr.reset(new corrector::CarCorrector(use_reference_yaw));
  } else if (label == Label::BUS) {
    corrector_ptr.reset(new corrector::BusCorrector(use_reference_yaw));
  } else if (label == Label::TRUCK) {
    corrector_ptr.reset(new corrector::TruckCorrector(use_reference_yaw));
  } else if (label == Label::TRAILER) {
    corrector_ptr.reset(new corrector::TrailerCorrector(use_reference_yaw));
  } else if (label == Label::MOTORCYCLE || label == Label::BICYCLE) {
    corrector_ptr.reset(new corrector::BicycleCorrector(use_reference_yaw));
  } else {
    corrector_ptr.reset(new corrector::NoCorrector);
  }

  return corrector_ptr->correct(shape, pose);
}

}  // namespace autoware::shape_estimation
