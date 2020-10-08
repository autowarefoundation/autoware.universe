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

#include "shape_estimation/shape_estimator.hpp"
#include <iostream>
#include <memory>
#include "autoware_perception_msgs/Semantic.h"
#include "corrector/normal/bus_corrector.hpp"
#include "corrector/normal/car_corrector.hpp"
#include "corrector/normal/no_corrector.hpp"
#include "corrector/normal/truck_corrector.hpp"
#include "corrector/yaw_fixed/bus_corrector.hpp"
#include "corrector/yaw_fixed/car_corrector.hpp"
#include "corrector/yaw_fixed/no_corrector.hpp"
#include "corrector/yaw_fixed/truck_corrector.hpp"
#include "filter/bus_filter.hpp"
#include "filter/car_filter.hpp"
#include "filter/no_filter.hpp"
#include "filter/truck_filter.hpp"
#include "model/normal/bounding_box.hpp"
#include "model/normal/convex_hull.hpp"
#include "model/normal/cylinder.hpp"
#include "model/yaw_fixed/bounding_box.hpp"
#include "shape_estimation/model_interface.hpp"

ShapeEstimator::ShapeEstimator()
  : ShapeEstimator::ShapeEstimator(3, true, true){}

ShapeEstimator::ShapeEstimator(double l_shape_fitting_search_angle_range, bool use_corrector, bool orientation_reliable)
  : l_shape_fitting_search_angle_range_(l_shape_fitting_search_angle_range),
    use_corrector_(use_corrector),
    orientation_reliable_(orientation_reliable){}

bool ShapeEstimator::getShapeAndPose(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output)
{
  autoware_perception_msgs::Shape shape;
  geometry_msgs::Pose pose;
  if (!process(type, cluster, shape, pose)) {
    return false;
  }
  shape_output = shape;
  pose_output = pose;
  return true;
}

bool ShapeEstimator::getShapeAndPose(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  const autoware_perception_msgs::State & state,
  autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output)
{
  autoware_perception_msgs::Shape shape;
  geometry_msgs::Pose pose = state.pose_covariance.pose;
  if (!process(type, cluster, shape, pose)) {
    return false;
  }
  shape_output = shape;
  pose_output = pose;
  return true;
}

bool ShapeEstimator::process(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::Shape & shape, geometry_msgs::Pose & pose)
{
  // check input
  if (cluster.empty()) return false;

  // estimate shape
  if (!estimateShape(type, cluster, shape, pose)) {
    return false;
  }

  // rule based filter
  if (!applyFilter(type, shape, pose)) {
    return false;
  }

  // rule based corrector
  if (use_corrector_) {
    if (!applyCorrector(type, shape, pose)) {
      return false;
    }
  }

  return true;
}

bool ShapeEstimator::estimateShape(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output)
{
  // estimate shape
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  if (
    type == autoware_perception_msgs::Semantic::CAR ||
    type == autoware_perception_msgs::Semantic::TRUCK ||
    type == autoware_perception_msgs::Semantic::BUS) {
    if (orientation_reliable_) {
      model_ptr.reset(new yaw_fixed::BoundingBoxModel(l_shape_fitting_search_angle_range_));
    } else {
      model_ptr.reset(new normal::BoundingBoxModel);
    }
  } else if (type == autoware_perception_msgs::Semantic::PEDESTRIAN) {
    model_ptr.reset(new normal::CylinderModel);
  } else if (type == autoware_perception_msgs::Semantic::MOTORBIKE) {
    model_ptr.reset(new normal::BoundingBoxModel);
  } else if (type == autoware_perception_msgs::Semantic::BICYCLE) {
    model_ptr.reset(new normal::BoundingBoxModel);
  } else {
    model_ptr.reset(new normal::ConvexHullModel);
  }

  return model_ptr->estimate(cluster, shape_output, pose_output);
}

bool ShapeEstimator::applyFilter(
  const int type, const autoware_perception_msgs::Shape & shape_output,
  const geometry_msgs::Pose & pose_output)
{
  std::unique_ptr<ShapeEstimationFilterInterface> filter_ptr;
  if (type == autoware_perception_msgs::Semantic::CAR) {
    filter_ptr.reset(new CarFilter);
  } else if (type == autoware_perception_msgs::Semantic::BUS) {
    filter_ptr.reset(new BusFilter);
  } else if (type == autoware_perception_msgs::Semantic::TRUCK) {
    filter_ptr.reset(new TruckFilter);
  } else {
    filter_ptr.reset(new NoFilter);
  }

  return filter_ptr->filter(shape_output, pose_output);
}

bool ShapeEstimator::applyCorrector(
  const int type, autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output)
{
  std::unique_ptr<ShapeEstimationCorrectorInterface> corrector_ptr;
  if (type == autoware_perception_msgs::Semantic::CAR) {
    if (orientation_reliable_)
      corrector_ptr.reset(new yaw_fixed::CarCorrector);
    else
      corrector_ptr.reset(new normal::CarCorrector);
  } else if (type == autoware_perception_msgs::Semantic::BUS) {
    if (orientation_reliable_)
      corrector_ptr.reset(new yaw_fixed::BusCorrector);
    else
      corrector_ptr.reset(new normal::BusCorrector);
  } else if (type == autoware_perception_msgs::Semantic::TRUCK) {
    if (orientation_reliable_)
      corrector_ptr.reset(new yaw_fixed::TruckCorrector);
    else
      corrector_ptr.reset(new normal::TruckCorrector);
  } else {
    if (orientation_reliable_)
      corrector_ptr.reset(new yaw_fixed::NoCorrector);
    else
      corrector_ptr.reset(new normal::NoCorrector);
  }

  return corrector_ptr->correct(shape_output, pose_output);
}
