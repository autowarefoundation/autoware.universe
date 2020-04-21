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
#include "corrector/bus_corrector.hpp"
#include "corrector/car_corrector.hpp"
#include "corrector/no_corrector.hpp"
#include "corrector/truck_corrector.hpp"
#include "filter/bus_filter.hpp"
#include "filter/car_filter.hpp"
#include "filter/no_filter.hpp"
#include "filter/truck_filter.hpp"
#include "model/bounding_box.hpp"
#include "model/convex_hull.hpp"
#include "model/cylinder.hpp"
#include "shape_estimation/model_interface.hpp"

ShapeEstimator::ShapeEstimator() {}

bool ShapeEstimator::getShapeAndPose(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output,
  bool & orientaion_output)
{
  // check input
  if (cluster.empty()) return false;

  autoware_perception_msgs::Shape shape;
  geometry_msgs::Pose pose;
  bool orientation;

  // estimate shape
  if (!estimateShape(type, cluster, shape, pose, orientation)) {
    return false;
  }

  // rule based filter
  if (!applyFilter(type, shape, pose, orientation)) {
    return false;
  }

  // rule based corrector
  if (!applyCorrector(type, shape, pose, orientation)) {
    return false;
  }

  shape_output = shape;
  pose_output = pose;
  orientaion_output = orientation;
  return true;
}

bool ShapeEstimator::estimateShape(
  const int type, const pcl::PointCloud<pcl::PointXYZ> & cluster,
  autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output,
  bool & orientaion_output)
{
  // estimate shape
  std::unique_ptr<ShapeEstimationModelInterface> model_ptr;
  if (
    type == autoware_perception_msgs::Semantic::CAR ||
    type == autoware_perception_msgs::Semantic::TRUCK ||
    type == autoware_perception_msgs::Semantic::BUS) {
    model_ptr.reset(new BoundingBoxModel);
  } else if (type == autoware_perception_msgs::Semantic::PEDESTRIAN) {
    model_ptr.reset(new CylinderModel);
  } else if (type == autoware_perception_msgs::Semantic::MOTORBIKE) {
    model_ptr.reset(new BoundingBoxModel);
  } else if (type == autoware_perception_msgs::Semantic::BICYCLE) {
    model_ptr.reset(new BoundingBoxModel);
  } else {
    model_ptr.reset(new ConvexHullModel);
  }

  return model_ptr->estimate(cluster, shape_output, pose_output, orientaion_output);
}

bool ShapeEstimator::applyFilter(
  const int type, const autoware_perception_msgs::Shape & shape_output,
  const geometry_msgs::Pose & pose_output, const bool & orientaion_output)
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

  return filter_ptr->filter(shape_output, pose_output, orientaion_output);
}

bool ShapeEstimator::applyCorrector(
  const int type, autoware_perception_msgs::Shape & shape_output, geometry_msgs::Pose & pose_output,
  bool & orientaion_output)
{
  std::unique_ptr<ShapeEstimationCorrectorInterface> corrector_ptr;
  if (type == autoware_perception_msgs::Semantic::CAR) {
    corrector_ptr.reset(new CarCorrector);
  } else if (type == autoware_perception_msgs::Semantic::BUS) {
    corrector_ptr.reset(new BusCorrector);
  } else if (type == autoware_perception_msgs::Semantic::TRUCK) {
    corrector_ptr.reset(new TruckCorrector);
  } else {
    corrector_ptr.reset(new NoCorrector);
  }

  return corrector_ptr->correct(shape_output, pose_output, orientaion_output);
}
