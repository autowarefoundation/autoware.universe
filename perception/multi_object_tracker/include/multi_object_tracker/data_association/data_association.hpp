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
 * v1.0 Yukihiro Saito
 */

#pragma once
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <list>
#include <unordered_map>
#include <vector>
#include "multi_object_tracker/tracker/tracker.hpp"
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
class DataAssociation
{
private:
  double getDistance(
    const geometry_msgs::Point & measurement, const geometry_msgs::Point & tracker);
  Eigen::MatrixXi can_assgin_matrix_;
  Eigen::MatrixXd max_dist_matrix_;
  Eigen::MatrixXd max_area_matrix_;
  Eigen::MatrixXd min_area_matrix_;
  const double score_threshold_;

public:
  DataAssociation();
  bool assign(
    const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
    std::unordered_map<int, int> & reverse_assignment);
  Eigen::MatrixXd calcScoreMatrix(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);
  virtual ~DataAssociation(){};
};
