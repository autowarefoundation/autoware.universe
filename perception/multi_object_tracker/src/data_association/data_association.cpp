// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#include <algorithm>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "multi_object_tracker/data_association/data_association.hpp"
#include "successive_shortest_path/successive_shortest_path.hpp"
#include "multi_object_tracker/utils/utils.hpp"

DataAssociation::DataAssociation(
  std::vector<int> can_assign_vector, std::vector<double> max_dist_vector,
  std::vector<double> max_area_vector, std::vector<double> min_area_vector)
: score_threshold_(0.01)
{
  {
    const int assign_label_num = static_cast<int>(std::sqrt(can_assign_vector.size()));
    Eigen::Map<Eigen::MatrixXi> can_assign_matrix_tmp(
      can_assign_vector.data(), assign_label_num, assign_label_num);
    can_assign_matrix_ = can_assign_matrix_tmp.transpose();
  }
  {
    const int max_dist_label_num = static_cast<int>(std::sqrt(max_dist_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_dist_matrix_tmp(
      max_dist_vector.data(), max_dist_label_num, max_dist_label_num);
    max_dist_matrix_ = max_dist_matrix_tmp.transpose();
  }
  {
    const int max_area_label_num = static_cast<int>(std::sqrt(max_area_vector.size()));
    Eigen::Map<Eigen::MatrixXd> max_area_matrix_tmp(
      max_area_vector.data(), max_area_label_num, max_area_label_num);
    max_area_matrix_ = max_area_matrix_tmp.transpose();
  }
  {
    const int min_area_label_num = static_cast<int>(std::sqrt(min_area_vector.size()));
    Eigen::Map<Eigen::MatrixXd> min_area_matrix_tmp(
      min_area_vector.data(), min_area_label_num, min_area_label_num);
    min_area_matrix_ = min_area_matrix_tmp.transpose();
  }
}

void DataAssociation::assign(
  const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment)
{
  std::vector<std::vector<double>> score(src.rows());
  for (int row = 0; row < src.rows(); ++row) {
    score.at(row).resize(src.cols());
    for (int col = 0; col < src.cols(); ++col) {
      score.at(row).at(col) = src(row, col);
    }
  }
  // Solve
  assignment_problem::MaximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  for (auto itr = direct_assignment.begin(); itr != direct_assignment.end(); ) {
    if (src(itr->first, itr->second) < score_threshold_) {
      itr = direct_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
  for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end(); ) {
    if (src(itr->second, itr->first) < score_threshold_) {
      itr = reverse_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
}

Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(trackers.size(), measurements.feature_objects.size());
  size_t tracker_idx = 0;
  for (auto tracker_itr = trackers.begin(); tracker_itr != trackers.end();
    ++tracker_itr, ++tracker_idx)
  {
    for (size_t measurement_idx = 0; measurement_idx < measurements.feature_objects.size();
      ++measurement_idx)
    {
      double score = 0.0;
      if (can_assign_matrix_(
          (*tracker_itr)->getType(),
          measurements.feature_objects.at(measurement_idx).object.semantic.type))
      {
        double max_dist = max_dist_matrix_(
          (*tracker_itr)->getType(),
          measurements.feature_objects.at(measurement_idx).object.semantic.type);
        double max_area = max_area_matrix_(
          (*tracker_itr)->getType(),
          measurements.feature_objects.at(measurement_idx).object.semantic.type);
        double min_area = min_area_matrix_(
          (*tracker_itr)->getType(),
          measurements.feature_objects.at(measurement_idx).object.semantic.type);
        double dist = getDistance(
          measurements.feature_objects.at(measurement_idx)
          .object.state.pose_covariance.pose.position,
          (*tracker_itr)->getPosition(measurements.header.stamp));
        double area = utils::getArea(measurements.feature_objects.at(measurement_idx).object.shape);
        score = (max_dist - std::min(dist, max_dist)) / max_dist;

        // dist gate
        if (max_dist < dist) {score = 0.0;}
        // area gate
        if (area < min_area || max_area < area) {score = 0.0;}
        // mahalanobis dist gate
        if (score < score_threshold_) {
          double mahalanobis_dist = getMahalanobisDistance(
            measurements.feature_objects.at(measurement_idx)
            .object.state.pose_covariance.pose.position,
            (*tracker_itr)->getPosition(measurements.header.stamp),
            (*tracker_itr)->getXYCovariance(measurements.header.stamp));

          if (2.448 <= mahalanobis_dist) {score = 0.0;}
        }
      }
      score_matrix(tracker_idx, measurement_idx) = score;
    }
  }

  return score_matrix;
}

double DataAssociation::getDistance(
  const geometry_msgs::msg::Point & measurement, const geometry_msgs::msg::Point & tracker)
{
  const double diff_x = tracker.x - measurement.x;
  const double diff_y = tracker.y - measurement.y;
  // const double diff_z = tracker.z - measurement.z;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

double DataAssociation::getMahalanobisDistance(
  const geometry_msgs::msg::Point & measurement, const geometry_msgs::msg::Point & tracker,
  const Eigen::Matrix2d & covariance)
{
  Eigen::Vector2d measurement_point;
  measurement_point << measurement.x, measurement.y;
  Eigen::Vector2d tracker_point;
  tracker_point << tracker.x, tracker.y;
  Eigen::MatrixXd mahalanobis_squared = (measurement_point - tracker_point).transpose() *
    covariance.inverse() * (measurement_point - tracker_point);
  return std::sqrt(mahalanobis_squared(0));
}
