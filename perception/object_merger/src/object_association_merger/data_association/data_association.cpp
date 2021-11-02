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

#include <object_association_merger/data_association.hpp>
#include <object_association_merger/successive_shortest_path.hpp>
#include <object_association_merger/utils/utils.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <unordered_map>
#include <vector>

DataAssociation::DataAssociation() : score_threshold_(0.1)
{
  can_assign_matrix_ = Eigen::MatrixXi::Identity(20, 20);
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::UNKNOWN,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::TRUCK) =
    1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::BUS) = 1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::CAR) =
    1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::BUS) =
    1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::CAR) = 1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::TRUCK) =
    1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 1;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  can_assign_matrix_(
    autoware_perception_msgs::msg::Semantic::ANIMAL,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0;
  max_dist_matrix_ = Eigen::MatrixXd::Constant(20, 20, 1.0);
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 4.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::CAR) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::TRUCK) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::BUS) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::CAR) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::TRUCK) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::BUS) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::CAR) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::TRUCK) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::BUS) =
    3.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::msg::Semantic::ANIMAL,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0;
  max_area_matrix_ = Eigen::MatrixXd::Constant(20, 20, /* large number */ 10000.0);
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::UNKNOWN,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 5.0 * 5.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.2 * 5.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::CAR) =
    2.2 * 5.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::TRUCK) =
    2.5 * 7.9;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::BUS) =
    2.7 * 12.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.5 * 7.9;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::CAR) =
    2.2 * 5.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::TRUCK) = 2.5 * 7.9;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::BUS) =
    2.7 * 12.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.7 * 12.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::CAR) =
    2.2 * 5.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::TRUCK) =
    2.5 * 7.9;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::BUS) =
    2.7 * 12.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 2.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 3.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 3.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 2.5;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 3.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN) = 2.0;
  max_area_matrix_(
    autoware_perception_msgs::msg::Semantic::ANIMAL,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0;
  min_area_matrix_ = Eigen::MatrixXd::Constant(20, 20, /* small number */ 0.0);
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 1.2 * 3.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::CAR) =
    1.2 * 3.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::TRUCK) =
    1.5 * 4.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::CAR, autoware_perception_msgs::msg::Semantic::BUS) =
    2.0 * 5.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 1.5 * 4.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::CAR) =
    1.2 * 3.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK,
    autoware_perception_msgs::msg::Semantic::TRUCK) = 1.5 * 4.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::TRUCK, autoware_perception_msgs::msg::Semantic::BUS) =
    2.0 * 5.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 2.0 * 5.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::CAR) =
    1.2 * 3.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::TRUCK) =
    1.5 * 4.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BUS, autoware_perception_msgs::msg::Semantic::BUS) =
    2.0 * 5.0;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::BICYCLE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::BICYCLE) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::MOTORBIKE,
    autoware_perception_msgs::msg::Semantic::MOTORBIKE) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN,
    autoware_perception_msgs::msg::Semantic::PEDESTRIAN) = 0.001;
  min_area_matrix_(
    autoware_perception_msgs::msg::Semantic::ANIMAL,
    autoware_perception_msgs::msg::Semantic::UNKNOWN) = 0.5;
}

bool DataAssociation::assign(
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

  for (auto itr = direct_assignment.begin(); itr != direct_assignment.end();) {
    if (src(itr->first, itr->second) < score_threshold_) {
      itr = direct_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
  for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end();) {
    if (src(itr->second, itr->first) < score_threshold_) {
      itr = reverse_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
  return true;
}

Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & object0,
  const autoware_perception_msgs::msg::DynamicObjectWithFeatureArray & object1)
{
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(object1.feature_objects.size(), object0.feature_objects.size());
  for (size_t object1_idx = 0; object1_idx < object1.feature_objects.size(); ++object1_idx) {
    for (size_t object0_idx = 0; object0_idx < object0.feature_objects.size(); ++object0_idx) {
      double score = 0.0;
      if (can_assign_matrix_(
            object1.feature_objects.at(object1_idx).object.semantic.type,
            object0.feature_objects.at(object0_idx).object.semantic.type)) {
        const double max_dist = max_dist_matrix_(
          object1.feature_objects.at(object1_idx).object.semantic.type,
          object0.feature_objects.at(object0_idx).object.semantic.type);
        const double max_area = max_area_matrix_(
          object1.feature_objects.at(object1_idx).object.semantic.type,
          object0.feature_objects.at(object0_idx).object.semantic.type);
        const double min_area = min_area_matrix_(
          object1.feature_objects.at(object1_idx).object.semantic.type,
          object0.feature_objects.at(object0_idx).object.semantic.type);
        const double dist = getDistance(
          object0.feature_objects.at(object0_idx).object.state.pose_covariance.pose.position,
          object1.feature_objects.at(object1_idx).object.state.pose_covariance.pose.position);
        const double area0 = utils::getArea(object0.feature_objects.at(object0_idx).object.shape);
        const double area1 = utils::getArea(object1.feature_objects.at(object1_idx).object.shape);
        score = (max_dist - std::min(dist, max_dist)) / max_dist;
        if (max_dist < dist) {
          score = 0.0;
        }
        if (area0 < min_area || max_area < area0) {
          score = 0.0;
        }
        if (area1 < min_area || max_area < area1) {
          score = 0.0;
        }
      }
      score_matrix(object1_idx, object0_idx) = score;
    }
  }
  return score_matrix;
}

double DataAssociation::getDistance(
  const geometry_msgs::msg::Point & point0, const geometry_msgs::msg::Point & point1)
{
  const double diff_x = point1.x - point0.x;
  const double diff_y = point1.y - point0.y;
  // const double diff_z = point1.z - point0.z;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

geometry_msgs::msg::Point DataAssociation::getCentroid(
  const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"),
       iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    centroid.x += *iter_x;
    centroid.y += *iter_y;
    centroid.z += *iter_z;
  }
  centroid.x =
    centroid.x / (static_cast<double>(pointcloud.height) * static_cast<double>(pointcloud.width));
  centroid.y =
    centroid.y / (static_cast<double>(pointcloud.height) * static_cast<double>(pointcloud.width));
  centroid.z =
    centroid.z / (static_cast<double>(pointcloud.height) * static_cast<double>(pointcloud.width));
  return centroid;
}
