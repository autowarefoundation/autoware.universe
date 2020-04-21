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

#include "cluster_data_association/data_association.hpp"
#include <sensor_msgs/point_cloud2_iterator.h>
#include "cluster_data_association/utils/utils.hpp"
#include "successive_shortest_path/successive_shortest_path.h"

DataAssociation::DataAssociation() : score_threshold_(0.1)
{
  can_assgin_matrix_ = Eigen::MatrixXi::Identity(20, 20);
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::UNKNOWN, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::TRUCK) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::BUS) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::CAR) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::BUS) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::CAR) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::TRUCK) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::MOTORBIKE) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::BICYCLE) = 1;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::PEDESTRIAN, autoware_perception_msgs::Semantic::UNKNOWN) =
    0;
  can_assgin_matrix_(
    autoware_perception_msgs::Semantic::ANIMAL, autoware_perception_msgs::Semantic::UNKNOWN) = 0;
  max_dist_matrix_ = Eigen::MatrixXd::Constant(20, 20, 1.0);
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::UNKNOWN) = 4.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::CAR) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::TRUCK) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::BUS) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::UNKNOWN) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::CAR) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::TRUCK) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::BUS) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::UNKNOWN) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::CAR) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::TRUCK) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::BUS) = 3.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::UNKNOWN) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::BICYCLE) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::MOTORBIKE) =
    2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::UNKNOWN) =
    2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::BICYCLE) =
    2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::MOTORBIKE) =
    2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::PEDESTRIAN, autoware_perception_msgs::Semantic::UNKNOWN) =
    2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::PEDESTRIAN,
    autoware_perception_msgs::Semantic::PEDESTRIAN) = 2.0;
  max_dist_matrix_(
    autoware_perception_msgs::Semantic::ANIMAL, autoware_perception_msgs::Semantic::UNKNOWN) = 2.0;
  // max_area_matrix_ = Eigen::MatrixXd::Constant(20, 20, /* large number */ 10000.0);
  // max_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::UNKNOWN) = 2.2 * 5.5;
  // max_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::CAR) = 2.2 * 5.5;
  // max_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::TRUCK) = 2.5 * 7.9;
  // max_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::BUS) = 2.7 * 12.0;
  // max_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::UNKNOWN) = 2.5
  // * 7.9; max_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::CAR) = 2.2
  // * 5.5; max_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::TRUCK) = 2.5
  // * 7.9; max_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::BUS) = 2.7
  // * 12.0; max_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::UNKNOWN)
  // = 2.7 * 12.0; max_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::CAR)
  // = 2.2 * 5.5; max_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::TRUCK)
  // = 2.5 * 7.9; max_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::BUS)
  // = 2.7 * 12.0; max_area_matrix_(autoware_perception_msgs::Semantic::BICYCLE,
  // autoware_perception_msgs::Semantic::UNKNOWN) = 2.5; max_area_matrix_(autoware_perception_msgs::Semantic::BICYCLE,
  // autoware_perception_msgs::Semantic::BICYCLE) = 2.5; max_area_matrix_(autoware_perception_msgs::Semantic::BICYCLE,
  // autoware_perception_msgs::Semantic::MOTORBIKE) = 3.0;
  // max_area_matrix_(autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::UNKNOWN) = 3.0;
  // max_area_matrix_(autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::BICYCLE) = 2.5;
  // max_area_matrix_(autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::MOTORBIKE)
  // = 3.0; max_area_matrix_(autoware_perception_msgs::Semantic::PEDESTRIAN,
  // autoware_perception_msgs::Semantic::UNKNOWN) = 2.0;
  // max_area_matrix_(autoware_perception_msgs::Semantic::PEDESTRIAN, autoware_perception_msgs::Semantic::PEDESTRIAN) =
  //     2.0;
  // max_area_matrix_(autoware_perception_msgs::Semantic::ANIMAL, autoware_perception_msgs::Semantic::UNKNOWN) = 2.0;
  // min_area_matrix_ = Eigen::MatrixXd::Constant(20, 20, /* small number */ 0.0);
  // min_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::UNKNOWN) = 1.2 * 3.0;
  // min_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::CAR) = 1.2 * 3.0;
  // min_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::TRUCK) = 1.5 * 4.0;
  // min_area_matrix_(autoware_perception_msgs::Semantic::CAR, autoware_perception_msgs::Semantic::BUS) = 2.0 * 5.0;
  // min_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::UNKNOWN) = 1.5
  // * 4.0; min_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::CAR) = 1.2
  // * 3.0; min_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::TRUCK) = 1.5
  // * 4.0; min_area_matrix_(autoware_perception_msgs::Semantic::TRUCK, autoware_perception_msgs::Semantic::BUS) = 2.0
  // * 5.0; min_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::UNKNOWN) = 2.0
  // * 5.0; min_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::CAR) = 1.2
  // * 3.0; min_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::TRUCK) = 1.5
  // * 4.0; min_area_matrix_(autoware_perception_msgs::Semantic::BUS, autoware_perception_msgs::Semantic::BUS) = 2.0
  // * 5.0; min_area_matrix_(autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::UNKNOWN) =
  // 0.001; min_area_matrix_(autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::BICYCLE) =
  // 0.001; min_area_matrix_(autoware_perception_msgs::Semantic::BICYCLE, autoware_perception_msgs::Semantic::MOTORBIKE)
  // = 0.001; min_area_matrix_(autoware_perception_msgs::Semantic::MOTORBIKE,
  // autoware_perception_msgs::Semantic::UNKNOWN) = 0.001;
  // min_area_matrix_(autoware_perception_msgs::Semantic::MOTORBIKE, autoware_perception_msgs::Semantic::BICYCLE) =
  // 0.001; min_area_matrix_(autoware_perception_msgs::Semantic::MOTORBIKE,
  // autoware_perception_msgs::Semantic::MOTORBIKE) =
  //     0.001;
  // min_area_matrix_(autoware_perception_msgs::Semantic::PEDESTRIAN, autoware_perception_msgs::Semantic::UNKNOWN) =
  // 0.001; min_area_matrix_(autoware_perception_msgs::Semantic::PEDESTRIAN,
  // autoware_perception_msgs::Semantic::PEDESTRIAN) =
  //     0.001;
  // min_area_matrix_(autoware_perception_msgs::Semantic::ANIMAL, autoware_perception_msgs::Semantic::UNKNOWN) = 0.5;
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
}

Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const autoware_perception_msgs::DynamicObjectWithFeatureArray & cluster0,
  const autoware_perception_msgs::DynamicObjectWithFeatureArray & cluster1)
{
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(cluster1.feature_objects.size(), cluster0.feature_objects.size());
  for (size_t cluster1_idx = 0; cluster1_idx < cluster1.feature_objects.size(); ++cluster1_idx) {
    for (size_t cluster0_idx = 0; cluster0_idx < cluster0.feature_objects.size(); ++cluster0_idx) {
      double score = 0.0;
      if (can_assgin_matrix_(
            cluster1.feature_objects.at(cluster1_idx).object.semantic.type,
            cluster0.feature_objects.at(cluster0_idx).object.semantic.type)) {
        double max_dist = max_dist_matrix_(
          cluster1.feature_objects.at(cluster1_idx).object.semantic.type,
          cluster0.feature_objects.at(cluster0_idx).object.semantic.type);
        // double max_area = max_area_matrix_(cluster1.feature_objects.at(cluster1_idx).object.semantic.type,
        //                                    cluster0.feature_objects.at(cluster0_idx).object.semantic.type);
        // double min_area = min_area_matrix_(cluster1.feature_objects.at(cluster1_idx).object.semantic.type,
        //                                    cluster0.feature_objects.at(cluster0_idx).object.semantic.type);
        double dist = getDistance(
          getCentroid(cluster0.feature_objects.at(cluster0_idx).feature.cluster),
          getCentroid(cluster1.feature_objects.at(cluster1_idx).feature.cluster));
        // double area0 = utils::getArea(cluster0.feature_objects.at(cluster0_idx).object.shape);
        // double area1 = utils::getArea(cluster1.feature_objects.at(cluster1_idx).object.shape);
        score = (max_dist - std::min(dist, max_dist)) / max_dist;
        if (max_dist < dist) score = 0.0;
        // if (area < min_area || max_area < area) score = 0.0;
      }
      score_matrix(cluster1_idx, cluster0_idx) = score;
    }
  }
  return score_matrix;
}

double DataAssociation::getDistance(
  const geometry_msgs::Point & point0, const geometry_msgs::Point & point1)
{
  const double diff_x = point1.x - point0.x;
  const double diff_y = point1.y - point0.y;
  // const double diff_z = point1.z - point0.z;
  return std::sqrt(diff_x * diff_x + diff_y * diff_y);
}

geometry_msgs::Point DataAssociation::getCentroid(const sensor_msgs::PointCloud2 & pointcloud)
{
  geometry_msgs::Point centroid;
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
  centroid.x = centroid.x / ((double)pointcloud.height * (double)pointcloud.width);
  centroid.y = centroid.y / ((double)pointcloud.height * (double)pointcloud.width);
  centroid.z = centroid.z / ((double)pointcloud.height * (double)pointcloud.width);
  return centroid;
}