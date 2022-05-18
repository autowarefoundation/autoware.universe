// Copyright 2017-2022 Arm Ltd., TierIV, Autoware Foundation, The Apollo Authors
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

#include <common/types.hpp>
#include <geometry/bounding_box_2d.hpp>
#include <lidar_apollo_segmentation_tvm/cluster2d.hpp>

#include <geometry_msgs/msg/point32.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>

using Point = geometry_msgs::msg::Point32;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace perception
{
namespace lidar_apollo_segmentation_tvm
{
Cluster2D::Cluster2D(const int32_t rows, const int32_t cols, const float32_t range)
: rows_(rows),
  cols_(cols),
  siz_(rows * cols),
  range_(range),
  scale_(0.5f * static_cast<float32_t>(rows) / range),
  inv_res_x_(0.5f * static_cast<float32_t>(cols) / range),
  inv_res_y_(0.5f * static_cast<float32_t>(rows) / range)
{
  point2grid_.clear();
  id_img_.assign(siz_, -1);
  pc_ptr_.reset();
  valid_indices_in_pc_ = nullptr;
}

void Cluster2D::traverse(Node * x) const
{
  std::vector<Node *> p;
  p.clear();

  while (x->traversed == 0) {
    p.push_back(x);
    x->traversed = 2;
    x = x->center_node;
  }
  if (x->traversed == 2) {
    for (int i = static_cast<int>(p.size()) - 1; i >= 0 && p[i] != x; i--) {
      p[i]->is_center = true;
    }
    x->is_center = true;
  }
  for (size_t i = 0; i < p.size(); i++) {
    Node * y = p[i];
    y->traversed = 1;
    y->parent = x->parent;
  }
}

void Cluster2D::cluster(
  const float32_t * inferred_data, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & pc_ptr,
  const pcl::PointIndices & valid_indices, float32_t objectness_thresh,
  bool use_all_grids_for_clustering)
{
  const float32_t * category_pt_data = inferred_data;
  const float32_t * instance_pt_x_data = inferred_data + siz_;
  const float32_t * instance_pt_y_data = inferred_data + siz_ * 2;

  pc_ptr_ = pc_ptr;

  std::vector<std::vector<Node>> nodes(rows_, std::vector<Node>(cols_, Node()));

  valid_indices_in_pc_ = &(valid_indices.indices);
  point2grid_.assign(valid_indices_in_pc_->size(), -1);

  for (size_t i = 0; i < valid_indices_in_pc_->size(); ++i) {
    int32_t point_id = valid_indices_in_pc_->at(i);
    const auto & point = pc_ptr_->points[point_id];
    // * the coordinates of x and y have been exchanged in feature generation
    // step,
    // so we swap them back here.
    int32_t pos_x = F2I(point.y, range_, inv_res_x_);  // col
    int32_t pos_y = F2I(point.x, range_, inv_res_y_);  // row
    if (IsValidRowCol(pos_y, pos_x)) {
      point2grid_[i] = RowCol2Grid(pos_y, pos_x);
      nodes[pos_y][pos_x].point_num++;
    }
  }

  for (int32_t row = 0; row < rows_; ++row) {
    for (int32_t col = 0; col < cols_; ++col) {
      int32_t grid = RowCol2Grid(row, col);
      Node * node = &nodes[row][col];
      DisjointSetMakeSet(node);
      node->is_object = (use_all_grids_for_clustering || nodes[row][col].point_num > 0) &&
                        (*(category_pt_data + grid) >= objectness_thresh);
      int32_t center_row =
        row + static_cast<int32_t>(std::round(instance_pt_x_data[grid] * scale_));
      int32_t center_col =
        col + static_cast<int32_t>(std::round(instance_pt_y_data[grid] * scale_));
      center_row = std::min(std::max(center_row, 0), rows_ - 1);
      center_col = std::min(std::max(center_col, 0), cols_ - 1);
      node->center_node = &nodes[center_row][center_col];
    }
  }

  for (int32_t row = 0; row < rows_; ++row) {
    for (int32_t col = 0; col < cols_; ++col) {
      Node * node = &nodes[row][col];
      if (node->is_object && node->traversed == 0) {
        traverse(node);
      }
    }
  }

  for (int32_t row = 0; row < rows_; ++row) {
    for (int32_t col = 0; col < cols_; ++col) {
      Node * node = &nodes[row][col];
      if (!node->is_center) {
        continue;
      }
      for (int32_t row2 = row - 1; row2 <= row + 1; ++row2) {
        for (int32_t col2 = col - 1; col2 <= col + 1; ++col2) {
          if ((row2 == row || col2 == col) && IsValidRowCol(row2, col2)) {
            Node * node2 = &nodes[row2][col2];
            if (node2->is_center) {
              DisjointSetUnion(node, node2);
            }
          }
        }
      }
    }
  }

  int32_t count_obstacles = 0;
  obstacles_.clear();
  id_img_.assign(siz_, -1);
  for (int32_t row = 0; row < rows_; ++row) {
    for (int32_t col = 0; col < cols_; ++col) {
      Node * node = &nodes[row][col];
      if (!node->is_object) {
        continue;
      }
      Node * root = DisjointSetFind(node);
      if (root->obstacle_id < 0) {
        root->obstacle_id = count_obstacles++;
        obstacles_.push_back(Obstacle());
      }
      int32_t grid = RowCol2Grid(row, col);
      id_img_[grid] = root->obstacle_id;
      obstacles_[root->obstacle_id].grids.push_back(grid);
    }
  }
  filter(inferred_data);
  classify(inferred_data);
}

void Cluster2D::filter(const float32_t * inferred_data)
{
  const float32_t * confidence_pt_data = inferred_data + siz_ * 3;
  const float32_t * height_pt_data = inferred_data + siz_ * 11;

  for (size_t obstacle_id = 0; obstacle_id < obstacles_.size(); obstacle_id++) {
    Obstacle * obs = &obstacles_[obstacle_id];
    float64_t score = 0.0;
    float64_t height = 0.0;
    for (int32_t grid : obs->grids) {
      score += static_cast<float64_t>(confidence_pt_data[grid]);
      height += static_cast<float64_t>(height_pt_data[grid]);
    }
    obs->score = static_cast<float32_t>(score / static_cast<float64_t>(obs->grids.size()));
    obs->height = static_cast<float32_t>(height / static_cast<float64_t>(obs->grids.size()));
    obs->cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
}

void Cluster2D::classify(const float32_t * inferred_data)
{
  const float32_t * classify_pt_data = inferred_data + siz_ * 4;
  int num_classes = static_cast<int>(MetaType::MAX_META_TYPE);
  for (size_t obs_id = 0; obs_id < obstacles_.size(); obs_id++) {
    Obstacle * obs = &obstacles_[obs_id];

    for (size_t grid_id = 0; grid_id < obs->grids.size(); grid_id++) {
      int32_t grid = obs->grids[grid_id];
      for (int k = 0; k < num_classes; k++) {
        obs->meta_type_probs[k] += classify_pt_data[k * siz_ + grid];
      }
    }
    int meta_type_id = 0;
    for (int k = 0; k < num_classes; k++) {
      obs->meta_type_probs[k] /= static_cast<float32_t>(obs->grids.size());
      if (obs->meta_type_probs[k] > obs->meta_type_probs[meta_type_id]) {
        meta_type_id = k;
      }
    }
    obs->meta_type = static_cast<MetaType>(meta_type_id);
  }
}

autoware_auto_perception_msgs::msg::BoundingBox Cluster2D::obstacleToObject(
  const Obstacle & in_obstacle) const
{
  auto in_points = in_obstacle.cloud_ptr->points;

  autoware_auto_perception_msgs::msg::BoundingBox resulting_object =
    common::geometry::bounding_box::lfit_bounding_box_2d(in_points.begin(), in_points.end());
  common::geometry::bounding_box::compute_height(
    in_points.begin(), in_points.end(), resulting_object);

  resulting_object.class_likelihood = in_obstacle.score;
  if (in_obstacle.meta_type == MetaType::META_PEDESTRIAN) {
    resulting_object.vehicle_label = autoware_auto_perception_msgs::msg::BoundingBox::PEDESTRIAN;
  } else if (in_obstacle.meta_type == MetaType::META_NONMOT) {
    resulting_object.vehicle_label = autoware_auto_perception_msgs::msg::BoundingBox::MOTORCYCLE;
  } else if (in_obstacle.meta_type == MetaType::META_SMALLMOT) {
    resulting_object.vehicle_label = autoware_auto_perception_msgs::msg::BoundingBox::CAR;
  } else {
    resulting_object.vehicle_label = autoware_auto_perception_msgs::msg::BoundingBox::NO_LABEL;
  }

  return resulting_object;
}

std::shared_ptr<autoware_auto_perception_msgs::msg::BoundingBoxArray> Cluster2D::getObjects(
  const float32_t confidence_thresh, const float32_t height_thresh, const int32_t min_pts_num)
{
  auto object_array = std::make_shared<autoware_auto_perception_msgs::msg::BoundingBoxArray>();

  for (size_t i = 0; i < point2grid_.size(); ++i) {
    int32_t grid = point2grid_[i];
    if (grid < 0) {
      continue;
    }

    int32_t obstacle_id = id_img_[grid];

    int32_t point_id = valid_indices_in_pc_->at(i);

    if (obstacle_id >= 0 && obstacles_[obstacle_id].score >= confidence_thresh) {
      if (
        height_thresh < 0 ||
        pc_ptr_->points[point_id].z <= obstacles_[obstacle_id].height + height_thresh) {
        obstacles_[obstacle_id].cloud_ptr->push_back(pc_ptr_->points[point_id]);
      }
    }
  }

  for (size_t obstacle_id = 0; obstacle_id < obstacles_.size(); obstacle_id++) {
    Obstacle * obs = &obstacles_[obstacle_id];
    if (static_cast<int>(obs->cloud_ptr->size()) < min_pts_num) {
      continue;
    }
    autoware_auto_perception_msgs::msg::BoundingBox out_obj = obstacleToObject(*obs);
    object_array->boxes.push_back(out_obj);
  }

  return object_array;
}
}  // namespace lidar_apollo_segmentation_tvm
}  // namespace perception
}  // namespace autoware
