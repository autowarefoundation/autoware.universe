// Copyright 2022 Tier IV, Inc.
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

#include "apparent_safe_velocity_limiter/pointcloud_utils.hpp"

#include <pcl_ros/transforms.hpp>
#include <tier4_autoware_utils/system/stop_watch.hpp>

#include <boost/geometry/algorithms/within.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/Vertices.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/concave_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <vector>

namespace apparent_safe_velocity_limiter
{

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
  const PointCloud & pointcloud_msg, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame)
{
  const auto & header = pointcloud_msg.header;
  const auto transform = transform_listener.getTransform(
    target_frame, header.frame_id, header.stamp, rclcpp::Duration::from_nanoseconds(0));
  const Eigen::Matrix4f transform_matrix =
    tf2::transformToEigen(transform->transform).matrix().cast<float>();

  PointCloud transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);
  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);
  return pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>(std::move(transformed_pointcloud)));
}

void filterPointCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const multipolygon_t & polygon_masks,
  const polygon_t & envelope)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr polygon_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & p : envelope.outer())
    polygon_cloud_ptr->push_back(pcl::PointXYZ(p.x(), p.y(), 0));
  std::vector<pcl::Vertices> polygons_idx(1);
  polygons_idx.front().vertices.resize(envelope.outer().size());
  std::iota(polygons_idx.front().vertices.begin(), polygons_idx.front().vertices.end(), 0);
  pcl::CropHull<pcl::PointXYZ> crop_outside_of_envelope;
  crop_outside_of_envelope.setInputCloud(pointcloud);
  crop_outside_of_envelope.setDim(2);
  crop_outside_of_envelope.setHullCloud(polygon_cloud_ptr);
  crop_outside_of_envelope.setHullIndices(polygons_idx);
  crop_outside_of_envelope.setCropOutside(true);
  crop_outside_of_envelope.filter(*pointcloud);

  std::cerr << "* envelope filter size = " << pointcloud->size() << "\n";

  pcl::PointCloud<pcl::PointXYZ>::Ptr mask_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> masks_idx(polygon_masks.size());
  size_t start_idx = 0;
  for (size_t i = 0; i < polygon_masks.size(); ++i) {
    const auto & polygon_mask = polygon_masks[i];
    const auto mask_size = polygon_mask.outer().size();
    std::cerr << "\tMask of size " << mask_size << "\n";
    for (const auto & p : polygon_mask.outer())
      mask_cloud_ptr->push_back(pcl::PointXYZ(p.x(), p.y(), 0));
    auto & mask_idx = masks_idx[i];
    mask_idx.vertices.resize(mask_size);
    std::iota(mask_idx.vertices.begin(), mask_idx.vertices.end(), start_idx);
    start_idx += mask_size;
  }
  pcl::CropHull<pcl::PointXYZ> crop_masks;
  crop_masks.setInputCloud(pointcloud);
  crop_masks.setDim(2);
  crop_masks.setHullCloud(mask_cloud_ptr);
  crop_masks.setHullIndices(masks_idx);
  crop_masks.setCropOutside(false);
  crop_masks.filter(*pointcloud);
  std::cerr << "* masks filter size = " << pointcloud->size() << "\n";
}

std::vector<Obstacle> extractObstacles(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr, const double cluster_tolerance)
{
  std::vector<Obstacle> obstacles;
  if (pointcloud_ptr->empty()) return obstacles;

  tier4_autoware_utils::StopWatch stopwatch;
  const pcl::PointCloud<pcl::PointXYZ>::Ptr projected_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  // Projection to the XY plane
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  coefficients->values.resize(4);
  coefficients->values[0] = coefficients->values[1] = 0;
  coefficients->values[2] = 1.0;
  coefficients->values[3] = 0;
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(pointcloud_ptr);
  proj.setModelCoefficients(coefficients);
  proj.filter(*projected_pointcloud_ptr);
  std::cerr << "* pointcloud project = " << stopwatch.toc() << " s\n";

  stopwatch.tic("extract");
  const pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(projected_pointcloud_ptr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(1);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(projected_pointcloud_ptr);
  ec.extract(cluster_indices);

  obstacles.reserve(cluster_indices.size());
  for (const auto & cluster : cluster_indices) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto idx : cluster.indices)
      cloud_cluster->push_back((*projected_pointcloud_ptr)[idx]);
    cloud_cluster->width = cloud_cluster->size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    pcl::PointCloud<pcl::PointXYZ> cloud_hull;
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud(cloud_cluster);
    chull.setAlpha(1.0);
    chull.reconstruct(cloud_hull);
    linestring_t line;
    line.reserve(cloud_hull.size());
    for (const auto & point : cloud_hull) line.push_back(point_t{point.x, point.y});
    if (!line.empty()) {
      line.push_back(line.front());
      obstacles.emplace_back(line);
    }
  }
  std::cerr << "* pointcloud extract = " << stopwatch.toc("extract") << " s\n";
  std::cerr << "** pointcloud obstacle total = " << stopwatch.toc() << " s\n";
  return obstacles;
}

}  // namespace apparent_safe_velocity_limiter
