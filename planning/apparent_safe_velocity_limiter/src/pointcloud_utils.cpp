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
  const PointCloud & pointcloud_msg, const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  downsampled_pointcloud->points.reserve(pointcloud->points.size());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pointcloud);
  filter.setLeafSize(0.5, 0.5, 100.0);
  filter.filter(*downsampled_pointcloud);
  pcl::PointIndices::Ptr idx_to_remove(new pcl::PointIndices());
  idx_to_remove->indices.reserve(downsampled_pointcloud->size());
  for (size_t i = 0; i < downsampled_pointcloud->size(); ++i) {
    const auto & point = downsampled_pointcloud->points[i];
    const auto p = point_t{point.x, point.y};
    if (boost::geometry::within(p, polygon_masks) || !boost::geometry::within(p, envelope))
      idx_to_remove->indices.push_back(i);
  }
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(downsampled_pointcloud);
  extract.setIndices(idx_to_remove);
  extract.setNegative(true);
  extract.filter(*pointcloud);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformAndFilterPointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud, const multipolygon_t & polygon_masks,
  const polygon_t & envelope, tier4_autoware_utils::TransformListener & transform_listener,
  const std::string & target_frame)
{
  tier4_autoware_utils::StopWatch stopwatch;
  const auto & header = pointcloud.header;
  const auto transform = transform_listener.getTransform(
    target_frame, header.frame_id, header.stamp, rclcpp::Duration::from_nanoseconds(0));
  auto obstacle_pointcloud = transformPointCloud(pointcloud, transform->transform);
  // TODO(Maxime CLEMENT): remove before merging
  std::cerr << "* pointcloud transform = " << stopwatch.toc() << " s\n";
  stopwatch.tic("filter");
  filterPointCloud(obstacle_pointcloud, polygon_masks, envelope);
  std::cerr << "* pointcloud filter traj = " << stopwatch.toc("filter") << " s\n";
  std::cerr << "** pointcloud total = " << stopwatch.toc() << " s\n";
  return obstacle_pointcloud;
}

multilinestring_t extractObstacleLines(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr, const double cluster_tolerance)
{
  multilinestring_t lines;
  if (pointcloud_ptr->empty()) return lines;

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

  lines.reserve(cluster_indices.size());
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
      lines.push_back(line);
    }
  }
  std::cerr << "* pointcloud extract = " << stopwatch.toc("extract") << " s\n";
  std::cerr << "** pointcloud obstacle total = " << stopwatch.toc() << " s\n";
  return lines;
}

}  // namespace apparent_safe_velocity_limiter
