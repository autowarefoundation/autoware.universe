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

#include "pointcloud_preprocessor/utility/utilities.hpp"

namespace pointcloud_preprocessor::utils
{
std::vector<PointCgal> to_cgal_polygon(
  const geometry_msgs::msg::Polygon::ConstSharedPtr & polygon_in)
{
  std::vector<PointCgal> polyline_polygon;
  if (polygon_in->points.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in->points;
  polyline_polygon.resize(vertices_in.size());
  std::transform(
    polygon_in->points.begin(), polygon_in->points.end(), polyline_polygon.begin(),
    [](const geometry_msgs::msg::Point32 & p_in) { return PointCgal(p_in.x, p_in.y); });
  return polyline_polygon;
}

std::vector<PointCgal> to_cgal_polygon(const lanelet::BasicPolygon2d & polygon_in)
{
  std::vector<PointCgal> polyline_polygon;
  if (polygon_in.size() < 3) {
    throw std::length_error("Polygon vertex count should be larger than 2.");
  }

  const auto & vertices_in = polygon_in;
  polyline_polygon.resize(vertices_in.size());
  std::transform(
    polygon_in.cbegin(), polygon_in.cend(), polyline_polygon.begin(),
    [](const Eigen::Matrix<double, 2, 1> & p_in) { return PointCgal(p_in.x(), p_in.y()); });
  return polyline_polygon;
}

sensor_msgs::msg::PointCloud2 remove_polygon_cgal_from_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_in_ptr,
  const std::vector<PointCgal> & polyline_polygon)
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;

  sensor_msgs::msg::PointCloud2 transformed_cluster = *cloud_in_ptr;

  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cluster, "x"),
       iter_y(transformed_cluster, "y"), iter_z(transformed_cluster, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    // check if the point is inside the polygon
    if (
      CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(), PointCgal(*iter_x, *iter_y), K()) ==
      CGAL::ON_UNBOUNDED_SIDE) {
      pcl::PointXYZ p;
      p.x = *iter_x;
      p.y = *iter_y;
      p.z = *iter_z;
      pcl_output.emplace_back(p);
    }
  }
  pcl::toROSMsg(pcl_output, output);
  output.header = cloud_in_ptr->header;
  return output;
}

pcl::PointCloud<pcl::PointXYZ> remove_polygon_cgal_from_cloud(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_in, const std::vector<PointCgal> & polyline_polygon)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_output;

  for (const auto & p : cloud_in) {
    // check if the point is inside the polygon
    if (
      CGAL::bounded_side_2(
        polyline_polygon.begin(), polyline_polygon.end(), PointCgal(p.x, p.y), K()) ==
      CGAL::ON_UNBOUNDED_SIDE) {
      pcl_output.emplace_back(p);
    }
  }

  return pcl_output;
}  // namespace

}  // namespace pointcloud_preprocessor::utils
