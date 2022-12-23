#pragma once
#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::ground_server
{
pcl::PointCloud<pcl::PointXYZ> sample_from_polygons(const lanelet::PolygonLayer & polygons);

pcl::PointCloud<pcl::PointXYZ> fill_points_in_polygon(
  const pcl::PointCloud<pcl::PointXYZ> & src_cloud);
}  // namespace pcdless::ground_server
