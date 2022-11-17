#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map
{
pcl::PointCloud<pcl::PointXYZ> fillPointsInPolygon(
  const pcl::PointCloud<pcl::PointXYZ> & src_cloud);
}
