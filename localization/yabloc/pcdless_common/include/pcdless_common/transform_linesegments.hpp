#pragma once
#include <sophus/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::common
{
pcl::PointCloud<pcl::PointNormal> transform_linesegments(
  const pcl::PointCloud<pcl::PointNormal> & src, const Sophus::SE3f & transform);
}  // namespace pcdless::common