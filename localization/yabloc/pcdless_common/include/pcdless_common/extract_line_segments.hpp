#pragma once
#include <sophus/geometry.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::common
{
pcl::PointCloud<pcl::PointNormal> extract_near_line_segments(
  const pcl::PointCloud<pcl::PointNormal> & line_segments, const Sophus::SE3f & transform,
  const float max_range = 40);

}  // namespace pcdless::common