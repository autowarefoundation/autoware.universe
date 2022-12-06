#pragma once
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>

namespace pcdless::common
{
template <typename T>
pcl::PointCloud<T> extract(const pcl::PointCloud<T> & cloud, pcl::PointIndices::Ptr indices_ptr)
{
  pcl::ExtractIndices<T> extract;
  extract.setIndices(indices_ptr);
  extract.setInputCloud(cloud);

  pcl::PointCloud<T> extracted_cloud;
  extract.filter(extracted_cloud);
  return extracted_cloud;
}
}  // namespace pcdless::common