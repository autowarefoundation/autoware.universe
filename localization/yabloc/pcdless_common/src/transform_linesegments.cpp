#include "pcdless_common/transform_linesegments.hpp"

namespace pcdless::common
{
pcl::PointCloud<pcl::PointXYZLNormal> transform_linesegments(
  const pcl::PointCloud<pcl::PointXYZLNormal> & src, const Sophus::SE3f & transform)
{
  pcl::PointCloud<pcl::PointXYZLNormal> dst;
  for (const pcl::PointXYZLNormal & line : src) {
    Eigen::Vector3f p1 = line.getVector3fMap();
    Eigen::Vector3f p2 = line.getNormalVector3fMap();

    pcl::PointXYZLNormal transformed;
    transformed.getVector3fMap() = transform * p1;
    transformed.getNormalVector3fMap() = transform * p2;
    transformed.label = line.label;
    dst.push_back(transformed);
  }
  return dst;
}

pcl::PointCloud<pcl::PointNormal> transform_linesegments(
  const pcl::PointCloud<pcl::PointNormal> & src, const Sophus::SE3f & transform)
{
  pcl::PointCloud<pcl::PointNormal> dst;
  for (const pcl::PointNormal & line : src) {
    Eigen::Vector3f p1 = line.getVector3fMap();
    Eigen::Vector3f p2 = line.getNormalVector3fMap();

    pcl::PointNormal transformed;
    transformed.getVector3fMap() = transform * p1;
    transformed.getNormalVector3fMap() = transform * p2;
    dst.push_back(transformed);
  }
  return dst;
}
}  // namespace pcdless::common