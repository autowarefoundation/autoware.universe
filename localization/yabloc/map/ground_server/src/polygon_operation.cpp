#include "ground_server/polygon_operation.hpp"

namespace map
{
void pushBackLine(
  pcl::PointCloud<pcl::PointXYZ> & dst_cloud, const pcl::PointXYZ & from, const pcl::PointXYZ & to)
{
  Eigen::Vector3f f = from.getVector3fMap();
  Eigen::Vector3f t = to.getVector3fMap();
  const float L = (f - t).norm();

  for (float l = 0.f; l < L; l += 0.25f) {
    Eigen::Vector3f xyz = f + (t - f) * l;
    dst_cloud.emplace_back(xyz.x(), xyz.y(), xyz.z());
  }
}

void pushBackContour(
  pcl::PointCloud<pcl::PointXYZ> & dst_cloud, const pcl::PointCloud<pcl::PointXYZ> & vertices)
{
  const int N = vertices.size();
  for (int i = 0; i < N - 1; ++i) {
    pushBackLine(dst_cloud, vertices.at(i), vertices.at(i + 1));
  }
  pushBackLine(dst_cloud, vertices.at(0), vertices.at(N - 1));
}

pcl::PointCloud<pcl::PointXYZ> shrinkVertices(
  const pcl::PointCloud<pcl::PointXYZ> & vertices, float rate)
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  for (const pcl::PointXYZ p : vertices) center += p.getVector3fMap();
  center /= vertices.size();

  pcl::PointCloud<pcl::PointXYZ> dst_cloud;
  for (const pcl::PointXYZ p : vertices) {
    Eigen::Vector3f xyz = (p.getVector3fMap() - center) * rate + center;
    dst_cloud.emplace_back(xyz.x(), xyz.y(), xyz.z());
  }
  return dst_cloud;
}

pcl::PointCloud<pcl::PointXYZ> fillPointsInPolygon(const pcl::PointCloud<pcl::PointXYZ> & src_cloud)
{
  pcl::PointCloud<pcl::PointXYZ> dst_cloud;

  std::vector<float> shrink_rates = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1};
  for (float rate : shrink_rates) {
    pushBackContour(dst_cloud, shrinkVertices(src_cloud, rate));
  }

  return dst_cloud;
}
}  // namespace map
