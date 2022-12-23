#pragma once
#include <Eigen/Core>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <unordered_set>
#include <vector>

namespace pcdless::ground_server
{
void upsample_line_string(
  const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::Vector3f f(from.x(), from.y(), from.z());
  Eigen::Vector3f t(to.x(), to.y(), to.z());
  float length = (t - f).norm();
  Eigen::Vector3f d = (t - f).normalized();
  for (float l = 0; l < length; l += 0.5f) {
    pcl::PointXYZ xyz;
    xyz.getVector3fMap() = (f + l * d);
    cloud->push_back(xyz);
  }
};

std::vector<int> merge_indices(const std::vector<int> & indices1, const std::vector<int> & indices2)
{
  std::unordered_set<int> set;
  for (int i : indices1) set.insert(i);
  for (int i : indices2) set.insert(i);

  std::vector<int> indices;
  indices.assign(set.begin(), set.end());
  return indices;
}

}  // namespace pcdless::ground_server