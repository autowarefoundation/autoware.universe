#include "pcdless_common/extract_line_segments.hpp"

namespace pcdless::common
{
pcl::PointCloud<pcl::PointNormal> extract_near_line_segments(
  const pcl::PointCloud<pcl::PointNormal> & line_segments, const Sophus::SE3f & transform,
  const float max_range)
{
  Eigen::Vector3f pose_vector = transform.translation();

  // Compute distance between pose and linesegment of linestring
  auto check_intersection = [max_range, pose_vector](const pcl::PointNormal & pn) -> bool {
    const Eigen::Vector3f from = pn.getVector3fMap() - pose_vector;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose_vector;

    Eigen::Vector3f tangent = to - from;
    if (tangent.squaredNorm() < 1e-3f) {
      return from.norm() < 1.414 * max_range;
    }

    float inner = from.dot(tangent);
    float mu = std::clamp(inner / tangent.squaredNorm(), -1.0f, 0.0f);
    Eigen::Vector3f nearest = from - tangent * mu;
    return nearest.norm() < 1.414 * max_range;
  };

  pcl::PointCloud<pcl::PointNormal> dst;
  for (const pcl::PointNormal & pn : line_segments) {
    if (check_intersection(pn)) {
      dst.push_back(pn);
    }
  }
  return dst;
}

}  // namespace pcdless::common