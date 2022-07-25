#include "validation/refine_optimizer.hpp"

namespace validation
{
Eigen::Affine3f refinePose(
  const Eigen::Affine3f & extrinsic, const Eigen::Matrix3f & intrinsic, const cv::Mat & cost_image,
  const Eigen::Affine3f & pose, pcl::PointCloud<pcl::PointNormal> & linesegments)
{
  //
  return pose;
}

}  // namespace validation