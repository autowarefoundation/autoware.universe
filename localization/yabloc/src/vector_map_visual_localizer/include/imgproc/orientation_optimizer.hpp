#pragma once
#include <sophus/geometry.hpp>

namespace imgproc::opt
{
void sample();
Sophus::SO3f optimizeOnce(const Sophus::SO3f & R, const Eigen::Vector3f & vp);
}  // namespace imgproc::opt