#pragma once
#include <Eigen/Core>

namespace pcdless::bayes_util
{
Eigen::Matrix2d approximate_by_spd(const Eigen::Matrix2d & target, bool verbose = false);

// Estimate likelihood from prior_covariance and post_covariance.
// This estimation assumes all PDFs are normal distribution.
Eigen::Matrix2f debayes_covariance(
  const Eigen::Matrix2f & post_covariance, const Eigen::Matrix2f & prior_covariance);
}  // namespace pcdless::bayes_util