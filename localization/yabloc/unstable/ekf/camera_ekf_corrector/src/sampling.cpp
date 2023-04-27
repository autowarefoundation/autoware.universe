// Copyright 2023 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "camera_ekf_corrector/sampling.hpp"

#include <bayes_util/bayes_util.hpp>
#include <yabloc_common/pose_conversions.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

namespace yabloc::ekf_corrector
{
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());

geometry_msgs::msg::PoseWithCovariance debug_debayes_distribution(
  const geometry_msgs::msg::PoseWithCovariance & post,
  const geometry_msgs::msg::PoseWithCovariance & prior)
{
  // Infer measurement distribution
  Eigen::Matrix2f post_cov, prior_cov;
  {
    auto c1 = prior.covariance;
    auto c2 = post.covariance;
    prior_cov << c1[6 * 0 + 0], c1[6 * 0 + 1], c1[6 * 1 + 0], c1[6 * 1 + 1];
    post_cov << c2[6 * 0 + 0], c2[6 * 0 + 1], c2[6 * 1 + 0], c2[6 * 1 + 1];
  }

  geometry_msgs::msg::PoseWithCovariance likelihood = post;
  Eigen::Matrix2f like_cov = bayes_util::debayes_covariance(post_cov, prior_cov);
  likelihood.covariance[0 * 6 + 0] = like_cov(0, 0);
  likelihood.covariance[0 * 6 + 1] = like_cov(0, 1);
  likelihood.covariance[1 * 6 + 0] = like_cov(1, 0);
  likelihood.covariance[1 * 6 + 1] = like_cov(1, 1);
  return likelihood;
}

geometry_msgs::msg::PoseWithCovariance debayes_distribution(
  const geometry_msgs::msg::PoseWithCovariance & post,
  const geometry_msgs::msg::PoseWithCovariance & prior)
{
  // Infer measurement distribution
  Eigen::Vector2f post_pos, prior_pos;
  Eigen::Matrix2f post_cov, prior_cov;
  {
    auto x1 = prior.pose.position;
    auto x2 = post.pose.position;
    prior_pos << x1.x, x1.y;
    post_pos << x2.x, x2.y;

    auto c1 = prior.covariance;
    auto c2 = post.covariance;
    prior_cov << c1[6 * 0 + 0], c1[6 * 0 + 1], c1[6 * 1 + 0], c1[6 * 1 + 1];
    post_cov << c2[6 * 0 + 0], c2[6 * 0 + 1], c2[6 * 1 + 0], c2[6 * 1 + 1];
  }

  const Eigen::Matrix2f epsilon = 1e-4f * Eigen::Matrix2f::Identity();
  const Eigen::Matrix2f post_info = (post_cov + epsilon).inverse();
  const Eigen::Matrix2f prior_info = (prior_cov + epsilon).inverse();

  Eigen::Matrix2f measure_info = post_info - prior_info;

  // Check whether info matrix is positive semi-definite or not
  float det = measure_info.determinant();
  while (det < 1e-4f || measure_info(0, 0) < 1e-3f || measure_info(1, 1) < 1e-3f) {
    measure_info += 0.1 * prior_info;
    std::cout << "avoiding non positive semi-definite " << det << std::endl;
    det = measure_info.determinant();
  }
  if (measure_info(0, 0) < 1e-3f || measure_info(1, 1) < 1e-3f) {
    std::cerr << "measure_info is weird" << std::endl;
    std::cerr << post_info << std::endl;
    std::cerr << measure_info << std::endl;
    std::cerr << prior_info << std::endl;
    geometry_msgs::msg::PoseWithCovariance measure = prior;
    measure.covariance[6 * 0 + 0] = 100;
    measure.covariance[6 * 0 + 1] = 0;
    measure.covariance[6 * 1 + 0] = 0;
    measure.covariance[6 * 1 + 1] = 100;
    measure.covariance[6 * 5 + 5] = 0.25;
    return measure;
  }

  const Eigen::Matrix2f measure_cov = (measure_info).inverse();
  const Eigen::Vector2f measure_pos = post_pos;
  // const Eigen::Vector2f measure_pos =
  //   prior_pos + (prior_cov + measure_cov) * prior_info * (post_pos - prior_pos);

  // TODO: De-bayesing for orientation

  geometry_msgs::msg::PoseWithCovariance measure = post;
  measure.pose.position.x = measure_pos.x();
  measure.pose.position.y = measure_pos.y();
  measure.covariance[6 * 0 + 0] = measure_cov(0, 0);
  measure.covariance[6 * 0 + 1] = measure_cov(0, 1);
  measure.covariance[6 * 1 + 0] = measure_cov(1, 0);
  measure.covariance[6 * 1 + 1] = measure_cov(1, 1);
  std::cout << "prior_cov\n" << prior_cov << std::endl;
  std::cout << "post_cov\n" << post_cov << std::endl;
  std::cout << "measure_cov\n" << measure_cov << std::endl;
  std::cout << "post_pos: " << post_pos.transpose() << std::endl;
  std::cout << "measure_pos: " << measure_pos.transpose() << std::endl;

  if ((measure_pos - prior_pos).norm() > 5) {
    std::cout << "\033[35m";
    std::cout << "too far measurement!!!!!\n";
    std::cout << "\033[0m" << std::endl;
  }

  return measure;
}

Eigen::Vector2d nrand_2d(const Eigen::Matrix2d & cov)
{
  Eigen::JacobiSVD<Eigen::Matrix2d> svd;
  svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Vector2d std = svd.singularValues();
  std = std.cwiseMax(0.01);

  std::normal_distribution<> dist(0.0, 1.0);
  Eigen::Vector2d xy;
  xy.x() = std::sqrt(std.x()) * dist(engine);
  xy.y() = std::sqrt(std.y()) * dist(engine);
  return svd.matrixU() * xy;
}

NormalDistribution2d::NormalDistribution2d(const Eigen::Matrix2d & cov)
{
  Eigen::JacobiSVD<Eigen::Matrix2d> svd;
  svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  std_ = svd.singularValues().cwiseMax(0.01).cwiseSqrt();
  rotation_ = svd.matrixU();
}

std::pair<double, Eigen::Vector2d> NormalDistribution2d::operator()() const
{
  std::normal_distribution<> dist(0.0, 1.0);
  const float x = dist(engine);
  const float y = dist(engine);

  float prob = 1 / (2 * M_PI) * std::exp(-0.5 * (x * x + y * y));

  Eigen::Vector2d xy;
  xy.x() = std_.x() * x;
  xy.y() = std_.y() * y;
  return {prob, rotation_ * xy};
}

double mean_radian(const std::vector<double> & angles, const std::vector<double> & weights)
{
  std::complex<double> c{};
  for (int i{0}; i < static_cast<int>(angles.size()); i++) {
    c += weights[i] * std::polar(1.0, angles[i]);
  }
  std::complex<double> cw{std::accumulate(weights.begin(), weights.end(), 0.0)};
  return std::arg(c / cw);
}

MeanResult compile_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array)
{
  using Pose = geometry_msgs::msg::Pose;
  using Particle = modularized_particle_filter_msgs::msg::Particle;

  double sum_weight = std::accumulate(
    particle_array.particles.begin(), particle_array.particles.end(), 0.0,
    [](double weight, const Particle & particle) { return weight + particle.weight; });

  if (std::isinf(sum_weight)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("meanPose"), "sum_weight: " << sum_weight);
  }

  // (1) compute mean
  Pose mean_pose;
  std::vector<double> yaws;
  std::vector<double> normalized_weights;
  for (const Particle & particle : particle_array.particles) {
    const double normalized_weight = particle.weight / sum_weight;

    mean_pose.position.x += particle.pose.position.x * normalized_weight;
    mean_pose.position.y += particle.pose.position.y * normalized_weight;
    mean_pose.position.z += particle.pose.position.z * normalized_weight;

    double yaw, pitch, roll;
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);
    yaws.push_back(yaw);
    normalized_weights.push_back(normalized_weight);
  }

  const Eigen::Vector3f mean(mean_pose.position.x, mean_pose.position.y, mean_pose.position.z);
  const double mean_yaw = mean_radian(yaws, normalized_weights);

  // (2) compute position covariance
  const int N = particle_array.particles.size();
  Eigen::Matrix3f cov_xyz = Eigen::Matrix3f::Zero();
  for (int i = 0; i < N; i++) {
    const auto p = particle_array.particles.at(i);
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    cov_xyz += normalized_weights.at(i) * (d * d.transpose());
  }

  // (3) compute position covariance
  std::vector<double> diff_yaws;
  float cov_theta = 0;
  for (int i = 0; i < N; i++) {
    double yaw = yaws.at(i);
    double d = yaw - mean_yaw;  // NOTE: Be careful!
    cov_theta += normalized_weights.at(i) * (d * d);
  }

  // (4) Assemble all data
  MeanResult result;
  result.pose_.position.x = mean.x();
  result.pose_.position.y = mean.y();
  result.pose_.position.z = mean.z();
  result.pose_.orientation.w = std::cos(mean_yaw / 2.0);
  result.pose_.orientation.z = std::sin(mean_yaw / 2.0);
  result.cov_xyz_ = cov_xyz;
  result.cov_theta_ = cov_theta;
  return result;
}

}  // namespace yabloc::ekf_corrector