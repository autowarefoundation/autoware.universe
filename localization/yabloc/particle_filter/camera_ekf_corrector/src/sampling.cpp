#include "camera_ekf_corrector/sampling.hpp"

#include <pcdless_common/pose_conversions.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/utils.h>

namespace pcdless::ekf_corrector
{
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());

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

}  // namespace pcdless::ekf_corrector