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

geometry_msgs::msg::Pose mean_pose(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array)
{
  using Pose = geometry_msgs::msg::Pose;
  using Particle = modularized_particle_filter_msgs::msg::Particle;

  Pose mean_pose;

  double sum_weight{std::accumulate(
    particle_array.particles.begin(), particle_array.particles.end(), 0.0,
    [](double weight, const Particle & particle) { return weight + particle.weight; })};

  if (std::isinf(sum_weight)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("meanPose"), "sum_weight: " << sum_weight);
  }

  std::vector<double> rolls{};
  std::vector<double> pitches{};
  std::vector<double> yaws{};
  std::vector<double> normalized_weights{};
  for (const Particle & particle : particle_array.particles) {
    double normalized_weight = particle.weight / sum_weight;

    mean_pose.position.x += particle.pose.position.x * normalized_weight;
    mean_pose.position.y += particle.pose.position.y * normalized_weight;
    mean_pose.position.z += particle.pose.position.z * normalized_weight;

    double yaw{0.0}, pitch{0.0}, roll{0.0};
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);

    rolls.push_back(roll);
    pitches.push_back(pitch);
    yaws.push_back(yaw);
    normalized_weights.push_back(normalized_weight);
  }

  const double mean_roll{mean_radian(rolls, normalized_weights)};
  const double mean_pitch{mean_radian(pitches, normalized_weights)};
  const double mean_yaw{mean_radian(yaws, normalized_weights)};

  tf2::Quaternion q;
  q.setRPY(mean_roll, mean_pitch, mean_yaw);
  mean_pose.orientation = tf2::toMsg(q);
  return mean_pose;
}

Eigen::Matrix3f covariance_of_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & array)
{
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  // TODO: Consider prticle weight
  float invN = 1.f / array.particles.size();
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    mean += affine.translation();
  }
  mean *= invN;

  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const Particle & p : array.particles) {
    Eigen::Affine3f affine = common::pose_to_affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    sigma += (d * d.transpose()) * invN;
  }

  return sigma;
}

float covariance_of_angle_distribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & array)
{
  // TODO: Consider prticle weight
  float invN = 1.f / array.particles.size();
  std::vector<double> yaws;
  std::vector<double> weights;
  for (const auto & particle : array.particles) {
    double yaw, pitch, roll;
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);
    yaws.push_back(yaw);
    weights.push_back(particle.weight);
  }

  const double meaned_radian = mean_radian(yaws, weights);
  double cov = 0;
  for (const double & yaw : yaws) {
    double tmp = (yaw - meaned_radian);
    cov += tmp * tmp;
  }

  return cov * invN;
}

}  // namespace pcdless::ekf_corrector