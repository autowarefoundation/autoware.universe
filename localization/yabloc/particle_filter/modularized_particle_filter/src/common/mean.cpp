#include "modularized_particle_filter/common/mean.hpp"

#include <eigen3/Eigen/Geometry>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>

namespace modularized_particle_filter
{
namespace
{
double meanRadian(const std::vector<double> & angles, const std::vector<double> & weights)
{
  std::complex<double> c{};
  for (int i{0}; i < static_cast<int>(angles.size()); i++) {
    c += weights[i] * std::polar(1.0, angles[i]);
  }
  std::complex<double> cw{std::accumulate(weights.begin(), weights.end(), 0.0)};
  return std::arg(c / cw);
}
}  // namespace

geometry_msgs::msg::Pose meanPose(
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

  const double mean_roll{meanRadian(rolls, normalized_weights)};
  const double mean_pitch{meanRadian(pitches, normalized_weights)};
  const double mean_yaw{meanRadian(yaws, normalized_weights)};

  tf2::Quaternion q;
  q.setRPY(mean_roll, mean_pitch, mean_yaw);
  mean_pose.orientation = tf2::toMsg(q);
  return mean_pose;
}

Eigen::Affine3f pose2Affine(const geometry_msgs::msg::Pose & pose)
{
  const auto pos = pose.position;
  const auto ori = pose.orientation;
  Eigen::Translation3f t(pos.x, pos.y, pos.z);
  Eigen::Quaternionf q(ori.w, ori.x, ori.y, ori.z);
  return t * q;
}

Eigen::Vector3f stdOfDistribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array)
{
  using Particle = modularized_particle_filter_msgs::msg::Particle;
  using Pose = geometry_msgs::msg::Pose;

  Pose mean_pose = meanPose(particle_array);
  auto ori = mean_pose.orientation;
  Eigen::Quaternionf orientation(ori.w, ori.x, ori.y, ori.z);

  float invN = 1.f / particle_array.particles.size();
  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (const Particle & p : particle_array.particles) {
    Eigen::Affine3f affine = pose2Affine(p.pose);
    mean += affine.translation();
  }
  mean *= invN;

  Eigen::Matrix3f sigma = Eigen::Matrix3f::Zero();
  for (const Particle & p : particle_array.particles) {
    Eigen::Affine3f affine = pose2Affine(p.pose);
    Eigen::Vector3f d = affine.translation() - mean;
    d = orientation.conjugate() * d;
    sigma += (d * d.transpose()) * invN;
  }

  return sigma.diagonal().cwiseMax(1e-4f).cwiseSqrt();
}

float stdOfWeight(const modularized_particle_filter_msgs::msg::ParticleArray & particle_array)
{
  using Particle = modularized_particle_filter_msgs::msg::Particle;

  const float invN = 1.f / particle_array.particles.size();
  float mean = 0;
  for (const Particle & p : particle_array.particles) {
    mean += p.weight;
  }
  mean *= invN;

  float sigma = 0.0;
  for (const Particle & p : particle_array.particles) {
    sigma += (p.weight - mean) * (p.weight - mean);
  }
  sigma *= invN;

  return std::sqrt(sigma);
}
}  // namespace modularized_particle_filter