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
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using Pose = geometry_msgs::msg::Pose;

  ParticleArray normalized_particle_array{particle_array};
  Pose mean_pose;

  auto minmax_weight = std::minmax_element(
    normalized_particle_array.particles.begin(), normalized_particle_array.particles.end(),
    [](auto p1, auto p2) { return p1.weight < p2.weight; });

  const float num_of_particles_inv{1.0f / normalized_particle_array.particles.size()};
  const float dif_weight{minmax_weight.second->weight - minmax_weight.first->weight};

  for (auto & particle : normalized_particle_array.particles) {
    if (dif_weight != 0.0f) {
      particle.weight = (particle.weight - minmax_weight.first->weight) / dif_weight;
    } else {
      particle.weight = num_of_particles_inv;
    }
  }

  double sum_weight{std::accumulate(
    normalized_particle_array.particles.begin(), normalized_particle_array.particles.end(), 0.0,
    [](double weight, modularized_particle_filter_msgs::msg::Particle & particle) {
      return weight + particle.weight;
    })};

  if (std::isinf(sum_weight)) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("meanPose"), "sum_weight: " << sum_weight);
  }

  std::vector<double> rolls{};
  std::vector<double> pitches{};
  std::vector<double> yaws{};
  std::vector<double> weights{};
  for (modularized_particle_filter_msgs::msg::Particle particle :
       normalized_particle_array.particles) {
    double weight{1.0 / normalized_particle_array.particles.size()};
    if (0.0f < sum_weight) weight = particle.weight / sum_weight;

    mean_pose.position.x += particle.pose.position.x * weight;
    mean_pose.position.y += particle.pose.position.y * weight;
    mean_pose.position.z += particle.pose.position.z * weight;

    double yaw{0.0}, pitch{0.0}, roll{0.0};
    tf2::getEulerYPR(particle.pose.orientation, yaw, pitch, roll);

    rolls.push_back(roll);
    pitches.push_back(pitch);
    yaws.push_back(yaw);
    weights.push_back(weight);
  }

  const double mean_roll{meanRadian(rolls, weights)};
  const double mean_pitch{meanRadian(pitches, weights)};
  const double mean_yaw{meanRadian(yaws, weights)};

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
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
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
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using Pose = geometry_msgs::msg::Pose;

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