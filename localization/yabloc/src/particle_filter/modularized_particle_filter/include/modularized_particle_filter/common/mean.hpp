#ifndef MODULARIZED_PARTICLE_FILTER__COMMON__MEAN_HPP_
#define MODULARIZED_PARTICLE_FILTER__COMMON__MEAN_HPP_

#include <eigen3/Eigen/StdVector>

#include <geometry_msgs/msg/pose.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

namespace modularized_particle_filter
{
geometry_msgs::msg::Pose meanPose(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);

Eigen::Vector3f stdOfDistribution(
  const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);

float stdOfWeight(const modularized_particle_filter_msgs::msg::ParticleArray & particle_array);
}  // namespace modularized_particle_filter

#endif  // MODULARIZED_PARTICLE_FILTER__COMMON__MEAN_HPP_