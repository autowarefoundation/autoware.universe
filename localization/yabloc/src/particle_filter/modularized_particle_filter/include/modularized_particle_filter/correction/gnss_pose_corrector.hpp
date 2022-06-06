#ifndef MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
#define MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_

#include "modularized_particle_filter/correction/abst_corrector.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <modularized_particle_filter_msgs/msg/particle_array.hpp>

#include <boost/circular_buffer.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

class GNSSPoseCorrector : public AbstCorrector
{
public:
  GNSSPoseCorrector();

private:
  using ParticleArray = modularized_particle_filter_msgs::msg::ParticleArray;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  // Circular buffer
  const int pose_buffer_size_;
  const float flat_radius_;
  boost::circular_buffer<PoseWithCovarianceStamped> pose_circular_buffer_;

  void poseCallback(const PoseWithCovarianceStamped::ConstSharedPtr pose_msg);
  ParticleArray calculateWeightedParticles(
    const ParticleArray & predicted_particles, PoseWithCovarianceStamped pose);

  float normalPDF(float x, float mu, float sigma, float inv_sqrt_2pi = 0.3989422804014327f);
};

#endif  // MODULARIZED_PARTICLE_FILTER__CORRECTION__GNSS_POSE_CORRECTOR_HPP_
