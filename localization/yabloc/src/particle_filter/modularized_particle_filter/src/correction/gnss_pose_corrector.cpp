#include <iostream>

#include <boost/circular_buffer.hpp>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "modularized_particle_filter/correction/correction_util.hpp"
#include "modularized_particle_filter/correction/gnss_pose_corrector.hpp"

namespace
{

float normalPDF(
  float x, float mu, float sigma,
  float inv_sqrt_2pi = 0.3989422804014327f)
{
  float a = (x - mu) / sigma;
  return inv_sqrt_2pi / sigma * std::exp(-0.5f * a * a);
}

} // namespace

GNSSPoseCorrector::GNSSPoseCorrector()
: Node("gnss_pose_corrector"),
  particles_buffer_size_(declare_parameter("particles_buffer_size", 50)),
  flat_radius_(declare_parameter("flat_radius", 1.0f)),
  particles_circular_buffer_(particles_buffer_size_),
  pose_buffer_size_(10),
  pose_circular_buffer_(pose_buffer_size_)
{
  prev_time_ = this->now();

  weighted_particle_pub_ =
    this->create_publisher<modularized_particle_filter_msgs::msg::ParticleArray>(
    "weighted_particles", 10);

  particle_sub_ =
    this->create_subscription<modularized_particle_filter_msgs::msg::ParticleArray>(
    "predicted_particles", 10,
    std::bind(&GNSSPoseCorrector::particleCallback, this, std::placeholders::_1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pose_with_covariance", 10,
    std::bind(&GNSSPoseCorrector::poseCallback, this, std::placeholders::_1));
}

void GNSSPoseCorrector::particleCallback(
  const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr particles)
{

  particles_circular_buffer_.push_front(*particles);

  correctAndPublishParticles();
}

void GNSSPoseCorrector::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
{

  pose_circular_buffer_.push_front(*pose_msg);

  correctAndPublishParticles();
}

void GNSSPoseCorrector::correctAndPublishParticles()
{

  if (particles_circular_buffer_.size() < 2 ||
    pose_circular_buffer_.size() == 0)
  {
    return;
  }

  while (rclcpp::Time(pose_circular_buffer_.back().header.stamp) <
    rclcpp::Time(particles_circular_buffer_.back().header.stamp))
  {

    pose_circular_buffer_.pop_back();

    if (pose_circular_buffer_.size() == 0) {
      return;
    }
  }

  if (rclcpp::Time(particles_circular_buffer_.front().header.stamp) <
    rclcpp::Time(pose_circular_buffer_.back().header.stamp))
  {
    return;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped synced_pose{
    pose_circular_buffer_.back()};
  pose_circular_buffer_.pop_back();

  modularized_particle_filter_msgs::msg::ParticleArray synced_particles{
    findSyncedParticles(particles_circular_buffer_, synced_pose.header.stamp)
    .value()};

  // ROS_WARN_STREAM(
  //     "dt: "
  //     << (synced_pose.header.stamp - synced_particles.header.stamp).toSec());

  modularized_particle_filter_msgs::msg::ParticleArray weighted_particles{
    calculateWeightedParticles(synced_particles, synced_pose, flat_radius_)};

  weighted_particle_pub_->publish(weighted_particles);
}

modularized_particle_filter_msgs::msg::ParticleArray
GNSSPoseCorrector::calculateWeightedParticles(
  modularized_particle_filter_msgs::msg::ParticleArray predicted_particles,
  geometry_msgs::msg::PoseWithCovarianceStamped pose, float flat_radius = 1.0)
{

  modularized_particle_filter_msgs::msg::ParticleArray weighted_particles{
    predicted_particles};

  float sigma{static_cast<float>(
      std::sqrt(
        pose.pose.covariance[0] * pose.pose.covariance[0] +
        pose.pose.covariance[7] * pose.pose.covariance[7]))};

  for (int i{0}; i < static_cast<int>(predicted_particles.particles.size()); i++) {
    float distance{static_cast<float>(
        std::hypot(
          predicted_particles.particles[i].pose.position.x -
          pose.pose.pose.position.x,
          predicted_particles.particles[i].pose.position.y -
          pose.pose.pose.position.y))};
    if (distance < flat_radius) {
      weighted_particles.particles[i].weight = normalPDF(0.0, 0.0, sigma);
    } else {
      weighted_particles.particles[i].weight =
        normalPDF(distance - flat_radius, 0.0, sigma);
    }
    // ROS_WARN_STREAM("i: " << i << ", distance: " << distance << ", sigma: "
    //                       << sigma << ", w: " <<
    //                       weighted_particles.particles[i].weight);
  }

  return weighted_particles;
}
