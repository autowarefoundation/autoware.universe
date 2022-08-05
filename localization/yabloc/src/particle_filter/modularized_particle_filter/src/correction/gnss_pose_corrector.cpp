#include "modularized_particle_filter/correction/gnss_pose_corrector.hpp"
namespace modularized_particle_filter
{

GNSSPoseCorrector::GNSSPoseCorrector()
: AbstCorrector("gnss_pose_corrector"),
  flat_radius_(declare_parameter("flat_radius", 1.0f)),
  min_prob_(declare_parameter("min_prob", 0.01f))
{
  using std::placeholders::_1;
  auto pose_callback = std::bind(&GNSSPoseCorrector::poseCallback, this, _1);
  pose_cov_sub_ =
    create_subscription<PoseWithCovarianceStamped>("/pose_with_covariance", 10, pose_callback);

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
}

void GNSSPoseCorrector::poseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
{
  const rclcpp::Time stamp = pose_msg->header.stamp;

  std::optional<ParticleArray> opt_particles = getSyncronizedParticleArray(stamp);
  if (!opt_particles.has_value()) return;

  auto dt = (stamp - rclcpp::Time(opt_particles->header.stamp));
  RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt.seconds());

  ParticleArray weighted_particles{calculateWeightedParticles(opt_particles.value(), *pose_msg)};
  setWeightedParticleArray(weighted_particles);
}

GNSSPoseCorrector::ParticleArray GNSSPoseCorrector::calculateWeightedParticles(
  const ParticleArray & predicted_particles, PoseWithCovarianceStamped pose)
{
  ParticleArray weighted_particles{predicted_particles};
  float sigma{static_cast<float>(std::hypot(pose.pose.covariance[0], pose.pose.covariance[7]))};

  for (int i{0}; i < static_cast<int>(predicted_particles.particles.size()); i++) {
    float distance{static_cast<float>(std::hypot(
      predicted_particles.particles[i].pose.position.x - pose.pose.pose.position.x,
      predicted_particles.particles[i].pose.position.y - pose.pose.pose.position.y))};
    if (distance < flat_radius_) {
      weighted_particles.particles[i].weight = 1.0f;
    } else {
      weighted_particles.particles[i].weight = normalPDF(distance - flat_radius_, 0.0, sigma);
    }
    weighted_particles.particles[i].weight =
      std::max(weighted_particles.particles[i].weight, min_prob_);
  }

  return weighted_particles;
}

float GNSSPoseCorrector::normalPDF(float x, float mu, float sigma)
{
  // NOTE: This is not exact normal distribution
  float a = (x - mu) / sigma;
  return std::exp(-0.5f * a * a);
}
}  // namespace modularized_particle_filter