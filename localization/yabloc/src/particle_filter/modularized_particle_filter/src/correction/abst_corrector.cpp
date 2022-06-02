#include "modularized_particle_filter/correction/abst_corrector.hpp"

AbstCorrector::AbstCorrector(const std::string & node_name) : Node(node_name)
{
  using std::placeholders::_1;
  particle_pub_ = this->create_publisher<ParticleArray>("/weighted_particles", 10);
  particle_sub_ = this->create_subscription<ParticleArray>(
    "/predicted_particles", 10, std::bind(&AbstCorrector::particleArrayCallback, this, _1));
}

void AbstCorrector::particleArrayCallback(const ParticleArray & particle_array)
{
  particle_array_buffer_.push_back(particle_array);
}

std::optional<AbstCorrector::ParticleArray> AbstCorrector::getSyncronizedParticleArray(
  const rclcpp::Time & stamp)
{
  auto itr = particle_array_buffer_.begin();
  while (itr != particle_array_buffer_.end()) {
    rclcpp::Duration dt = rclcpp::Time(itr->header.stamp) - stamp;
    if (dt.seconds() < -1.0)
      particle_array_buffer_.erase(itr++);
    else
      break;
  }

  if (particle_array_buffer_.empty()) return std::nullopt;

  auto comp = [stamp](ParticleArray & x1, ParticleArray & x2) -> bool {
    auto dt1 = rclcpp::Time(x1.header.stamp) - stamp;
    auto dt2 = rclcpp::Time(x2.header.stamp) - stamp;
    return std::abs(dt1.seconds()) < std::abs(dt2.seconds());
  };
  return *std::min_element(particle_array_buffer_.begin(), particle_array_buffer_.end(), comp);
}

void AbstCorrector::setWeightedParticleArray(const ParticleArray & particle_array)
{
  particle_pub_->publish(particle_array);
}