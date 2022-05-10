#include "sign_detector/particle_filter.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParticleFilter>());
  rclcpp::shutdown();
  return 0;
}
