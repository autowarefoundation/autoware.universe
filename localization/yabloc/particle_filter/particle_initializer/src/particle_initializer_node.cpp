#include "particle_initializer/particle_initializer.hpp"

int main(int argc, char * argv[])
{
  namespace mpf = modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::ParticleInitializer>());
  rclcpp::shutdown();
  return 0;
}
