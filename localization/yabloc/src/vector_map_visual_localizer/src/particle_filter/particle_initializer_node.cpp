#include "particle_filter/particle_initializer.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<particle_filter::ParticleInitializer>());
  rclcpp::shutdown();
  return 0;
}
