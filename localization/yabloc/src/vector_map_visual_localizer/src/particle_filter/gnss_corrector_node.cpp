#include "particle_filter/gnss_corrector.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<particle_filter::GnssParticleCorrector>());
  rclcpp::shutdown();
  return 0;
}