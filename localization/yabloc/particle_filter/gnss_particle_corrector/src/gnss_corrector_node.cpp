#include "gnss_particle_corrector/gnss_particle_corrector.hpp"

int main(int argc, char * argv[])
{
  namespace mpf = pcdless::modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::GnssParticleCorrector>());
  rclcpp::shutdown();
  return 0;
}