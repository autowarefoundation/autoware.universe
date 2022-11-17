#include "lidar_particle_corrector/lidar_particle_corrector.hpp"

int main(int argc, char ** argv)
{
  namespace mpf = modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::LidarPoseCorrector>());
  rclcpp::shutdown();
  return 0;
}
