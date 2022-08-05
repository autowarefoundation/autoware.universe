#include "modularized_particle_filter/correction/gnss_pose_corrector.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  namespace mpf = modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::GNSSPoseCorrector>());
  rclcpp::shutdown();
  return 0;
}
