#include "modularized_particle_filter/correction/camera_pose_corrector.hpp"

#include <memory>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPoseCorrector>());
  rclcpp::shutdown();
  return 0;
}
