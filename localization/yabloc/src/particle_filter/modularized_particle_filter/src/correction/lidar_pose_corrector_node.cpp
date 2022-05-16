#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "modularized_particle_filter/correction/lidar_pose_corrector.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarPoseCorrector>());
  rclcpp::shutdown();
  return 0;
}
