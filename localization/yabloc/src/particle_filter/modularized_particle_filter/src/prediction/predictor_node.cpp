#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "modularized_particle_filter/prediction/predictor.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Predictor>());
  rclcpp::shutdown();
  return 0;
}
