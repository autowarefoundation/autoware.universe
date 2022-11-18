#include "modularized_particle_filter/prediction/predictor.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>

int main(int argc, char ** argv)
{
  namespace mpf = pcdless::modularized_particle_filter;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mpf::Predictor>());
  rclcpp::shutdown();
  return 0;
}
