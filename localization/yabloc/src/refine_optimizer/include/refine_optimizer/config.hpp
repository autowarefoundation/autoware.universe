#pragma once
#include <rclcpp/node.hpp>

namespace refine_optimizer
{
struct RefineConfig
{
  RefineConfig(rclcpp::Node * node);

  RefineConfig(bool verbose = false, int max_iteration = 50, double euler_bound = 0.1)
  : verbose_(verbose), max_iteration_(max_iteration), euler_bound_(euler_bound)
  {
    long_bound_ = 0.1;
    late_bound_ = 1.0;
    height_bound_ = 0.1;
  }
  bool verbose_;
  int max_iteration_;
  double euler_bound_;
  double long_bound_;
  double late_bound_;
  double height_bound_;
};

}  // namespace refine_optimizer