#include "refine_optimizer/config.hpp"

namespace refine_optimizer
{
RefineConfig::RefineConfig(rclcpp::Node * node)
{
  verbose_ = node->declare_parameter<bool>("refine.verbose", false);
  max_iteration_ = node->declare_parameter<int>("refine.max_iteration", 30);
  euler_bound_ = node->declare_parameter<double>("refine.euler_bound", 0.1);

  long_bound_ = node->declare_parameter<double>("refine.long_bound", 0.1);
  late_bound_ = node->declare_parameter<double>("refine.late_bound", 1.0);
  height_bound_ = node->declare_parameter<double>("refine.height_bound", 0.1);
}

}  // namespace refine_optimizer