#ifndef BASE_POSE_ESTIMATOR_SUB_MANAGER
#define BASE_POSE_ESTIMATOR_SUB_MANAGER

#include <rclcpp/rclcpp.hpp>

namespace multi_pose_estimator
{
class BasePoseEstimatorSubManager
{
public:
  using SharedPtr = std::shared_ptr<BasePoseEstimatorSubManager>;

  BasePoseEstimatorSubManager(rclcpp::Node * node) : logger_(node->get_logger()) {}

  void enable() { set_enable(true); }
  void disable() { set_enable(false); }

  virtual void set_enable(bool enabled) = 0;

protected:
  rclcpp::Logger logger_;

private:
};
}  // namespace multi_pose_estimator

#endif /* BASE_POSE_ESTIMATOR_SUB_MANAGER */