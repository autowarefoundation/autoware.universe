#ifndef BASE_POSE_ESTIMATOR_SUB_MANAGER
#define BASE_POSE_ESTIMATOR_SUB_MANAGER

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace multi_pose_estimator
{
class BasePoseEstimatorSubManager
{
public:
  using SetBool = std_srvs::srv::SetBool;

  BasePoseEstimatorSubManager(rclcpp::Node * node)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    auto on_service = std::bind(&BasePoseEstimatorSubManager::on_service, this, _1, _2);
    enable_server_ = node->create_service<SetBool>("~/enable_srv", on_service);
  }

protected:
  virtual void on_service(
    SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response) = 0;

private:
  rclcpp::Service<SetBool>::SharedPtr enable_server_;
};
}  // namespace multi_pose_estimator

#endif /* BASE_POSE_ESTIMATOR_SUB_MANAGER */