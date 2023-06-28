#ifndef MANAGER_CLIENT
#define MANAGER_CLIENT

#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

namespace multi_pose_estimator
{
class ManagerClient
{
public:
  using SharedPtr = std::shared_ptr<ManagerClient>;
  using SetBool = std_srvs::srv::SetBool;

  ManagerClient(
    rclcpp::Node * node, const std::string & service_name,
    rclcpp::CallbackGroup::SharedPtr callback_group)
  : logger_(node->get_logger())
  {
    using namespace std::chrono_literals;
    enable_service_client_ =
      node->create_client<SetBool>(service_name, rmw_qos_profile_services_default, callback_group);

    while (!enable_service_client_->wait_for_service(1s) && rclcpp::ok()) {
      RCLCPP_INFO_STREAM(logger_, "Waiting for service... " << service_name);
    }
  }

  void enable() { request(true); }
  void disable() { request(false); }

private:
  rclcpp::Client<SetBool>::SharedPtr enable_service_client_;
  rclcpp::Logger logger_;

  void request(bool flag)
  {
    using namespace std::chrono_literals;
    auto request = std::make_shared<SetBool::Request>();
    request->data = flag;

    auto result_future = enable_service_client_->async_send_request(request);
    std::future_status status = result_future.wait_for(1000ms);
    if (status == std::future_status::ready) {
      RCLCPP_INFO_STREAM(logger_, "pose_estimator dis/enableing service exited successfully");
    } else {
      RCLCPP_ERROR_STREAM(logger_, "pose_estimator dis/enableing service exited unexpectedly");
    }
  }
};
}  // namespace multi_pose_estimator

#endif /* MANAGER_CLIENT */