#include "pose_estimator_manager/base_pose_estimator_sub_manager.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <ublox_msgs/msg/nav_pvt.hpp>

namespace multi_pose_estimator
{
class YabLocSubManager : public rclcpp::Node, BasePoseEstimatorSubManager
{
public:
  using Image = sensor_msgs::msg::Image;
  using NavPVT = ublox_msgs::msg::NavPVT;
  using SetBool = std_srvs::srv::SetBool;

  YabLocSubManager() : Node("yabloc_sub_manager"), BasePoseEstimatorSubManager(this)
  {
    pcdless_is_enabled_ = true;

    using std::placeholders::_1;
    auto on_image = std::bind(&YabLocSubManager::on_image, this, _1);
    auto on_navpvt = std::bind(&YabLocSubManager::on_navpvt, this, _1);

    sub_image_ = create_subscription<Image>("~/input/image", 5, on_image);
    sub_navpvt_ = create_subscription<NavPVT>("~/input/navpvt", 5, on_navpvt);
    pub_image_ = create_publisher<Image>("~/output/image", 5);
    pub_navpvt_ = create_publisher<NavPVT>("~/output/navpvt", 5);

    using namespace std::chrono_literals;
    service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    enable_service_client_ = create_client<SetBool>(
      "~/yabloc_suspend_srv", rmw_qos_profile_services_default, service_callback_group_);
    while (!enable_service_client_->wait_for_service(1s) && rclcpp::ok()) {
      RCLCPP_INFO_STREAM(get_logger(), "Waiting for service... ");
    }
  }

protected:
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  void on_service(
    SetBool::Request::ConstSharedPtr request, SetBool::Response::SharedPtr response) override
  {
    pcdless_is_enabled_ = request->data;
    response->success = true;

    // TODO: start/stop yabloc particle filter
    request_service(request->data);
  }
  void request_service(bool flag)
  {
    using namespace std::chrono_literals;
    auto request = std::make_shared<SetBool::Request>();
    request->data = flag;

    auto result_future = enable_service_client_->async_send_request(request);
    std::future_status status = result_future.wait_for(1000ms);
    if (status == std::future_status::ready) {
      RCLCPP_INFO_STREAM(get_logger(), "yabloc_manager dis/enableing service exited successfully");
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "yabloc_manager dis/enableing service exited unexpectedly");
    }
  }

private:
  bool pcdless_is_enabled_;
  rclcpp::Client<SetBool>::SharedPtr enable_service_client_;
  rclcpp::Subscription<Image>::SharedPtr sub_image_;
  rclcpp::Subscription<NavPVT>::SharedPtr sub_navpvt_;

  rclcpp::Publisher<Image>::SharedPtr pub_image_;
  rclcpp::Publisher<NavPVT>::SharedPtr pub_navpvt_;

  void on_navpvt(NavPVT::ConstSharedPtr msg)
  {
    if (pcdless_is_enabled_) {
      pub_navpvt_->publish(*msg);
    }
  }

  void on_image(Image::ConstSharedPtr msg)
  {
    if (pcdless_is_enabled_) {
      pub_image_->publish(*msg);
    }
  }
};
}  // namespace multi_pose_estimator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<multi_pose_estimator::YabLocSubManager>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
