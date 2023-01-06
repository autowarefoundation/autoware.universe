#pragma once
#include "corrector_manager/init_area.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pcdless
{
class CorrectorManager : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using SetBool = std_srvs::srv::SetBool;

  CorrectorManager();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_init_area_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Client<SetBool>::SharedPtr client_;

  std::optional<InitArea> init_area_{std::nullopt};
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  void call_service(bool data);

  void on_timer();
  void on_init_area(const PointCloud2 & msg);
  void on_gnss_pose(const PoseStamped & msg);
};
}  // namespace pcdless