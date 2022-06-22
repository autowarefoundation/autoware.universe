#pragma once
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

namespace validation
{
class FreeSpace : public rclcpp::Node
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  FreeSpace();

private:
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg);
};
}  // namespace validation