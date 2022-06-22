#include "sign_detector/ll2_util.hpp"
#include "validation/freespace.hpp"

namespace validation
{
FreeSpace::FreeSpace() : rclcpp::Node("freespace")
{
  using std::placeholders::_1;
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();
  auto cb_map = std::bind(&FreeSpace::mapCallback, this, _1);
  sub_map_ = this->create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
}

void FreeSpace::mapCallback(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map = fromBinMsg(msg);
  RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << viz_lanelet_map->laneletLayer.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "line: " << viz_lanelet_map->lineStringLayer.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "point: " << viz_lanelet_map->pointLayer.size());
}

}  // namespace validation