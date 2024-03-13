

#ifndef AUTOWARE__DYNAMIC_LANELET_PROVIDER__DYNAMIC_LANELET_PROVIDER_HPP_
#define AUTOWARE__DYNAMIC_LANELET_PROVIDER__DYNAMIC_LANELET_PROVIDER_HPP_

#include <component_interface_specs/map.hpp>
#include <component_interface_utils/rclcpp.hpp>
#include <geography_utils/lanelet2_projector.hpp>
#include <lanelet2_extension/projection/mgrs_projector.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "autoware_map_msgs/msg/lanelet_map_meta_data.hpp"
#include "autoware_map_msgs/srv/get_differential_lanelet2_map.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tier4_map_msgs/msg/map_projector_info.hpp>

namespace autoware
{
namespace dynamic_lanelet_provider
{

struct Lanelet2FileMetaData
{
  int id;
  double min_x;
  double max_x;
  double min_y;
  double max_y;
};

class DynamicLaneletProviderNode : public rclcpp::Node
{
public:
  explicit DynamicLaneletProviderNode(const rclcpp::NodeOptions & options);

private:
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void mapUpdateTimerCallback();

  void updateMap(const geometry_msgs::msg::Point & pose);
  bool should_update_map() const;

  static void convertLaneletMapBinToHADMapBin(
    const autoware_map_msgs::msg::LaneletMapBin & lanelet_map_bin,
    autoware_auto_mapping_msgs::msg::HADMapBin & had_map_bin);

  rclcpp::Publisher<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr dynamic_map_pub_;

  rclcpp::Client<autoware_map_msgs::srv::GetDifferentialLanelet2Map>::SharedPtr map_loader_client_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  component_interface_utils::Subscription<map_interface::LaneletMapMetaData>::SharedPtr
    lanelet_map_meta_data_sub_;

  rclcpp::TimerBase::SharedPtr map_update_timer_;

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  std::string map_frame_;

  std::optional<geometry_msgs::msg::Point> last_update_position_ = std::nullopt;
  std::optional<geometry_msgs::msg::Point> current_position_ = std::nullopt;

  const double dynamic_map_loading_grid_size_;
  const double dynamic_map_loading_update_distance_;
  const double dynamic_map_loading_map_radius_;

  std::vector<Lanelet2FileMetaData> lanelet_map_meta_data_list_;
};
}  // namespace dynamic_lanelet_provider
}  // namespace autoware

#endif  // AUTOWARE__DYNAMIC_LANELET_PROVIDER__DYNAMIC_LANELET_PROVIDER_HPP_
