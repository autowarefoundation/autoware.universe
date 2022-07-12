#pragma once
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map
{
class Ll2Decomposer : public rclcpp::Node
{
public:
  using Cloud2 = sensor_msgs::msg::PointCloud2;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  Ll2Decomposer();

private:
  rclcpp::Publisher<Cloud2>::SharedPtr pub_cloud_;
  rclcpp::Publisher<Cloud2>::SharedPtr pub_sign_board_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_marker_;

  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  pcl::PointNormal toPointNormal(
    const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to) const;

  pcl::PointCloud<pcl::PointNormal> extractSpecifiedLineString(
    const lanelet::LineStringLayer & line_strings, const std::set<std::string> & visible_labels);

  void mapCallback(const HADMapBin & msg);

  void publishSignMarker(const lanelet::LineStringLayer & line_string_layer);
};
}  // namespace map
