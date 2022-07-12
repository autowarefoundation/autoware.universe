#include "common/util.hpp"
#include "map/ll2_decomposer.hpp"
#include "map/ll2_util.hpp"

#include <pcl_conversions/pcl_conversions.h>

namespace map
{
Ll2Decomposer::Ll2Decomposer() : Node("ll2_to_image")
{
  using std::placeholders::_1;
  const rclcpp::QoS latch_qos = rclcpp::QoS(10).transient_local();
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  // Publisher
  pub_cloud_ = create_publisher<Cloud2>("/ll2_road_marking", latch_qos);
  pub_sign_board_ = create_publisher<Cloud2>("/ll2_sign_board", latch_qos);
  pub_marker_ = create_publisher<MarkerArray>("/sign_board_marker", latch_qos);

  // Subscriber
  auto cb_map = std::bind(&Ll2Decomposer::mapCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
}

void Ll2Decomposer::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(msg);

  const rclcpp::Time stamp = msg.header.stamp;
  const std::set<std::string> sign_board_labels = {"sign-board"};
  const std::set<std::string> road_marking_labels = {
    "zebra_marking", "virtual", "line_thin", "line_thick", "pedestrian_marking", "stop_line"};

  pcl::PointCloud<pcl::PointNormal> ll2_sign_board =
    extractSpecifiedLineString(lanelet_map->lineStringLayer, sign_board_labels);
  pcl::PointCloud<pcl::PointNormal> ll2_road_marking =
    extractSpecifiedLineString(lanelet_map->lineStringLayer, road_marking_labels);

  publishSignMarker(lanelet_map->lineStringLayer);
  util::publishCloud(*pub_sign_board_, ll2_sign_board, stamp);
  util::publishCloud(*pub_cloud_, ll2_road_marking, stamp);
}

pcl::PointCloud<pcl::PointNormal> Ll2Decomposer::extractSpecifiedLineString(
  const lanelet::LineStringLayer & line_strings, const std::set<std::string> & visible_labels)
{
  pcl::PointCloud<pcl::PointNormal> extracted;

  for (const lanelet::ConstLineString3d & line : line_strings) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d to : line) {
      if (from.has_value()) {
        pcl::PointNormal pn = toPointNormal(from.value(), to);
        extracted.push_back(pn);
      }
      from = to;
    }
  }
  return extracted;
}

pcl::PointNormal Ll2Decomposer::toPointNormal(
  const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to) const
{
  pcl::PointNormal pn;
  pn.x = from.x();
  pn.y = from.y();
  pn.z = from.z();
  pn.normal_x = to.x();
  pn.normal_y = to.y();
  pn.normal_z = to.z();
  return pn;
}

void Ll2Decomposer::publishSignMarker(const lanelet::LineStringLayer & line_string_layer)
{
  const std::unordered_set<std::string> visible_labels = {"sign-board"};

  lanelet::ConstLineStrings3d sign_boards;
  for (const lanelet::ConstLineString3d & line : line_string_layer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    sign_boards.push_back(line);
  }

  MarkerArray marker_array;
  int id = 0;
  for (const lanelet::ConstLineString3d & sign_board : sign_boards) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.type = Marker::LINE_STRIP;
    marker.color = util::color(1.0f, 1.0f, 1.0f, 1.0f);
    marker.scale.x = 0.1;
    marker.id = id++;

    for (const lanelet::ConstPoint3d & p : sign_board) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      marker.points.push_back(gp);
    }
    marker_array.markers.push_back(marker);
  }

  pub_marker_->publish(marker_array);
}

}  // namespace map