#include "vmvl_map/ll2_decomposer.hpp"
#include "vmvl_map/ll2_util.hpp"

#include <vml_common/color.hpp>
#include <vml_common/pub_sub.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace map
{
Ll2Decomposer::Ll2Decomposer() : Node("ll2_to_image")
{
  using std::placeholders::_1;
  const rclcpp::QoS latch_qos = rclcpp::QoS(10).transient_local();
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  // Publisher
  pub_cloud_ = create_publisher<Cloud2>("ll2_road_marking", latch_qos);
  pub_sign_board_ = create_publisher<Cloud2>("ll2_sign_board", latch_qos);
  pub_marker_ = create_publisher<MarkerArray>("sign_board_marker", latch_qos);

  // Subscriber
  auto cb_map = std::bind(&Ll2Decomposer::mapCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("map/vector_map", map_qos, cb_map);

  {
    // Load road marking labels from ros params
    declare_parameter("road_marking_labels", std::vector<std::string>{});
    auto labels = get_parameter("road_marking_labels").as_string_array();
    for (auto l : labels) road_marking_labels_.insert(l);
    if (road_marking_labels_.empty()) {
      RCLCPP_FATAL_STREAM(
        get_logger(), "There are no road marking labels. No LL2 elements will publish");
    }
  }
}

void Ll2Decomposer::mapCallback(const HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(msg);

  const rclcpp::Time stamp = msg.header.stamp;
  const std::set<std::string> sign_board_labels = {"sign-board"};

  const auto & ls_layer = lanelet_map->lineStringLayer;
  auto tmp1 = extractSpecifiedLineString(ls_layer, sign_board_labels);
  auto tmp2 = extractSpecifiedLineString(ls_layer, road_marking_labels_);
  pcl::PointCloud<pcl::PointNormal> ll2_sign_board = splitLineStrings(tmp1);
  pcl::PointCloud<pcl::PointNormal> ll2_road_marking = splitLineStrings(tmp2);

  publishSignMarker(lanelet_map->lineStringLayer);
  vml_common::publishCloud(*pub_sign_board_, ll2_sign_board, stamp);
  vml_common::publishCloud(*pub_cloud_, ll2_road_marking, stamp);
}

pcl::PointCloud<pcl::PointNormal> Ll2Decomposer::splitLineStrings(
  const lanelet::ConstLineStrings3d & line_strings)
{
  pcl::PointCloud<pcl::PointNormal> extracted;
  for (const lanelet::ConstLineString3d & line : line_strings) {
    lanelet::ConstPoint3d const * from = nullptr;
    for (const lanelet::ConstPoint3d & to : line) {
      if (from != nullptr) {
        pcl::PointNormal pn = toPointNormal(*from, to);
        extracted.push_back(pn);
      }
      from = &to;
    }
  }
  return extracted;
}

lanelet::ConstLineStrings3d Ll2Decomposer::extractSpecifiedLineString(
  const lanelet::LineStringLayer & line_string_layer, const std::set<std::string> & visible_labels)
{
  lanelet::ConstLineStrings3d line_strings;
  for (const lanelet::ConstLineString3d & line : line_string_layer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;
    line_strings.push_back(line);
  }
  return line_strings;
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

Ll2Decomposer::MarkerArray Ll2Decomposer::makeSignMarkerMsg(
  const lanelet::LineStringLayer & line_string_layer, const std::set<std::string> & labels,
  const std::string & ns)
{
  lanelet::ConstLineStrings3d line_strings = extractSpecifiedLineString(line_string_layer, labels);

  MarkerArray marker_array;
  int id = 0;
  for (const lanelet::ConstLineString3d & line_string : line_strings) {
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = get_clock()->now();
    marker.type = Marker::LINE_STRIP;
    marker.color = vml_common::Color(0.6f, 0.6f, 0.6f, 0.999f);
    marker.scale.x = 0.1;
    marker.ns = ns;
    marker.id = id++;

    for (const lanelet::ConstPoint3d & p : line_string) {
      geometry_msgs::msg::Point gp;
      gp.x = p.x();
      gp.y = p.y();
      gp.z = p.z();
      marker.points.push_back(gp);
    }
    marker_array.markers.push_back(marker);
  }
  return marker_array;
}

void Ll2Decomposer::publishSignMarker(const lanelet::LineStringLayer & line_string_layer)
{
  auto marker1 = makeSignMarkerMsg(line_string_layer, {"sign-board"}, "sign_board");
  auto marker2 = makeSignMarkerMsg(line_string_layer, {"virtual"}, "virtual");

  std::copy(marker2.markers.begin(), marker2.markers.end(), std::back_inserter(marker1.markers));
  pub_marker_->publish(marker1);
}

}  // namespace map