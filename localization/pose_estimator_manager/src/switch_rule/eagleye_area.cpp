#include "pose_estimator_manager/switch_rule/eagleye_area.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

namespace multi_pose_estimator
{
using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;
rclcpp::Logger logger = rclcpp::get_logger("EagleyeArea");

struct EagleyeArea::Impl
{
  std::vector<BoostPolygon> bounding_boxes_;
  void init(HADMapBin::ConstSharedPtr msg);
  bool within(const geometry_msgs::msg::Point & point) const;
  std::string debug_string() const;
  MarkerArray debug_marker_array() const { return marker_array_; }

private:
  MarkerArray marker_array_;
};

EagleyeArea::EagleyeArea()
{
  impl_ = std::make_shared<Impl>();
}

void EagleyeArea::init(HADMapBin::ConstSharedPtr msg)
{
  impl_->init(msg);
}

bool EagleyeArea::within(const geometry_msgs::msg::Point & point) const
{
  return impl_->within(point);
}

std::string EagleyeArea::debug_string() const
{
  return impl_->debug_string();
}

EagleyeArea::MarkerArray EagleyeArea::debug_marker_array() const
{
  return impl_->debug_marker_array();
}

void EagleyeArea::Impl::init(HADMapBin::ConstSharedPtr msg)
{
  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map);

  const auto & po_layer = lanelet_map->polygonLayer;
  const std::unordered_set<std::string> bounding_box_labels_ = {"eagleye_area"};

  RCLCPP_INFO_STREAM(logger, "Polygon layer size: " << po_layer.size());
  for (const auto & polygon : po_layer) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) {
      continue;
    }

    const lanelet::Attribute attr = polygon.attribute(lanelet::AttributeName::Type);
    RCLCPP_INFO_STREAM(logger, "a polygon attribute: " << attr.value());

    if (bounding_box_labels_.count(attr.value()) == 0) {
      continue;
    }

    Marker marker;
    marker.type = Marker::LINE_STRIP;
    marker.scale.set__x(0.2f).set__y(0.2f).set__z(0.2f);
    marker.color.set__r(1.0f).set__g(1.0f).set__b(0.0f).set__a(1.0f);
    marker.ns = "eagleye_area";
    marker.header.frame_id = "map";

    BoostPolygon poly;
    for (const lanelet::ConstPoint3d & p : polygon) {
      poly.outer().push_back(BoostPoint(p.x(), p.y()));

      geometry_msgs::msg::Point point_msg;
      point_msg.set__x(p.x()).set__y(p.y()).set__z(p.z());
      marker.points.push_back(point_msg);
    }

    bounding_boxes_.push_back(poly);
    marker.points.push_back(marker.points.front());
    marker_array_.markers.push_back(marker);
  }
}

bool EagleyeArea::Impl::within(const geometry_msgs::msg::Point & point) const
{
  const BoostPoint boost_point(point.x, point.y);
  for (const BoostPolygon & box : bounding_boxes_) {
    if (!boost::geometry::disjoint(box, boost_point)) {
      return true;
    }
  }
  return false;
}

std::string EagleyeArea::Impl::debug_string() const
{
  std::stringstream ss;
  for (const BoostPolygon & box : bounding_boxes_) {
    for (const auto point : box.outer()) {
      ss << point.x() << "," << point.y() << " ";
    }
    ss << "\n";
  }
  return ss.str();
}

}  // namespace multi_pose_estimator