#include "pose_estimator_manager/switch_rule/eagleye_area.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>

namespace multi_pose_estimator
{
using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

struct EagleyeArea::Impl
{
  std::vector<BoostPolygon> bounding_boxes_;
  void init(HADMapBin::ConstSharedPtr msg);
  bool within(geometry_msgs::msg::Point & point) const;
};

EagleyeArea::EagleyeArea()
{
  impl_ = std::make_shared<Impl>();
}

void EagleyeArea::init(HADMapBin::ConstSharedPtr msg)
{
  impl_->init(msg);
}

bool EagleyeArea::within(geometry_msgs::msg::Point & point)
{
  return impl_->within(point);
}

void EagleyeArea::Impl::init(HADMapBin::ConstSharedPtr msg)
{
  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map);

  const auto & po_layer = lanelet_map->polygonLayer;
  const std::unordered_set<std::string> bounding_box_labels_ = {"eagleye_area"};

  BoostPolygon poly;
  for (const auto & polygon : po_layer) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) {
      continue;
    }

    lanelet::Attribute attr = polygon.attribute(lanelet::AttributeName::Type);

    if (bounding_box_labels_.count(attr.value()) == 0) {
      continue;
    }

    for (const lanelet::ConstPoint3d & p : polygon) {
      poly.outer().push_back(BoostPoint(p.x(), p.y()));
    }

    bounding_boxes_.push_back(poly);
    poly.outer().clear();
  }

  bounding_boxes_.push_back(poly);
}

bool EagleyeArea::Impl::within(geometry_msgs::msg::Point & point) const
{
  const BoostPoint boost_point(point.x, point.y);
  for (const BoostPolygon & box : bounding_boxes_) {
    if (boost::geometry::disjoint(box, boost_point)) {
      return true;
    }
  }
  return false;
}

}  // namespace multi_pose_estimator