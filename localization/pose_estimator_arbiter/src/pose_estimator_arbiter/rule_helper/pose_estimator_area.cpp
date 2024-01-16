// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pose_estimator_arbiter/rule_helper/pose_estimator_area.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/utilities.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry/geometry.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <unordered_set>
#include <vector>

namespace pose_estimator_arbiter::rule_helper
{
using BoostPoint = boost::geometry::model::d2::point_xy<double>;
using BoostPolygon = boost::geometry::model::polygon<BoostPoint>;

struct PoseEstimatorArea::Impl
{
  explicit Impl(rclcpp::Logger logger) : logger_(logger) {}
  std::vector<BoostPolygon> bounding_boxes_;

  void init(HADMapBin::ConstSharedPtr msg);
  bool within(const geometry_msgs::msg::Point & point) const;

  std::string debug_string() const;
  MarkerArray debug_marker_array() const { return marker_array_; }

private:
  rclcpp::Logger logger_;
  MarkerArray marker_array_;
};

PoseEstimatorArea::PoseEstimatorArea(const rclcpp::Logger & logger) : logger_(logger)
{
  impl_ = std::make_shared<Impl>(logger_);
}

PoseEstimatorArea::PoseEstimatorArea(rclcpp::Node * node) : PoseEstimatorArea(node->get_logger())
{
}

void PoseEstimatorArea::init(HADMapBin::ConstSharedPtr msg)
{
  impl_->init(msg);
}

bool PoseEstimatorArea::within(
  const geometry_msgs::msg::Point & point, const std::string & pose_estimator_name) const
{
  (void)pose_estimator_name;
  return impl_->within(point);
}

std::string PoseEstimatorArea::debug_string() const
{
  return impl_->debug_string();
}

PoseEstimatorArea::MarkerArray PoseEstimatorArea::debug_marker_array() const
{
  return impl_->debug_marker_array();
}

void PoseEstimatorArea::Impl::init(HADMapBin::ConstSharedPtr msg)
{
  std::cout << "PoseEstimatorArea::Impl::init()" << std::endl;
  if (!bounding_boxes_.empty()) {
    // already initialized
    return;
  }

  lanelet::LaneletMapPtr lanelet_map(new lanelet::LaneletMap);
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map);

  const auto & po_layer = lanelet_map->polygonLayer;
  const std::string pose_estimator_specifying_attribute = "pose_estimator_specify";

  RCLCPP_DEBUG_STREAM(logger_, "Polygon layer size: " << po_layer.size());
  for (const auto & polygon : po_layer) {
    if (!polygon.hasAttribute(lanelet::AttributeName::Type)) {
      continue;
    }

    const lanelet::Attribute attr = polygon.attribute(lanelet::AttributeName::Type);
    RCLCPP_DEBUG_STREAM(logger_, "polygon type: " << attr.value());
    if (pose_estimator_specifying_attribute != attr.value()) {
      continue;
    }

    const std::string subtype{polygon.attributeOr(lanelet::AttributeName::Subtype, "none")};
    RCLCPP_DEBUG_STREAM(logger_, "polygon sub type: " << subtype);

    // Create a marker for visualization
    Marker marker;
    marker.type = Marker::LINE_STRIP;
    marker.scale.set__x(0.2f).set__y(0.2f).set__z(0.2f);
    marker.color.set__r(1.0f).set__g(1.0f).set__b(0.0f).set__a(1.0f);
    marker.ns = "eagleye_area";
    marker.header.frame_id = "map";
    marker.id = marker_array_.markers.size();

    BoostPolygon poly;
    for (const lanelet::ConstPoint3d & p : polygon) {
      poly.outer().push_back(BoostPoint(p.x(), p.y()));

      geometry_msgs::msg::Point point_msg;
      point_msg.set__x(p.x()).set__y(p.y()).set__z(p.z());
      marker.points.push_back(point_msg);
    }
    // Push the first vertex again to enclose the polygon
    poly.outer().push_back(poly.outer().front());

    bounding_boxes_.push_back(poly);
    marker.points.push_back(marker.points.front());
    marker_array_.markers.push_back(marker);
  }
}

bool PoseEstimatorArea::Impl::within(const geometry_msgs::msg::Point & point) const
{
  const BoostPoint boost_point(point.x, point.y);
  for (const BoostPolygon & box : bounding_boxes_) {
    if (!boost::geometry::disjoint(box, boost_point)) {
      return true;
    }
  }
  return false;
}

std::string PoseEstimatorArea::Impl::debug_string() const
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

}  // namespace pose_estimator_arbiter::rule_helper
