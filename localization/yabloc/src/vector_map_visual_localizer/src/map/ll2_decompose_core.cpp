#include "common/util.hpp"
#include "map/ll2_decomposer.hpp"
#include "map/ll2_util.hpp"

#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace map
{
Ll2Decomposer::Ll2Decomposer() : Node("ll2_to_image")
{
  using std::placeholders::_1;
  const rclcpp::QoS latch_qos = rclcpp::QoS(10).transient_local();
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  // Publisher
  pub_cloud_ = create_publisher<Cloud2>("/ll2_cloud", latch_qos);
  pub_sign_board_ = create_publisher<Cloud2>("/sign_board", latch_qos);

  // Subscriber
  auto cb_map = std::bind(&Ll2Decomposer::mapCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
}

void Ll2Decomposer::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(msg);
  linestrings_ = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  const std::set<std::string> visible_labels = {
    "zebra_marking", "virtual", "line_thin", "line_thick", "pedestrian_marking", "stop_line"};

  for (lanelet::LineString3d & line : lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      if (from.has_value()) {
        pcl::PointNormal pn;
        pn.x = from->x();
        pn.y = from->y();
        pn.z = from->z();
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = p.z();
        linestrings_->push_back(pn);
      }
      from = p;
    }
  }

  publishSignBoard(lanelet_map->lineStringLayer, msg.header.stamp);
  util::publishCloud(*pub_cloud_, *linestrings_, msg.header.stamp);
}

void Ll2Decomposer::publishSignBoard(
  const lanelet::LineStringLayer & line_strings, const rclcpp::Time & stamp)
{
  pcl::PointCloud<pcl::PointNormal>::Ptr sign_board =
    boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

  auto toPointXYZ = [](const lanelet::ConstPoint3d & p) -> pcl::PointXYZ {
    pcl::PointXYZ q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    return q;
  };

  const std::set<std::string> visible_labels = {"sign-board"};

  for (const lanelet::ConstLineString3d & line : line_strings) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (visible_labels.count(attr.value()) == 0) continue;

    std::optional<lanelet::ConstPoint3d> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      if (from.has_value()) {
        pcl::PointNormal pn;
        pn.x = from->x();
        pn.y = from->y();
        pn.z = from->z();
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = p.z();
        sign_board->push_back(pn);
      }
      from = p;
    }
  }

  // Convert to msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*sign_board, cloud_msg);
  cloud_msg.header.stamp = stamp;
  cloud_msg.header.frame_id = "map";
  pub_sign_board_->publish(cloud_msg);
}

}  // namespace map