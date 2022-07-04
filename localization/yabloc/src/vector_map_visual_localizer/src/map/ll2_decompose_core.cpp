#include "common/util.hpp"
#include "map/ll2_decomposer.hpp"
#include "map/ll2_util.hpp"

#include <opencv4/opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

namespace map
{

Ll2Decomposer::Ll2Decomposer()
: Node("ll2_to_image"),
  line_thick_(declare_parameter<int>("line_thick", 1)),
  image_size_(declare_parameter<int>("image_size", 800)),
  max_range_(declare_parameter<float>("max_range", 20.f))
{
  using std::placeholders::_1;
  const rclcpp::QoS latch_qos = rclcpp::QoS(10).transient_local();
  const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

  // Publisher
  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/ll2_image", 10);
  pub_cloud_ = create_publisher<Cloud2>("/ll2_cloud", latch_qos);
  pub_sign_board_ = create_publisher<Cloud2>("/sign_board", latch_qos);

  // Subscriber
  auto cb_map = std::bind(&Ll2Decomposer::mapCallback, this, _1);
  auto cb_pose = std::bind(&Ll2Decomposer::poseCallback, this, _1);
  sub_map_ = create_subscription<HADMapBin>("/map/vector_map", map_qos, cb_map);
  sub_pose_stamped_ = create_subscription<PoseStamped>("/pose_topic", 10, cb_pose);
}

void Ll2Decomposer::poseCallback(const PoseStamped & pose_stamped)
{
  if (linestrings_ == nullptr) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 500, "wating for /map/vector_map");
    return;
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "try to make image");

  Eigen::Vector3f pose;
  {
    const auto p = pose_stamped.pose.position;
    pose << p.x, p.y, p.z;
  }

  // Compute distance between pose and linesegment of linestring
  auto checkIntersection = [this, pose](const pcl::PointNormal & pn) -> bool {
    const Eigen::Vector3f from = pn.getVector3fMap() - pose;
    const Eigen::Vector3f to = pn.getNormalVector3fMap() - pose;
    Eigen::Vector3f dir = to - from;
    float inner = from.dot(dir);
    if (std::abs(inner) < 1e-3f) {
      return from.norm() < 1.42 * this->max_range_;
    }

    float mu = std::clamp(dir.squaredNorm() / inner, 0.f, 1.0f);
    Eigen::Vector3f nearest = from + dir * mu;
    return nearest.norm() < 2 * 1.42 * this->max_range_;
  };

  float w = pose_stamped.pose.orientation.w;
  float z = pose_stamped.pose.orientation.z;
  Eigen::Matrix3f rotation =
    Eigen::AngleAxisf(-2.f * std::atan2(z, w), Eigen::Vector3f::UnitZ()).matrix();

  pcl::PointCloud<pcl::PointNormal> near_linestring;
  for (const pcl::PointNormal & pn : *linestrings_) {
    if (checkIntersection(pn)) {
      pcl::PointNormal relative_pn;
      relative_pn.getVector3fMap() = rotation * (pn.getVector3fMap() - pose);
      relative_pn.getNormalVector3fMap() = rotation * (pn.getNormalVector3fMap() - pose);
      relative_pn.z = 0;
      relative_pn.normal_z = 0;
      near_linestring.push_back(relative_pn);
    }
  }

  // Draw image
  cv::Mat image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
  const cv::Size center(image.cols / 2, image.rows / 2);
  auto toCvPoint = [center, this, rotation](const Eigen::Vector3f v) -> cv::Point {
    cv::Point pt;
    pt.x = -v.y() / this->max_range_ * center.width + center.width;
    pt.y = -v.x() / this->max_range_ * center.height + 2 * center.height;
    return pt;
  };

  for (const pcl::PointNormal & pn : near_linestring) {
    cv::line(
      image, toCvPoint(pn.getVector3fMap()), toCvPoint(pn.getNormalVector3fMap()),
      cv::Scalar(0, 255, 255), line_thick_, cv::LineTypes::LINE_8);
  }

  // Publish
  util::publishImage(*pub_image_, image, pose_stamped.header.stamp);
  // TODO:
  static bool first_publish_cloud = true;
  if (first_publish_cloud)
    util::publishCloud(*pub_cloud_, *linestrings_, pose_stamped.header.stamp);
  first_publish_cloud = false;
}

void Ll2Decomposer::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  lanelet::LaneletMapPtr lanelet_map = fromBinMsg(msg);
  RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << lanelet_map->laneletLayer.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "line: " << lanelet_map->lineStringLayer.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "point: " << lanelet_map->pointLayer.size());

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
        pn.z = 0;
        pn.normal_x = p.x();
        pn.normal_y = p.y();
        pn.normal_z = 0;
        linestrings_->push_back(pn);
      }
      from = p;
    }
  }

  publishSignBoard(lanelet_map->lineStringLayer, msg.header.stamp);
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