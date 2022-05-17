#include "sign_detector/ll2_to_image.hpp"
#include "sign_detector/ll2_util.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>

Ll2ImageConverter::Ll2ImageConverter()
: Node("ll2_to_image"), line_thick_(this->declare_parameter<int>("line_thick", 1))
{
  image_size_ = this->declare_parameter<int>("image_size", 800);
  max_range_ = this->declare_parameter<float>("max_range", 20.f);

  std::string map_topic = "/map/vector_map";
  std::string pose_topic = "/eagleye/pose";

  // Publisher
  pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/ll2_image", 10);
  pub_height_ = this->create_publisher<std_msgs::msg::Float32>("/height", 10);
  pub_cloud_ = this->create_publisher<CloudWithPose>("/ll2_cloud", 10);

  using std::placeholders::_1;

  // Subscriber
  sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    map_topic, rclcpp::QoS(10).transient_local().reliable(),
    std::bind(&Ll2ImageConverter::mapCallback, this, _1));
  sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic, 10, std::bind(&Ll2ImageConverter::poseCallback, this, _1));
}

void Ll2ImageConverter::poseCallback(const geometry_msgs::msg::PoseStamped & pose_stamped)
{
  if (!visible_linestrings_.has_value()) {
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

  auto toVector3f = [&pose](const lanelet::ConstPoint3d & p) -> Eigen::Vector3f {
    Eigen::Vector3f v;
    v << p.x(), p.y(), p.z();
    return v - pose;
  };

  auto checkIntersection = [this](
                             const Eigen::Vector3f & from, const Eigen::Vector3f & to) -> bool {
    // Compute distance between pose and linesegment of linestring
    Eigen::Vector3f dir = to - from;
    float inner = from.dot(dir);
    if (std::abs(inner) < 1e-3f) {
      return from.norm() < 1.42 * this->max_range_;
    }

    float mu = std::clamp(dir.squaredNorm() / inner, 0.f, 1.0f);
    Eigen::Vector3f nearest = from + dir * mu;
    return nearest.norm() < 2 * 1.42 * this->max_range_;
  };

  struct LineString
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LineString(const Eigen::Vector3f & from, const Eigen::Vector3f & to) : from(from), to(to) {}
    Eigen::Vector3f from, to;
  };

  std::vector<LineString> near_linestring;

  for (const lanelet::LineString3d & line : *visible_linestrings_) {
    std::optional<Eigen::Vector3f> from = std::nullopt;
    for (const lanelet::ConstPoint3d p : line) {
      Eigen::Vector3f to = toVector3f(p);
      if (!from.has_value()) {
        from = to;
        continue;
      }

      if (checkIntersection(*from, to)) near_linestring.push_back({*from, to});
      from = to;
    }
  }

  float w = pose_stamped.pose.orientation.w;
  float z = pose_stamped.pose.orientation.z;
  Eigen::Matrix2f rotation = Eigen::Rotation2D(-2 * std::atan2(z, w)).matrix();

  cv::Mat image = cv::Mat::zeros(cv::Size{image_size_, image_size_}, CV_8UC3);
  // Draw image
  {
    const cv::Size center(image.cols / 2, image.rows / 2);
    auto toCvPoint = [center, this, rotation](const Eigen::Vector3f v) -> cv::Point {
      cv::Point pt;
      Eigen::Vector2f w = rotation * v.topRows(2);
      pt.x = -w.y() / this->max_range_ * center.width + center.width;
      pt.y = -w.x() / this->max_range_ * center.height + 2 * center.height;
      return pt;
    };

    for (const LineString & ls : near_linestring)
      cv::line(
        image, toCvPoint(ls.from), toCvPoint(ls.to), cv::Scalar(0, 255, 255), line_thick_,
        cv::LineTypes::LINE_8);
  }

  // Publish
  publishImage(image, pose_stamped.header.stamp);
  publishCloud(image, pose_stamped.header.stamp, pose_stamped.pose);

  std_msgs::msg::Float32 height;
  height.data = pose.z();
  pub_height_->publish(height);
}

void Ll2ImageConverter::publishImage(const cv::Mat & image, const rclcpp::Time & stamp)
{
  cv_bridge::CvImage raw_image;
  raw_image.header.stamp = stamp;
  raw_image.header.frame_id = "map";
  raw_image.encoding = "bgr8";
  raw_image.image = image;
  pub_image_->publish(*raw_image.toImageMsg());
}

void Ll2ImageConverter::publishCloud(
  const cv::Mat & image, const rclcpp::Time & stamp, const geometry_msgs::msg::Pose & pose)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::vector<cv::Point2i> nonzero_pix;
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::findNonZero(gray_image, nonzero_pix);
  for (const auto p : nonzero_pix) {
    pcl::PointXYZ xyz;
    xyz.x = p.x;
    xyz.y = p.y;
    xyz.z = 0;
    cloud.push_back(xyz);
  }

  // Convert to msg
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  CloudWithPose msg;
  msg.pose = pose;
  msg.cloud = cloud_msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  pub_cloud_->publish(msg);
}

void Ll2ImageConverter::mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map = fromBinMsg(msg);
  RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << viz_lanelet_map->laneletLayer.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "line: " << viz_lanelet_map->lineStringLayer.size());
  RCLCPP_INFO_STREAM(this->get_logger(), "point: " << viz_lanelet_map->pointLayer.size());

  visible_linestrings_ = lanelet::LineStrings3d{};

  for (lanelet::LineString3d & line : viz_lanelet_map->lineStringLayer) {
    if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
    lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
    if (
      attr.value() != "line_thin" && attr.value() != "pedestrian_marking" &&
      attr.value() != "stop_line")
      continue;

    visible_linestrings_->push_back(line);
  }
}