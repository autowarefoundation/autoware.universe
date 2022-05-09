#include "sign_detector/ll2_util.hpp"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

class Ll2ImageConverter : public rclcpp::Node
{
public:
  Ll2ImageConverter() : Node("ll2_to_image")
  {
    std::string map_topic = "/map/vector_map";
    std::string pose_topic = "/eagleye/pose";
    this->declare_parameter<std::string>("map_topic", map_topic);
    this->declare_parameter<std::string>("pose_topic", pose_topic);
    this->get_parameter("map_topic", map_topic);
    this->get_parameter("pose_topic", pose_topic);

    pub_image_ = this->create_publisher<sensor_msgs::msg::Image>("/ll2_image", 10);
    sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(map_topic, rclcpp::QoS(10).transient_local().reliable(), std::bind(&Ll2ImageConverter::mapCallback, this, std::placeholders::_1));
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Ll2ImageConverter::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped& pose_stamped)
  {
    if (!visible_linestrings_.has_value()) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 500, "wating for /map/vector_map");
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "try to make image");

    Eigen::Vector3f pose;
    {
      const auto p = pose_stamped.pose.position;
      pose << p.x, p.y, p.z;
    }

    auto toVector3f = [&pose](const lanelet::ConstPoint3d& p) -> Eigen::Vector3f {
      Eigen::Vector3f v;
      v << p.x(), p.y(), p.z();
      return v - pose;
    };

    const float max_range = 20;  // [m]

    auto checkIntersection = [max_range](const Eigen::Vector3f& from, const Eigen::Vector3f& to) -> bool {
      // Compute distance between pose and linesegment of linestring
      Eigen::Vector3f dir = to - from;
      float inner = from.dot(dir);
      if (std::abs(inner) < 1e-3f) {
        return from.norm() < 1.42 * max_range;
      }

      float mu = std::clamp(dir.squaredNorm() / inner, 0.f, 1.0f);
      Eigen::Vector3f nearest = from + dir * mu;
      return nearest.norm() < 1.42 * max_range;
    };

    struct LineString {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      LineString(const Eigen::Vector3f& from, const Eigen::Vector3f& to) : from(from), to(to) {}
      Eigen::Vector3f from, to;
    };

    std::vector<LineString> near_linestring;

    for (const lanelet::LineString3d& line : *visible_linestrings_) {

      std::optional<Eigen::Vector3f> from = std::nullopt;
      for (const lanelet::ConstPoint3d p : line) {
        Eigen::Vector3f to = toVector3f(p);
        if (!from.has_value()) {
          from = to;
          continue;
        }

        if (checkIntersection(*from, to))
          near_linestring.push_back({*from, to});
        from = to;
      }
    }

    float w = pose_stamped.pose.orientation.w;
    float z = pose_stamped.pose.orientation.z;
    Eigen::Matrix2f rotation = Eigen::Rotation2D(-2 * std::atan2(z, w)).matrix();

    const float max_image_size = 600;
    cv::Mat image = cv::Mat::zeros(cv::Size{max_image_size, max_image_size}, CV_8UC3);
    // Draw image
    {
      const cv::Size center(image.cols / 2, image.rows / 2);
      auto toCvPoint = [center, max_range, rotation](const Eigen::Vector3f v) -> cv::Point {
        cv::Point pt;
        Eigen::Vector2f w = rotation * v.topRows(2);
        pt.y = -w.x() / max_range * center.width + center.width;
        pt.x = -w.y() / max_range * center.height + center.height;
        return pt;
      };

      for (const LineString& ls : near_linestring)
        cv::line(image, toCvPoint(ls.from), toCvPoint(ls.to), cv::Scalar(0, 255, 255), 2, cv::LineTypes::LINE_8);
      cv::circle(image, center, 5, cv::Scalar(0, 255, 0), -1);
    }


    // Publish
    publishImage(image, this->get_clock()->now());
  }

  void publishImage(const cv::Mat& image, const rclcpp::Time& stamp)
  {
    cv_bridge::CvImage raw_image;
    raw_image.header.stamp = stamp;
    raw_image.header.frame_id = "map";
    raw_image.encoding = "bgr8";
    raw_image.image = image;
    pub_image_->publish(*raw_image.toImageMsg());
  }

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin& msg)
  {
    lanelet::LaneletMapPtr viz_lanelet_map = fromBinMsg(msg);
    RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << viz_lanelet_map->laneletLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "line: " << viz_lanelet_map->lineStringLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "point: " << viz_lanelet_map->pointLayer.size());

    visible_linestrings_ = lanelet::LineStrings3d{};

    for (lanelet::LineString3d& line : viz_lanelet_map->lineStringLayer) {
      if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
      lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
      if (attr.value() != "line_thin" && attr.value() != "pedestrian_marking") continue;

      visible_linestrings_->push_back(line);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_;
  std::optional<lanelet::LineStrings3d> visible_linestrings_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Ll2ImageConverter>());
  rclcpp::shutdown();
  return 0;
}
