#include "sign_detector/ll2_util.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
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

    sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(map_topic, rclcpp::QoS(10).transient_local().reliable(), std::bind(&Ll2ImageConverter::mapCallback, this, std::placeholders::_1));
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(pose_topic, 10, std::bind(&Ll2ImageConverter::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped&)
  {
  }

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin& msg)
  {
    lanelet::LaneletMapPtr viz_lanelet_map = fromBinMsg(msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << viz_lanelet_map->laneletLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "line: " << viz_lanelet_map->lineStringLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "point: " << viz_lanelet_map->pointLayer.size());

    for (lanelet::LineString3d& line : viz_lanelet_map->lineStringLayer) {
      if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
      lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
      if (attr.value() != "line_thin") continue;

      // for (const lanelet::ConstPoint3d& p : line) {
      // }
    }
  }

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
