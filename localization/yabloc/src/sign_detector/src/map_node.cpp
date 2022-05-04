#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "sign_detector/fix2mgrs.hpp"
#include <pcl-1.10/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber(const std::string& map_topic) : Node("map_subscriber"), kdtree_(nullptr)
  {
    sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(map_topic, rclcpp::QoS(10).transient_local(), std::bind(&MapSubscriber::mapCallback, this, std::placeholders::_1));
    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("eagleye/fix", 10, std::bind(&MapSubscriber::fixCallback, this, std::placeholders::_1));
    pub_ground_ = this->create_publisher<visualization_msgs::msg::Marker>("ground", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void publishTf(const geometry_msgs::msg::Pose& pose, const rclcpp::Time&)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = pose.position.x;
    t.transform.translation.y = pose.position.y;
    t.transform.translation.z = pose.position.z;
    t.transform.rotation.w = 1;

    tf_broadcaster_->sendTransform(t);
  }

  void fixCallback(const sensor_msgs::msg::NavSatFix& msg)
  {
    if (!kdtree_) return;


    Eigen::Vector3d mgrs = fix2Mgrs(msg);
    pcl::PointXYZ p;
    p.x = mgrs.x();
    p.y = mgrs.y();
    p.z = mgrs.z();

    constexpr int K = 10;
    std::vector<int> indices;
    std::vector<float> distances;
    kdtree_->nearestKSearch(p, K, indices, distances);

    float height = std::numeric_limits<float>::max();
    for (int index : indices) {
      Eigen::Vector3f v = cloud_->at(index).getVector3fMap();
      height = std::min(height, v.z());
    }

    RCLCPP_INFO_STREAM(this->get_logger(), mgrs.x() << " " << mgrs.y() << " (" << msg.latitude << ", " << msg.longitude << ") " << height);

    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = height;
    publishVisMarker(pose);
    publishTf(pose, msg.header.stamp);
  }

  void publishVisMarker(const geometry_msgs::msg::Pose& pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.pose.position.x = pose.position.x;
    marker.pose.position.y = pose.position.y;
    marker.pose.position.z = pose.position.z;
    marker.pose.orientation.w = 1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    pub_ground_->publish(marker);
  }

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin& msg)
  {
    lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);
    fromBinMsg(msg, viz_lanelet_map);

    RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << viz_lanelet_map->laneletLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "line: " << viz_lanelet_map->lineStringLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "point: " << viz_lanelet_map->pointLayer.size());

    kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();

    auto toPointXYZ = [](const lanelet::ConstPoint3d& p) -> pcl::PointXYZ {
      pcl::PointXYZ q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      return q;
    };

    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (lanelet::LineString3d& line : viz_lanelet_map->lineStringLayer) {
      if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
      lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
      if (attr.value() != "line_thin") continue;

      for (const lanelet::ConstPoint3d& p : line)
        cloud_->push_back(toPointXYZ(p));
    }
    kdtree_->setInputCloud(cloud_);
  }

  void fromBinMsg(const autoware_auto_mapping_msgs::msg::HADMapBin& msg, lanelet::LaneletMapPtr map) const
  {
    if (!map) {
      std::cerr << __FUNCTION__ << ": map is null pointer!";
      return;
    }
    std::string data_str;
    data_str.assign(msg.data.begin(), msg.data.end());

    std::stringstream ss;
    ss << data_str;
    boost::archive::binary_iarchive oa(ss);
    oa >> *map;
    lanelet::Id id_counter;
    oa >> id_counter;
    lanelet::utils::registerId(id_counter);
  }
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_ground_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const std::string map_topic = "/map/vector_map";

  rclcpp::spin(std::make_shared<MapSubscriber>(map_topic));
  rclcpp::shutdown();
  return 0;
}
