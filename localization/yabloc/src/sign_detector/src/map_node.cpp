#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl-1.10/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.10/pcl/point_cloud.h>

class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber(const std::string& map_topic) : Node("map_subscriber"), kdtree_(nullptr)
  {
    sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(map_topic, rclcpp::QoS(10).transient_local(), std::bind(&MapSubscriber::mapCallback, this, std::placeholders::_1));
    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("eagleye/fix", 10, std::bind(&MapSubscriber::fixCallback, this, std::placeholders::_1));
    pub_ground_ = this->create_publisher<visualization_msgs::msg::Marker>("ground", 10);
  }

private:
  void fixCallback(const sensor_msgs::msg::NavSatFix&) const
  {
    if (!kdtree_) return;
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (lanelet::LineString3d& line : viz_lanelet_map->lineStringLayer) {
      if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
      lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
      if (attr.value() != "line_thin") continue;

      for (const lanelet::ConstPoint3d& p : line)
        cloud->push_back(toPointXYZ(p));
    }

    std::cout << "cloud: " << cloud->size() << std::endl;

    kdtree_->setInputCloud(cloud);
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
