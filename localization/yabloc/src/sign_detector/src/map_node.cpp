#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <rclcpp/rclcpp.hpp>


class MapSubscriber : public rclcpp::Node
{
public:
  MapSubscriber(const std::string& map_topic) : Node("map_subscriber")
  {
    sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(map_topic, 10, std::bind(&MapSubscriber::mapCallback, this, std::placeholders::_1));
  }

private:
  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin& msg) const
  {
    lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);
    fromBinMsg(msg, viz_lanelet_map);

    std::cout << viz_lanelet_map->laneletLayer.size() << std::endl;
    std::cout << viz_lanelet_map->lineStringLayer.size() << std::endl;
    std::cout << viz_lanelet_map->pointLayer.size() << std::endl;
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
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  const std::string map_topic = "/map/vector_map";

  rclcpp::spin(std::make_shared<MapSubscriber>(map_topic));
  rclcpp::shutdown();
  return 0;
}
