#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "sign_detector/fix2mgrs.hpp"
#include <pcl-1.10/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>

class Fix2Pose : public rclcpp::Node
{
public:
  Fix2Pose() : Node("fix_to_pose"), kdtree_(nullptr)
  {
    std::string map_topic = "/map/vector_map";
    std::string fix_topic = "/eagleye/fix";
    std::string pose_topic = "/eagleye/pose";
    this->declare_parameter<std::string>("map_topic", map_topic);
    this->declare_parameter<std::string>("fix_topic", fix_topic);
    this->declare_parameter<std::string>("pose_topic", pose_topic);
    this->get_parameter("map_topic", map_topic);
    this->get_parameter("fix_topic", fix_topic);
    this->get_parameter("pose_topic", pose_topic);

    sub_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(map_topic, rclcpp::QoS(10).transient_local().reliable(), std::bind(&Fix2Pose::mapCallback, this, std::placeholders::_1));
    sub_fix_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(fix_topic, 10, std::bind(&Fix2Pose::fixCallback, this, std::placeholders::_1));
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void publishTf(const geometry_msgs::msg::PoseStamped& pose, const rclcpp::Time&)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = pose.header.stamp;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = pose.pose.position.x;
    t.transform.translation.y = pose.pose.position.y;
    t.transform.translation.z = pose.pose.position.z;
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

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = msg.header.stamp;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = height;

    pub_pose_stamped_->publish(pose);

    publishTf(pose, msg.header.stamp);
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Fix2Pose>());
  rclcpp::shutdown();
  return 0;
}
