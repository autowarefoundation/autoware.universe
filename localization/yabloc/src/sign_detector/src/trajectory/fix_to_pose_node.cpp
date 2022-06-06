#include "sign_detector/fix2mgrs.hpp"
#include "sign_detector/ll2_util.hpp"

#include <rclcpp/rclcpp.hpp>

#include <eagleye_msgs/msg/heading.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <pcl-1.10/pcl/kdtree/kdtree_flann.h>
#include <pcl-1.10/pcl/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>

class Fix2Pose : public rclcpp::Node
{
public:
  Fix2Pose()
  : Node("fix_to_pose"),
    pose_topic_(declare_parameter<std::string>("pose_topic", "/eagleye/pose")),
    covariance_(declare_parameter<float>("covariance", 16.f))
  {
    using autoware_auto_mapping_msgs::msg::HADMapBin;
    using eagleye_msgs::msg::Heading;
    using geometry_msgs::msg::PoseWithCovarianceStamped;
    using sensor_msgs::msg::NavSatFix;
    using std::placeholders::_1;
    const rclcpp::QoS map_qos = rclcpp::QoS(10).transient_local().reliable();

    // Subscriber
    sub_heading_ = create_subscription<Heading>(
      "/eagleye/heading_interpolate_3rd", 10, std::bind(&Fix2Pose::headingCallback, this, _1));
    sub_map_ = create_subscription<HADMapBin>(
      "/map/vector_map", map_qos, std::bind(&Fix2Pose::mapCallback, this, _1));
    sub_fix_ =
      create_subscription<NavSatFix>("/fix_topic", 10, std::bind(&Fix2Pose::fixCallback, this, _1));

    // Publisher
    pub_pose_stamped_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
    pub_pose_covariance_ =
      this->create_publisher<PoseWithCovarianceStamped>("/pose_with_covariance", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  const std::string pose_topic_;
  const float covariance_;
  float heading_;
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<eagleye_msgs::msg::Heading>::SharedPtr sub_heading_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_stamped_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_covariance_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  void publishTf(const geometry_msgs::msg::PoseStamped & pose, const rclcpp::Time &)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "map";
    t.child_frame_id = pose_topic_;
    t.transform.translation.x = pose.pose.position.x;
    t.transform.translation.y = pose.pose.position.y;
    t.transform.translation.z = pose.pose.position.z;
    t.transform.rotation.w = pose.pose.orientation.w;
    t.transform.rotation.x = pose.pose.orientation.x;
    t.transform.rotation.y = pose.pose.orientation.y;
    t.transform.rotation.z = pose.pose.orientation.z;

    tf_broadcaster_->sendTransform(t);
  }

  void headingCallback(const eagleye_msgs::msg::Heading & msg) { heading_ = msg.heading_angle; }

  void fixCallback(const sensor_msgs::msg::NavSatFix & msg)
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

    RCLCPP_INFO_STREAM(
      this->get_logger(), mgrs.x() << " " << mgrs.y() << " (" << msg.latitude << ", "
                                   << msg.longitude << ") " << height);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = msg.header.stamp;
    pose.pose.position.x = p.x;
    pose.pose.position.y = p.y;
    pose.pose.position.z = height;

    pose.pose.orientation.w = std::cos(0.5f * (M_PI_2 - heading_));
    pose.pose.orientation.z = std::sin(0.5f * (M_PI_2 - heading_));

    pub_pose_stamped_->publish(pose);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_covariance;
    pose_covariance.header = pose.header;
    pose_covariance.pose.pose = pose.pose;
    pose_covariance.pose.covariance.at(0) = covariance_;
    pose_covariance.pose.covariance.at(7) = covariance_;
    pose_covariance.pose.covariance.at(14) = covariance_;
    pub_pose_covariance_->publish(pose_covariance);

    publishTf(pose, msg.header.stamp);
  }

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin & msg)
  {
    lanelet::LaneletMapPtr viz_lanelet_map = fromBinMsg(msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "lanelet: " << viz_lanelet_map->laneletLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "line: " << viz_lanelet_map->lineStringLayer.size());
    RCLCPP_INFO_STREAM(this->get_logger(), "point: " << viz_lanelet_map->pointLayer.size());

    kdtree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();

    auto toPointXYZ = [](const lanelet::ConstPoint3d & p) -> pcl::PointXYZ {
      pcl::PointXYZ q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      return q;
    };

    cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (lanelet::LineString3d & line : viz_lanelet_map->lineStringLayer) {
      if (!line.hasAttribute(lanelet::AttributeName::Type)) continue;
      lanelet::Attribute attr = line.attribute(lanelet::AttributeName::Type);
      if (attr.value() != "line_thin") continue;

      for (const lanelet::ConstPoint3d & p : line) cloud_->push_back(toPointXYZ(p));
    }
    kdtree_->setInputCloud(cloud_);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Fix2Pose>());
  rclcpp::shutdown();
  return 0;
}
