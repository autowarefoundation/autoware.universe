#pragma once
#include <pcdless_common/ground_plane.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ground_msgs/srv/ground.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/circular_buffer.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcdless::ground_server
{
class GroundServer : public rclcpp::Node
{
public:
  using GroundPlane = vml_common::GroundPlane;
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Ground = ground_msgs::srv::Ground;
  using Pose = geometry_msgs::msg::Pose;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Float32 = std_msgs::msg::Float32;
  using Float32Array = std_msgs::msg::Float32MultiArray;
  using Marker = visualization_msgs::msg::Marker;
  using String = std_msgs::msg::String;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using Point = geometry_msgs::msg::Point;
  GroundServer();

private:
  const bool force_zero_tilt_;
  const float R;
  const int K;

  rclcpp::Service<Ground>::SharedPtr service_;

  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;
  rclcpp::Subscription<PoseStamped>::SharedPtr sub_pose_stamped_;
  rclcpp::Publisher<Float32>::SharedPtr pub_ground_height_;
  rclcpp::Publisher<Float32Array>::SharedPtr pub_ground_plane_;
  rclcpp::Publisher<Marker>::SharedPtr pub_marker_;
  rclcpp::Publisher<String>::SharedPtr pub_string_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_near_cloud_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  boost::circular_buffer<Eigen::Vector3f> vector_buffer_;

  std::vector<int> last_near_point_indices_;

  void on_map(const HADMapBin & msg);
  void on_pose_stamped(const PoseStamped & msg);
  void on_service(
    const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response);

  GroundPlane estimate_ground(const Point & point);

  float estimate_height_simply(const Point & point) const;

  void publish_marker(const GroundPlane & plane);

  std::vector<int> ransac_estimation(const std::vector<int> & indices_raw);

  void upsample_line_string(
    const lanelet::ConstPoint3d & from, const lanelet::ConstPoint3d & to,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  pcl::PointCloud<pcl::PointXYZ> sample_from_polygons(const lanelet::PolygonLayer & polygons);
};

}  // namespace pcdless::ground_server