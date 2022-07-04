#pragma once
#include <rclcpp/rclcpp.hpp>

#include "vmvl_msgs/srv/ground.hpp"
#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace map
{
class GroundServer : public rclcpp::Node
{
public:
  using HADMapBin = autoware_auto_mapping_msgs::msg::HADMapBin;
  using Ground = vmvl_msgs::srv::Ground;

  GroundServer();

private:
  rclcpp::Service<Ground>::SharedPtr service_;

  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_{nullptr};
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_{nullptr};

  void mapCallback(const HADMapBin & msg);
  void computeGround(
    const std::shared_ptr<Ground::Request> request, std::shared_ptr<Ground::Response> response);
};

}  // namespace map