/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _MAP_PROVIDER_HPP_
#define _MAP_PROVIDER_HPP_

#include "autoware_map_srvs/srv/load_pcd_partially.hpp"
#include "autoware_map_srvs/srv/provide_pcd.hpp"

#include <boost/optional.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MapProvider : public rclcpp::Node
{
public:
  MapProvider();
  ~MapProvider() = default;

private:
  tf2_ros::Buffer tf2_buffer_{get_clock()};
  tf2_ros::TransformListener tf2_listener_;

  // ros param
  float pointcloud_map_radius_;
  float update_threshold_distance_;

  autoware_map_srvs::srv::LoadPCDPartially::Response::SharedPtr pcd_loader_res_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_pub_;
  rclcpp::Service<autoware_map_srvs::srv::ProvidePCD>::SharedPtr pcd_provider_server_;
  rclcpp::Client<autoware_map_srvs::srv::LoadPCDPartially>::SharedPtr pcd_loader_client_;
  rclcpp::TimerBase::SharedPtr update_map_timer_;

  bool pcdProviderServiceCallback(
    autoware_map_srvs::srv::ProvidePCD::Request::SharedPtr req,
    autoware_map_srvs::srv::ProvidePCD::Response::SharedPtr res);
  void updateMapTimerCallback();
};

#endif
