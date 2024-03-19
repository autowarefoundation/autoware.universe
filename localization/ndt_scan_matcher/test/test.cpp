// Copyright 2023 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "../include/ndt_scan_matcher/ndt_scan_matcher_core.hpp"
#include "stub_initialpose_client.hpp"
#include "stub_pcd_loader.hpp"
#include "stub_sensor_pcd_publisher.hpp"
#include "stub_trigger_node_client.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <rcl_yaml_param_parser/parser.h>

#include <memory>
#include <string>
#include <vector>

class TestNDTScanMatcher : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string yaml_path = ament_index_cpp::get_package_share_directory("ndt_scan_matcher") +
                                  "/config/ndt_scan_matcher.param.yaml";

    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      std::cout << "Failed to parse yaml file" << std::endl;
      return;
    }

    const rclcpp::ParameterMap param_map = rclcpp::parameter_map_from(params_st, "");
    rclcpp::NodeOptions node_options;
    for (const auto & param_pair : param_map) {
      for (const auto & param : param_pair.second) {
        node_options.parameter_overrides().push_back(param);
      }
    }
    node_ = std::make_shared<NDTScanMatcher>(node_options);
    rcl_yaml_node_struct_fini(params_st);

    // prepare tf_static "base_link -> sensor_frame"
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    geometry_msgs::msg::TransformStamped tf_static;
    tf_static.header.stamp.sec = 0;
    tf_static.header.stamp.nanosec = 0;
    tf_static.header.frame_id = "base_link";
    tf_static.child_frame_id = "sensor_frame";
    tf_static.transform.translation.x = 0.0;
    tf_static.transform.translation.y = 0.0;
    tf_static.transform.translation.z = 0.0;
    tf_static.transform.rotation.x = 0.0;
    tf_static.transform.rotation.y = 0.0;
    tf_static.transform.rotation.z = 0.0;
    tf_static.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(tf_static);

    // prepare stubs
    pcd_loader_ = std::make_shared<StubPcdLoader>();
    initialpose_client_ = std::make_shared<StubInitialposeClient>();
    trigger_node_client_ = std::make_shared<StubTriggerNodeClient>();
    sensor_pcd_publisher_ = std::make_shared<StubSensorPcdPublisher>();
  }

  std::shared_ptr<NDTScanMatcher> node_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<StubPcdLoader> pcd_loader_;
  std::shared_ptr<StubInitialposeClient> initialpose_client_;
  std::shared_ptr<StubTriggerNodeClient> trigger_node_client_;
  std::shared_ptr<StubSensorPcdPublisher> sensor_pcd_publisher_;
};

TEST_F(TestNDTScanMatcher,
       standard_sequence_for_initial_pose_estimation)  // NOLINT
{
  //---------//
  // Arrange //
  //---------//

  // prepare input source PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud = make_sample_pcd(-10.0, 10.0, 1.0);
  sensor_msgs::msg::PointCloud2 input_cloud;
  pcl::toROSMsg(cloud, input_cloud);
  input_cloud.header.frame_id = "sensor_frame";
  input_cloud.header.stamp.sec = 1;
  input_cloud.header.stamp.nanosec = 0;

  // prepare input initial pose
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_msg;
  initial_pose_msg.pose.pose.position.x = 10.0;
  initial_pose_msg.pose.pose.position.y = 10.0;
  initial_pose_msg.pose.pose.position.z = 0.0;
  initial_pose_msg.pose.pose.orientation.x = 0.0;
  initial_pose_msg.pose.pose.orientation.y = 0.0;
  initial_pose_msg.pose.pose.orientation.z = 0.0;
  initial_pose_msg.pose.pose.orientation.w = 1.0;
  initial_pose_msg.header.frame_id = "map";
  initial_pose_msg.header.stamp.sec = 0;
  initial_pose_msg.header.stamp.nanosec = 0;

  // start threads
  std::thread t1([&]() {
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node_);
    exec.spin();
  });
  std::thread t2([&]() { rclcpp::spin(pcd_loader_); });

  //-----//
  // Act //
  //-----//
  // (1) trigger initial pose estimation
  EXPECT_TRUE(trigger_node_client_->send_trigger_node(true));

  // (2) publish LiDAR point cloud
  sensor_pcd_publisher_->publish_pcd(input_cloud);

  // (3) send initial pose
  const geometry_msgs::msg::Pose result_pose =
    initialpose_client_->send_initialpose(initial_pose_msg).pose.pose;

  //--------//
  // Assert //
  //--------//
  RCLCPP_INFO_STREAM(
    node_->get_logger(), std::fixed << "result_pose: " << result_pose.position.x << ", "
                                    << result_pose.position.y << ", " << result_pose.position.z);
  EXPECT_NEAR(result_pose.position.x, 10.0, 3.0);
  EXPECT_NEAR(result_pose.position.y, 10.0, 3.0);
  EXPECT_NEAR(result_pose.position.z, 0.0, 3.0);

  rclcpp::shutdown();
  t1.join();
  t2.join();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
