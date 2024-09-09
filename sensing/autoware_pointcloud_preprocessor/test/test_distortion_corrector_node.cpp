// Copyright 2024 TIER IV, Inc.
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

// Note: To regenerate the ground truth (GT) for the expected undistorted point cloud values,
// set the "debug_" value to true to display the point cloud values. Then,
// replace the expected values with the newly displayed undistorted point cloud values.
//
// Also, make sure the point stamp, twist stamp, and imu stamp are not identical.
// In the current hardcoded design, timestamp of pointcloud, twist, and imu msg are listed below
// pointcloud (1 msgs, 10points):
// 10.10, 10.11, 10.12, 10.13, 10.14, 10.15, 10.16, 10.17, 10.18, 10.19
// twist (6msgs):
// 10.095, 10.119, 10.143, 10.167, 10.191, 10.215
// imu (6msgs):
// 10.09, 10.117, 10.144, 10.171, 10.198, 10.225

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"
#include "autoware/universe_utils/math/constants.hpp"
#include "autoware/universe_utils/math/trigonometry.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cassert>

class DistortionCorrectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_node");
    distortion_corrector_2d_ =
      std::make_shared<autoware::pointcloud_preprocessor::DistortionCorrector2D>(node_.get(), true);
    distortion_corrector_3d_ =
      std::make_shared<autoware::pointcloud_preprocessor::DistortionCorrector3D>(node_.get(), true);

    // Setup TF
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
    tf_broadcaster_->sendTransform(generateStaticTransformMsg());

    // Spin the node for a while to ensure transforms are published
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() - start < timeout) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void TearDown() override {}

  void checkInput(int ms) { ASSERT_LT(ms, 1000) << "ms should be less than a second."; }

  rclcpp::Time addMilliseconds(rclcpp::Time stamp, int ms)
  {
    checkInput(ms);
    auto ms_in_ns = rclcpp::Duration(0, ms * 1000000);
    return stamp + ms_in_ns;
  }

  rclcpp::Time subtractMilliseconds(rclcpp::Time stamp, int ms)
  {
    checkInput(ms);
    auto ms_in_ns = rclcpp::Duration(0, ms * 1000000);
    return stamp - ms_in_ns;
  }

  geometry_msgs::msg::TransformStamped generateTransformMsg(
    const std::string & parent_frame, const std::string & child_frame, double x, double y, double z,
    double qx, double qy, double qz, double qw)
  {
    rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = timestamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf_msg.transform.rotation.x = qx;
    tf_msg.transform.rotation.y = qy;
    tf_msg.transform.rotation.z = qz;
    tf_msg.transform.rotation.w = qw;
    return tf_msg;
  }

  std::vector<geometry_msgs::msg::TransformStamped> generateStaticTransformMsg()
  {
    // generate defined transformations
    return {
      generateTransformMsg("base_link", "lidar_top", 5.0, 5.0, 5.0, 0.683, 0.5, 0.183, 0.499),
      generateTransformMsg("base_link", "imu_link", 1.0, 1.0, 3.0, 0.278, 0.717, 0.441, 0.453)};
  }

  std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> generateTwistMsg(
    double linear_x, double angular_z, rclcpp::Time stamp)
  {
    auto twist_msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
    twist_msg->header.stamp = stamp;
    twist_msg->header.frame_id = "base_link";
    twist_msg->twist.twist.linear.x = linear_x;
    twist_msg->twist.twist.angular.z = angular_z;
    return twist_msg;
  }

  std::shared_ptr<sensor_msgs::msg::Imu> generateImuMsg(
    double angular_vel_x, double angular_vel_y, double angular_vel_z, rclcpp::Time stamp)
  {
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = stamp;
    imu_msg->header.frame_id = "imu_link";
    imu_msg->angular_velocity.x = angular_vel_x;
    imu_msg->angular_velocity.y = angular_vel_y;
    imu_msg->angular_velocity.z = angular_vel_z;
    return imu_msg;
  }

  std::vector<std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped>> generateTwistMsgs(
    rclcpp::Time pointcloud_timestamp)
  {
    std::vector<std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped>> twist_msgs;
    rclcpp::Time twist_stamp = subtractMilliseconds(pointcloud_timestamp, 5);

    for (int i = 0; i < number_of_twist_msgs_; ++i) {
      auto twist_msg = generateTwistMsg(
        twist_linear_x_ + i * twist_linear_x_increment_,
        twist_angular_z_ + i * twist_angular_z_increment_, twist_stamp);
      twist_msgs.push_back(twist_msg);

      twist_stamp = addMilliseconds(twist_stamp, twist_msgs_interval_ms_);
    }

    return twist_msgs;
  }

  std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> generateImuMsgs(
    rclcpp::Time pointcloud_timestamp)
  {
    std::vector<std::shared_ptr<sensor_msgs::msg::Imu>> imu_msgs;
    rclcpp::Time imu_stamp = subtractMilliseconds(pointcloud_timestamp, 10);

    for (int i = 0; i < number_of_imu_msgs_; ++i) {
      auto imu_msg = generateImuMsg(
        imu_angular_x_ + i * imu_angular_x_increment_,
        imu_angular_y_ + i * imu_angular_y_increment_,
        imu_angular_z_ + i * imu_angular_z_increment_, imu_stamp);
      imu_msgs.push_back(imu_msg);
      imu_stamp = addMilliseconds(imu_stamp, imu_msgs_interval_ms_);
    }

    return imu_msgs;
  }

  sensor_msgs::msg::PointCloud2 generatePointCloudMsg(
    bool generate_points, bool is_lidar_frame, std::string vendor, rclcpp::Time stamp,
    bool use_default_pointcloud, std::vector<Eigen::Vector3f> defined_points,
    std::vector<float> defined_azimuths)
  {
    sensor_msgs::msg::PointCloud2 pointcloud_msg;
    pointcloud_msg.header.stamp = stamp;
    pointcloud_msg.header.frame_id = is_lidar_frame ? "lidar_top" : "base_link";
    pointcloud_msg.height = 1;
    pointcloud_msg.is_dense = true;
    pointcloud_msg.is_bigendian = false;

    if (generate_points) {
      std::vector<Eigen::Vector3f> points;
      std::vector<float> azimuths;

      if (use_default_pointcloud) {
        std::vector<Eigen::Vector3f> default_points = {{
          Eigen::Vector3f(10.0f, 0.0f, 1.0f),   // point 1
          Eigen::Vector3f(5.0f, -5.0f, 2.0f),   // point 2
          Eigen::Vector3f(0.0f, -10.0f, 3.0f),  // point 3
          Eigen::Vector3f(-5.0f, -5.0f, 4.0f),  // point 4
          Eigen::Vector3f(-10.0f, 0.0f, 5.0f),  // point 5
          Eigen::Vector3f(-5.0f, 5.0f, -5.0f),  // point 6
          Eigen::Vector3f(0.0f, 10.0f, -4.0f),  // point 7
          Eigen::Vector3f(5.0f, 5.0f, -3.0f),   // point 8
          Eigen::Vector3f(8.0f, 3.0f, -2.0f),   // point 9
          Eigen::Vector3f(9.0f, 1.0f, -1.0f)    // point 10
        }};

        std::vector<float> default_azimuths;
        for (const auto & point : default_points) {
          if (vendor == "velodyne") {
            float cartesian_deg =
              std::atan2(point.y(), point.x()) * 180 / autoware::universe_utils::pi;
            if (cartesian_deg < 0) cartesian_deg += 360;
            float velodyne_deg = 360 - cartesian_deg;
            if (velodyne_deg == 360) velodyne_deg = 0;
            default_azimuths.push_back(velodyne_deg * autoware::universe_utils::pi / 180);
          } else if (vendor == "hesai") {
            float cartesian_deg =
              std::atan2(point.y(), point.x()) * 180 / autoware::universe_utils::pi;
            if (cartesian_deg < 0) cartesian_deg += 360;
            float hesai_deg =
              90 - cartesian_deg < 0 ? 90 - cartesian_deg + 360 : 90 - cartesian_deg;
            if (hesai_deg == 360) hesai_deg = 0;
            default_azimuths.push_back(hesai_deg * autoware::universe_utils::pi / 180);
          } else {  // empty string
            default_azimuths.push_back(std::atan2(point.y(), point.x()));
          }
        }

        points = default_points;
        azimuths = default_azimuths;
      } else {
        points = defined_points;
        azimuths = defined_azimuths;
      }

      // Generate timestamps for the points
      std::vector<std::uint32_t> timestamps = generatePointTimestamps(stamp, points.size());

      sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
      modifier.setPointCloud2Fields(
        10, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
        sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16,
        "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "elevation", 1,
        sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
        "time_stamp", 1, sensor_msgs::msg::PointField::UINT32);

      modifier.resize(points.size());

      sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_azimuth(pointcloud_msg, "azimuth");
      sensor_msgs::PointCloud2Iterator<float> iter_distance(pointcloud_msg, "distance");
      sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(pointcloud_msg, "time_stamp");

      for (size_t i = 0; i < points.size(); ++i) {
        *iter_x = points[i].x();
        *iter_y = points[i].y();
        *iter_z = points[i].z();

        *iter_azimuth = azimuths[i];
        *iter_distance = points[i].norm();
        *iter_t = timestamps[i];
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_azimuth;
        ++iter_distance;
        ++iter_t;
      }
    } else {
      pointcloud_msg.width = 0;
      pointcloud_msg.row_step = 0;
    }

    return pointcloud_msg;
  }

  std::vector<std::uint32_t> generatePointTimestamps(
    rclcpp::Time pointcloud_timestamp, size_t number_of_points)
  {
    std::vector<std::uint32_t> timestamps;
    rclcpp::Time global_point_stamp = pointcloud_timestamp;
    for (size_t i = 0; i < number_of_points; ++i) {
      std::uint32_t relative_timestamp = (global_point_stamp - pointcloud_timestamp).nanoseconds();
      timestamps.push_back(relative_timestamp);
      global_point_stamp = addMilliseconds(global_point_stamp, points_interval_ms_);
    }

    return timestamps;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware::pointcloud_preprocessor::DistortionCorrector2D>
    distortion_corrector_2d_;
  std::shared_ptr<autoware::pointcloud_preprocessor::DistortionCorrector3D>
    distortion_corrector_3d_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  static constexpr float standard_tolerance_{1e-4};
  static constexpr float coarse_tolerance_{5e-3};
  static constexpr int number_of_twist_msgs_{6};
  static constexpr int number_of_imu_msgs_{6};
  static constexpr size_t number_of_points_{10};
  static constexpr int32_t timestamp_seconds_{10};
  static constexpr uint32_t timestamp_nanoseconds_{100000000};

  static constexpr double twist_linear_x_{10.0};
  static constexpr double twist_angular_z_{0.02};
  static constexpr double twist_linear_x_increment_{2.0};
  static constexpr double twist_angular_z_increment_{0.01};

  static constexpr double imu_angular_x_{0.01};
  static constexpr double imu_angular_y_{-0.02};
  static constexpr double imu_angular_z_{0.05};
  static constexpr double imu_angular_x_increment_{0.005};
  static constexpr double imu_angular_y_increment_{0.005};
  static constexpr double imu_angular_z_increment_{0.005};

  static constexpr int points_interval_ms_{10};
  static constexpr int twist_msgs_interval_ms_{24};
  static constexpr int imu_msgs_interval_ms_{27};

  static constexpr double epsilon = 1e-5;

  // for debugging or regenerating the ground truth point cloud
  bool debug_{false};
};

TEST_F(DistortionCorrectorTest, TestProcessTwistMessage)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  auto twist_msg = generateTwistMsg(twist_linear_x_, twist_angular_z_, timestamp);
  distortion_corrector_2d_->processTwistMessage(twist_msg);

  ASSERT_FALSE(distortion_corrector_2d_->get_twist_queue().empty());
  EXPECT_EQ(distortion_corrector_2d_->get_twist_queue().front().twist.linear.x, twist_linear_x_);
  EXPECT_EQ(distortion_corrector_2d_->get_twist_queue().front().twist.angular.z, twist_angular_z_);
}

TEST_F(DistortionCorrectorTest, TestProcessIMUMessage)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  auto imu_msg = generateImuMsg(imu_angular_x_, imu_angular_y_, imu_angular_z_, timestamp);
  distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);

  ASSERT_FALSE(distortion_corrector_2d_->get_angular_velocity_queue().empty());
  EXPECT_NEAR(
    distortion_corrector_2d_->get_angular_velocity_queue().front().vector.z, -0.03159,
    standard_tolerance_);
}

TEST_F(DistortionCorrectorTest, TestIsInputValid)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);

  // input normal pointcloud without twist
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});
  bool result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_FALSE(result);

  // input normal pointcloud with valid twist
  auto twist_msg = generateTwistMsg(twist_linear_x_, twist_angular_z_, timestamp);
  distortion_corrector_2d_->processTwistMessage(twist_msg);

  pointcloud = generatePointCloudMsg(true, false, "", timestamp, true, {}, {});
  result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_TRUE(result);

  // input empty pointcloud
  pointcloud = generatePointCloudMsg(false, false, "", timestamp, true, {}, {});
  result = distortion_corrector_2d_->isInputValid(pointcloud);
  EXPECT_FALSE(result);
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithBaseLink)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  EXPECT_TRUE(distortion_corrector_2d_->pointcloudTransformExists());
  EXPECT_FALSE(distortion_corrector_2d_->pointcloudTransformNeeded());
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithLidarFrame)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  EXPECT_TRUE(distortion_corrector_2d_->pointcloudTransformExists());
  EXPECT_TRUE(distortion_corrector_2d_->pointcloudTransformNeeded());
}

TEST_F(DistortionCorrectorTest, TestSetPointCloudTransformWithMissingFrame)
{
  distortion_corrector_2d_->setPointCloudTransform("base_link", "missing_lidar_frame");
  EXPECT_FALSE(distortion_corrector_2d_->pointcloudTransformExists());
  EXPECT_FALSE(distortion_corrector_2d_->pointcloudTransformNeeded());
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithEmptyTwist)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate the point cloud message
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Process empty twist queue
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->undistortPointCloud(false, false, pointcloud);

  // Verify the point cloud is not changed
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 1.0f), Eigen::Vector3f(5.0f, -5.0f, 2.0f),
     Eigen::Vector3f(0.0f, -10.0f, 3.0f), Eigen::Vector3f(-5.0f, -5.0f, 4.0f),
     Eigen::Vector3f(-10.0f, 0.0f, 5.0f), Eigen::Vector3f(-5.0f, 5.0f, -5.0f),
     Eigen::Vector3f(0.0f, 10.0f, -4.0f), Eigen::Vector3f(5.0f, 5.0f, -3.0f),
     Eigen::Vector3f(8.0f, 3.0f, -2.0f), Eigen::Vector3f(9.0f, 1.0f, -1.0f)}};
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithEmptyPointCloud)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  // Generate an empty point cloud message
  sensor_msgs::msg::PointCloud2 empty_pointcloud =
    generatePointCloudMsg(false, false, "", timestamp, true, {}, {});

  // Process empty point cloud
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->undistortPointCloud(true, false, empty_pointcloud);

  // Verify the point cloud is still empty
  EXPECT_EQ(empty_pointcloud.width, 0);
  EXPECT_EQ(empty_pointcloud.row_step, 0);
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithoutImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Test using only twist
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(false, false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 1.0f), Eigen::Vector3f(5.12144f, -4.99853f, 2.0f),
     Eigen::Vector3f(0.266711f, -9.99986f, 3.0f), Eigen::Vector3f(-4.59472f, -5.00498f, 4.0f),
     Eigen::Vector3f(-9.45999f, -0.0148422f, 5.0f), Eigen::Vector3f(-4.31006f, 4.99074f, -5.0f),
     Eigen::Vector3f(0.835072f, 10.0012f, -4.0f), Eigen::Vector3f(6.02463f, 5.0171f, -3.0f),
     Eigen::Vector3f(9.20872f, 3.03234f, -2.0f), Eigen::Vector3f(10.3956f, 1.04204f, -1.0f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(true, false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 1.0f), Eigen::Vector3f(5.11856f, -5.00147f, 2.0f),
     Eigen::Vector3f(0.254248f, -10.0001f, 3.0f), Eigen::Vector3f(-4.60431f, -4.99592f, 4.0f),
     Eigen::Vector3f(-9.45999f, 0.0111079f, 5.0f), Eigen::Vector3f(-4.2928f, 5.00656f, -5.0f),
     Eigen::Vector3f(0.877257f, 9.99908f, -4.0f), Eigen::Vector3f(6.05006f, 4.98867f, -3.0f),
     Eigen::Vector3f(9.22659f, 2.98069f, -2.0f), Eigen::Vector3f(10.4025f, 0.975451f, -1.0f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud2dWithImuInLidarFrame)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, true, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  distortion_corrector_2d_->undistortPointCloud(true, false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, -2.63462e-08f, 1.0f), Eigen::Vector3f(5.05137f, -4.93869f, 2.09267f),
     Eigen::Vector3f(0.112092f, -9.86876f, 3.19715f),
     Eigen::Vector3f(-4.83021f, -4.80022f, 4.30059f),
     Eigen::Vector3f(-9.7743f, 0.266927f, 5.40228f),
     Eigen::Vector3f(-4.69786f, 5.35289f, -4.47032f),
     Eigen::Vector3f(0.365836f, 10.4368f, -3.33969f),
     Eigen::Vector3f(5.44511f, 5.53184f, -2.19585f), Eigen::Vector3f(8.52226f, 3.62536f, -1.0538f),
     Eigen::Vector3f(9.59921f, 1.71784f, 0.0862978f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithoutImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_3d_->processTwistMessage(twist_msg);
  }

  // Test using only twist
  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(false, false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 1.0f), Eigen::Vector3f(5.1215f, -4.99848f, 2.0f),
     Eigen::Vector3f(0.267f, -9.99991f, 3.0f), Eigen::Vector3f(-4.5945f, -5.00528f, 4.0f),
     Eigen::Vector3f(-9.45999f, -0.0146016f, 5.0f), Eigen::Vector3f(-4.30999f, 4.9907f, -5.0f),
     Eigen::Vector3f(0.834999f, 10.0011f, -4.0f), Eigen::Vector3f(6.02447f, 5.01714f, -3.0f),
     Eigen::Vector3f(9.20884f, 3.03192f, -2.0f), Eigen::Vector3f(10.3956f, 1.04182f, -1.0f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithImuInBaseLink)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_3d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_3d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(true, false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, number_of_points_> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 0.0f, 1.0f), Eigen::Vector3f(5.11902f, -5.00239f, 1.9965f),
     Eigen::Vector3f(0.255945f, -10.0027f, 2.99104f),
     Eigen::Vector3f(-4.60055f, -5.00116f, 3.99787f),
     Eigen::Vector3f(-9.45347f, 0.00231253f, 5.01268f),
     Eigen::Vector3f(-4.30145f, 5.01808f, -4.98054f),
     Eigen::Vector3f(0.868468f, 10.0103f, -3.97336f),
     Eigen::Vector3f(6.04213f, 4.99888f, -2.99811f), Eigen::Vector3f(9.22048f, 2.98838f, -2.01552f),
     Eigen::Vector3f(10.3988f, 0.980287f, -1.03088f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloud3dWithImuInLidarFrame)
{
  // Generate the point cloud message
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, true, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_3d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_3d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "lidar_top");
  distortion_corrector_3d_->undistortPointCloud(true, false, pointcloud);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(pointcloud, "z");

  // Expected undistorted point cloud values
  std::array<Eigen::Vector3f, 10> expected_pointcloud = {
    {Eigen::Vector3f(10.0f, 4.76837e-07f, 1.0f), Eigen::Vector3f(5.05124f, -4.93713f, 2.09134f),
     Eigen::Vector3f(0.114215f, -9.86881f, 3.19198f),
     Eigen::Vector3f(-4.82904f, -4.80461f, 4.30305f),
     Eigen::Vector3f(-9.77529f, 0.255465f, 5.41964f), Eigen::Vector3f(-4.71431f, 5.36722f, -4.441f),
     Eigen::Vector3f(0.342873f, 10.4561f, -3.29519f), Eigen::Vector3f(5.426f, 5.55628f, -2.16926f),
     Eigen::Vector3f(8.50523f, 3.65298f, -1.03512f),
     Eigen::Vector3f(9.58544f, 1.74465f, 0.0959219f)}};

  // Verify each point in the undistorted point cloud
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++i) {
    oss << "Point " << i << ": (" << *iter_x << ", " << *iter_y << ", " << *iter_z << ")\n";
    EXPECT_NEAR(*iter_x, expected_pointcloud[i].x(), standard_tolerance_);
    EXPECT_NEAR(*iter_y, expected_pointcloud[i].y(), standard_tolerance_);
    EXPECT_NEAR(*iter_z, expected_pointcloud[i].z(), standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithPureLinearMotion)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 test2d_pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});
  sensor_msgs::msg::PointCloud2 test3d_pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process a single twist message with constant linear velocity
  auto twist_msg = generateTwistMsg(1.0, 0.0, timestamp);

  distortion_corrector_2d_->processTwistMessage(twist_msg);
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(false, false, test2d_pointcloud);

  distortion_corrector_3d_->processTwistMessage(twist_msg);
  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(false, false, test3d_pointcloud);

  // Generate expected point cloud for testing
  sensor_msgs::msg::PointCloud2 expected_pointcloud_msg =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Calculate expected point cloud values based on constant linear motion
  double velocity = 1.0;  // 1 m/s linear velocity
  sensor_msgs::PointCloud2Iterator<float> iter_x(expected_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(expected_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(expected_pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(expected_pointcloud_msg, "time_stamp");

  std::vector<Eigen::Vector3f> expected_points;
  for (; iter_t != iter_t.end(); ++iter_t, ++iter_x, ++iter_y, ++iter_z) {
    double time_offset = static_cast<double>(*iter_t) / 1e9;
    expected_points.emplace_back(
      *iter_x + static_cast<float>(velocity * time_offset), *iter_y, *iter_z);
  }

  // Verify each point in the undistorted point cloud
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_x(test2d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_y(test2d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_z(test2d_pointcloud, "z");
  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; test2d_iter_x != test2d_iter_x.end();
       ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z, ++i) {
    oss << "Point " << i << ": (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")\n";
    EXPECT_FLOAT_EQ(*test2d_iter_x, expected_points[i].x());
    EXPECT_FLOAT_EQ(*test2d_iter_y, expected_points[i].y());
    EXPECT_FLOAT_EQ(*test2d_iter_z, expected_points[i].z());
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }

  // Verify each point in the undistorted 2d point cloud with undistorted 3d point cloud
  test2d_iter_x = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "x");
  test2d_iter_y = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "y");
  test2d_iter_z = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "z");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_x(test3d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_y(test3d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_z(test3d_pointcloud, "z");

  i = 0;
  oss.str("");
  oss.clear();

  oss << "Expected pointcloud:\n";
  for (; test2d_iter_x != test2d_iter_x.end(); ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z,
                                               ++test3d_iter_x, ++test3d_iter_y, ++test3d_iter_z,
                                               ++i) {
    oss << "Point " << i << " - 2D: (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")"
        << " vs 3D: (" << *test3d_iter_x << ", " << *test3d_iter_y << ", " << *test3d_iter_z
        << ")\n";
    EXPECT_FLOAT_EQ(*test2d_iter_x, *test3d_iter_x);
    EXPECT_FLOAT_EQ(*test2d_iter_y, *test3d_iter_y);
    EXPECT_FLOAT_EQ(*test2d_iter_z, *test3d_iter_z);
  }
  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudWithPureRotationalMotion)
{
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 test2d_pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});
  sensor_msgs::msg::PointCloud2 test3d_pointcloud =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process a single twist message with constant angular velocity
  auto twist_msg = generateTwistMsg(0.0, 0.1, timestamp);

  distortion_corrector_2d_->processTwistMessage(twist_msg);
  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_2d_->undistortPointCloud(false, false, test2d_pointcloud);

  distortion_corrector_3d_->processTwistMessage(twist_msg);
  distortion_corrector_3d_->initialize();
  distortion_corrector_3d_->setPointCloudTransform("base_link", "base_link");
  distortion_corrector_3d_->undistortPointCloud(false, false, test3d_pointcloud);

  // Generate expected point cloud for testing
  sensor_msgs::msg::PointCloud2 expected_pointcloud_msg =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Calculate expected point cloud values based on constant rotational motion
  double angular_velocity = 0.1;  // 0.1 rad/s rotational velocity
  sensor_msgs::PointCloud2Iterator<float> iter_x(expected_pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(expected_pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(expected_pointcloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> iter_t(expected_pointcloud_msg, "time_stamp");

  std::vector<Eigen::Vector3f> expected_pointcloud;
  for (; iter_t != iter_t.end(); ++iter_t, ++iter_x, ++iter_y, ++iter_z) {
    double time_offset = static_cast<double>(*iter_t) / 1e9;
    float angle = angular_velocity * time_offset;

    // Set the quaternion for the current angle
    tf2::Quaternion quaternion;
    quaternion.setValue(
      0, 0, autoware::universe_utils::sin(angle * 0.5f),
      autoware::universe_utils::cos(angle * 0.5f));

    tf2::Vector3 point(*iter_x, *iter_y, *iter_z);
    tf2::Vector3 rotated_point = tf2::quatRotate(quaternion, point);
    expected_pointcloud.emplace_back(
      static_cast<float>(rotated_point.x()), static_cast<float>(rotated_point.y()),
      static_cast<float>(rotated_point.z()));
  }

  // Verify each point in the undistorted 2D point cloud
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_x(test2d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_y(test2d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test2d_iter_z(test2d_pointcloud, "z");

  size_t i = 0;
  std::ostringstream oss;
  oss << "Expected pointcloud:\n";

  for (; test2d_iter_x != test2d_iter_x.end();
       ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z, ++i) {
    oss << "Point " << i << ": (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")\n";
    EXPECT_FLOAT_EQ(*test2d_iter_x, expected_pointcloud[i].x());
    EXPECT_FLOAT_EQ(*test2d_iter_y, expected_pointcloud[i].y());
    EXPECT_FLOAT_EQ(*test2d_iter_z, expected_pointcloud[i].z());
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }

  // Verify each point in the undistorted 2D point cloud with undistorted 3D point cloud
  test2d_iter_x = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "x");
  test2d_iter_y = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "y");
  test2d_iter_z = sensor_msgs::PointCloud2Iterator<float>(test2d_pointcloud, "z");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_x(test3d_pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_y(test3d_pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> test3d_iter_z(test3d_pointcloud, "z");

  i = 0;
  oss.str("");
  oss.clear();

  oss << "Expected pointcloud:\n";
  for (; test2d_iter_x != test2d_iter_x.end(); ++test2d_iter_x, ++test2d_iter_y, ++test2d_iter_z,
                                               ++test3d_iter_x, ++test3d_iter_y, ++test3d_iter_z,
                                               ++i) {
    oss << "Point " << i << " - 2D: (" << *test2d_iter_x << ", " << *test2d_iter_y << ", "
        << *test2d_iter_z << ")"
        << " vs 3D: (" << *test3d_iter_x << ", " << *test3d_iter_y << ", " << *test3d_iter_z
        << ")\n";
    EXPECT_NEAR(*test2d_iter_x, *test3d_iter_x, coarse_tolerance_);
    EXPECT_NEAR(*test2d_iter_y, *test3d_iter_y, coarse_tolerance_);
    EXPECT_NEAR(*test2d_iter_z, *test3d_iter_z, coarse_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudNotUpdateAzimuthAndDistance)
{
  // Test the case when the cloud will not update the azimuth and distance values
  // 1. when pointcloud is in the base_link

  // Generate the point cloud message in base_link
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud_base_link =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "base_link");
  auto can_update_azimuth_and_distance =
    distortion_corrector_2d_->azimuthConversionExists(pointcloud_base_link);

  distortion_corrector_2d_->undistortPointCloud(
    true, can_update_azimuth_and_distance, pointcloud_base_link);

  sensor_msgs::msg::PointCloud2 original_pointcloud_base_link =
    generatePointCloudMsg(true, false, "", timestamp, true, {}, {});

  sensor_msgs::PointCloud2ConstIterator<float> test_iter_azimuth_base_link(
    pointcloud_base_link, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> test_iter_distance_base_link(
    pointcloud_base_link, "distance");

  sensor_msgs::PointCloud2ConstIterator<float> original_iter_azimuth_base_link(
    original_pointcloud_base_link, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> original_iter_distance_base_link(
    original_pointcloud_base_link, "distance");

  size_t i = 0;
  std::ostringstream oss;

  oss << "Expected pointcloud:\n";
  for (; test_iter_azimuth_base_link != test_iter_azimuth_base_link.end();
       ++test_iter_azimuth_base_link, ++test_iter_distance_base_link,
       ++original_iter_azimuth_base_link, ++original_iter_distance_base_link, ++i) {
    oss << "Point " << i << " - Output azimuth and distance: (" << *test_iter_azimuth_base_link
        << ", " << *test_iter_distance_base_link << ")"
        << " vs Original azimuth and distance: (" << *original_iter_azimuth_base_link << ", "
        << *original_iter_distance_base_link << ")\n";

    EXPECT_FLOAT_EQ(*test_iter_azimuth_base_link, *original_iter_azimuth_base_link);
    EXPECT_FLOAT_EQ(*test_iter_distance_base_link, *original_iter_distance_base_link);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }

  // Test the case when the cloud will not update the azimuth and distance values
  // 2. when "can_update_azimuth_and_distance" is false

  // Generate the point cloud message in sensor frame
  sensor_msgs::msg::PointCloud2 pointcloud_lidar_top =
    generatePointCloudMsg(true, true, "", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  // set the empty coordinate system
  can_update_azimuth_and_distance = false;
  distortion_corrector_2d_->undistortPointCloud(
    true, can_update_azimuth_and_distance, pointcloud_lidar_top);

  sensor_msgs::msg::PointCloud2 original_pointcloud_lidar_top =
    generatePointCloudMsg(true, true, "", timestamp, true, {}, {});

  sensor_msgs::PointCloud2ConstIterator<float> test_iter_azimuth_lidar_top(
    pointcloud_lidar_top, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> test_iter_distance_lidar_top(
    pointcloud_lidar_top, "distance");

  sensor_msgs::PointCloud2ConstIterator<float> original_iter_azimuth_lidar_top(
    original_pointcloud_lidar_top, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> original_iter_distance_lidar_top(
    original_pointcloud_lidar_top, "distance");

  i = 0;
  oss.str("");
  oss.clear();

  oss << "Expected pointcloud:\n";
  for (; test_iter_azimuth_lidar_top != test_iter_azimuth_lidar_top.end();
       ++test_iter_azimuth_lidar_top, ++test_iter_distance_lidar_top,
       ++original_iter_azimuth_lidar_top, ++original_iter_distance_lidar_top, ++i) {
    oss << "Point " << i << " - Output azimuth and distance: (" << *test_iter_azimuth_lidar_top
        << ", " << *test_iter_distance_lidar_top << ")"
        << " vs Original azimuth and distance: (" << *original_iter_azimuth_lidar_top << ", "
        << *original_iter_distance_lidar_top << ")\n";

    EXPECT_FLOAT_EQ(*test_iter_azimuth_lidar_top, *original_iter_azimuth_lidar_top);
    EXPECT_FLOAT_EQ(*test_iter_distance_lidar_top, *original_iter_distance_lidar_top);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudUpdateAzimuthAndDistanceInVelodyne)
{
  // Generate the point cloud message in sensor frame
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, true, "velodyne", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  // set the azimuth coordinate system
  auto can_update_azimuth_and_distance =
    distortion_corrector_2d_->azimuthConversionExists(pointcloud);
  distortion_corrector_2d_->undistortPointCloud(true, can_update_azimuth_and_distance, pointcloud);

  sensor_msgs::msg::PointCloud2 original_pointcloud =
    generatePointCloudMsg(true, true, "velodyne", timestamp, true, {}, {});

  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(pointcloud, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(pointcloud, "distance");

  // Expected undistorted azimuth and distance values
  std::array<std::array<float, 2>, 10> expected_distance_azimuth = {{
    {0.0f, 10.0499f},      // points: (10.0f, -2.63462e-08f, 1.0f)
    {0.77413f, 7.36792f},  // points: (5.05137f, -4.93869f, 2.09267f)
    {1.55944f, 10.3743f},  // points: (0.112092f, -9.86876f, 3.19715f)
    {2.35942f, 8.05408f},  // points: (-4.83021f, -4.80022f, 4.30059f)
    {3.16889f, 11.1711f},  // points: (-9.7743f, 0.266927f, 5.40228f)
    {3.99196f, 8.40875f},  // points: (-4.69786f, 5.35289f, -4.47032f)
    {4.74742f, 10.9642f},  // points: (0.365836f, 10.4368f, -3.33969f)
    {5.48985f, 8.06673f},  // points: (5.44511f, 5.53184f, -2.19585f)
    {5.8809f, 9.32108f},   // points: (8.52226f, 3.62536f, -1.0538f)
    {6.10611f, 9.75209f}   // points: (9.59921f, 1.71784f, 0.0862978f)
  }};

  size_t i = 0;
  std::ostringstream oss;

  oss << "Expected pointcloud:\n";
  for (; iter_azimuth != iter_azimuth.end(); ++iter_azimuth, ++iter_distance, ++i) {
    oss << "Point " << i << " - Output azimuth and distance: (" << *iter_azimuth << ", "
        << *iter_distance << ")"
        << " vs Expected azimuth and distance: (" << expected_distance_azimuth[i][0] << ", "
        << expected_distance_azimuth[i][1] << ")\n";

    EXPECT_NEAR(*iter_azimuth, expected_distance_azimuth[i][0], standard_tolerance_);
    EXPECT_NEAR(*iter_distance, expected_distance_azimuth[i][1], standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestUndistortPointCloudUpdateAzimuthAndDistanceInHesai)
{
  // Generate the point cloud message in sensor frame
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, true, "hesai", timestamp, true, {}, {});

  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }

  // Generate and process multiple IMU messages
  auto imu_msgs = generateImuMsgs(timestamp);
  for (const auto & imu_msg : imu_msgs) {
    distortion_corrector_2d_->processIMUMessage("base_link", imu_msg);
  }

  distortion_corrector_2d_->initialize();
  distortion_corrector_2d_->setPointCloudTransform("base_link", "lidar_top");
  // set the azimuth coordinate system
  auto can_update_azimuth_and_distance =
    distortion_corrector_2d_->azimuthConversionExists(pointcloud);
  distortion_corrector_2d_->undistortPointCloud(true, can_update_azimuth_and_distance, pointcloud);

  sensor_msgs::msg::PointCloud2 original_pointcloud =
    generatePointCloudMsg(true, true, "hesai", timestamp, true, {}, {});

  sensor_msgs::PointCloud2ConstIterator<float> iter_azimuth(pointcloud, "azimuth");
  sensor_msgs::PointCloud2ConstIterator<float> iter_distance(pointcloud, "distance");

  // Expected undistorted azimuth and distance values
  std::array<std::array<float, 2>, 10> expected_distance_azimuth = {
    {{1.5708f, 10.0499f},
     {2.34493f, 7.36792f},
     {3.13024f, 10.3743f},
     {3.93021f, 8.05408f},
     {4.73969f, 11.1711f},
     {5.56275f, 8.40875f},
     {0.0350311f, 10.9642f},
     {0.777465f, 8.06673f},
     {1.16851f, 9.32108f},
     {1.39372f, 9.75209f}}};

  size_t i = 0;
  std::ostringstream oss;

  oss << "Expected pointcloud:\n";
  for (; iter_azimuth != iter_azimuth.end(); ++iter_azimuth, ++iter_distance, ++i) {
    oss << "Point " << i << " - Output azimuth and distance: (" << *iter_azimuth << ", "
        << *iter_distance << ")"
        << " vs Expected azimuth and distance: (" << expected_distance_azimuth[i][0] << ", "
        << expected_distance_azimuth[i][1] << ")\n";

    EXPECT_NEAR(*iter_azimuth, expected_distance_azimuth[i][0], standard_tolerance_);
    EXPECT_NEAR(*iter_distance, expected_distance_azimuth[i][1], standard_tolerance_);
  }

  if (debug_) {
    RCLCPP_INFO(node_->get_logger(), "%s", oss.str().c_str());
  }
}

TEST_F(DistortionCorrectorTest, TestAzimuthConversionExistsEmptyPointcloud)
{
  // test empty pointcloud
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  sensor_msgs::msg::PointCloud2 empty_pointcloud =
    generatePointCloudMsg(false, false, "", timestamp, true, {}, {});
  EXPECT_FALSE(distortion_corrector_2d_->azimuthConversionExists(empty_pointcloud));
}

TEST_F(DistortionCorrectorTest, TestAzimuthConversionExistsVelodynePointcloud)
{
  // test velodyne pointcloud (x-axis: 0 degree, y-axis: 270 degree)
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  std::vector<Eigen::Vector3f> velodyne_points = {
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(1.0f, -1.0f, 1.0f),
    Eigen::Vector3f(0.0f, -2.0f, 1.0f),
  };
  std::vector<float> velodyne_azimuths = {
    0.0f, autoware::universe_utils::pi / 4, autoware::universe_utils::pi / 2};
  sensor_msgs::msg::PointCloud2 velodyne_pointcloud =
    generatePointCloudMsg(true, true, "", timestamp, false, velodyne_points, velodyne_azimuths);
  EXPECT_TRUE(distortion_corrector_2d_->azimuthConversionExists(velodyne_pointcloud));

  auto [a, b] = distortion_corrector_2d_->getConversion();
  EXPECT_EQ(b, -1);
  EXPECT_NEAR(a, autoware::universe_utils::pi * 2, standard_tolerance_);
}

TEST_F(DistortionCorrectorTest, TestAzimuthConversionExistsHesaiPointcloud)
{
  // test hesai pointcloud (x-axis: 90 degree, y-axis: 0 degree)
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  std::vector<Eigen::Vector3f> hesai_points = {
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(1.0f, -1.0f, 1.0f),
    Eigen::Vector3f(0.0f, -2.0f, 1.0f),
  };
  std::vector<float> hesai_azimuths = {
    autoware::universe_utils::pi / 2, autoware::universe_utils::pi * 3 / 4,
    autoware::universe_utils::pi};
  sensor_msgs::msg::PointCloud2 hesai_pointcloud =
    generatePointCloudMsg(true, true, "", timestamp, false, hesai_points, hesai_azimuths);
  EXPECT_TRUE(distortion_corrector_2d_->azimuthConversionExists(hesai_pointcloud));

  auto [a, b] = distortion_corrector_2d_->getConversion();
  EXPECT_EQ(b, -1);
  EXPECT_NEAR(a, autoware::universe_utils::pi / 2, standard_tolerance_);
}

TEST_F(DistortionCorrectorTest, TestAzimuthConversionExistsCartesianPointcloud)
{
  // test pointcloud that use cartesian coordinate for azimuth (x-axis: 0 degree, y-axis: 90 degree)
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  std::vector<Eigen::Vector3f> cartesian_points = {
    Eigen::Vector3f(0.0f, 0.0f, 0.0f),
    Eigen::Vector3f(1.0f, 1.0f, 1.0f),
    Eigen::Vector3f(0.0f, 2.0f, 1.0f),
  };
  std::vector<float> cartesian_azimuths = {
    0, autoware::universe_utils::pi / 4, autoware::universe_utils::pi / 2};
  sensor_msgs::msg::PointCloud2 cartesian_pointcloud =
    generatePointCloudMsg(true, true, "", timestamp, false, cartesian_points, cartesian_azimuths);
  EXPECT_TRUE(distortion_corrector_2d_->azimuthConversionExists(cartesian_pointcloud));

  auto [a, b] = distortion_corrector_2d_->getConversion();
  EXPECT_EQ(b, 1);
  EXPECT_NEAR(a, 0, standard_tolerance_);
}

TEST_F(DistortionCorrectorTest, TestAzimuthConversionExistsRandomPointcloud1)
{
  // test pointcloud that use coordinate (x-axis: 270 degree, y-axis: 0 degree)
  rclcpp::Time timestamp(timestamp_seconds_, timestamp_nanoseconds_, RCL_ROS_TIME);
  // Generate and process multiple twist messages
  auto twist_msgs = generateTwistMsgs(timestamp);
  for (const auto & twist_msg : twist_msgs) {
    distortion_corrector_2d_->processTwistMessage(twist_msg);
  }
  std::vector<Eigen::Vector3f> points = {
    Eigen::Vector3f(0.0f, 1.0f, 0.0f),
    Eigen::Vector3f(2.0f, 0.0f, 1.0f),
    Eigen::Vector3f(1.0f, 1.0f, 1.0f),
  };
  std::vector<float> azimuths = {
    0, autoware::universe_utils::pi * 3 / 2, autoware::universe_utils::pi * 7 / 4};
  sensor_msgs::msg::PointCloud2 pointcloud =
    generatePointCloudMsg(true, true, "", timestamp, false, points, azimuths);
  EXPECT_TRUE(distortion_corrector_2d_->azimuthConversionExists(pointcloud));

  auto [a, b] = distortion_corrector_2d_->getConversion();
  EXPECT_EQ(b, 1);
  EXPECT_NEAR(a, autoware::universe_utils::pi * 3 / 2, standard_tolerance_);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
