// Copyright 2023 Tier IV, Inc.
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

#include "probabilistic_occupancy_grid_map/utils/utils.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>

#include <string>

namespace
{
/**
 * @brief Hash function for voxel keys.
 * Utilizes prime numbers to calculate a unique hash for each voxel key.
 */
struct VoxelKeyHash
{
  std::size_t operator()(const std::array<int, 3> & k) const
  {
    // Primes based on the following paper: 'Investigating the Use of Primes in Hashing for
    // Volumetric Data'.
    return (k[0] * 73856093 ^ k[1] * 19349663 ^ k[2] * 83492791);
  }
};

/**
 * @brief Equality function for voxel keys.
 * Checks if two voxel keys are equal.
 */
struct VoxelKeyEqual
{
  bool operator()(const std::array<int, 3> & a, const std::array<int, 3> & b) const
  {
    return a == b;
  }
};
}  // namespace

namespace utils
{

// used in laserscan based occupancy grid map
bool transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, sensor_msgs::msg::PointCloud2 & output)
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  // lookup transform
  try {
    tf_stamped = tf2.lookupTransform(
      target_frame, input.header.frame_id, input.header.stamp, rclcpp::Duration::from_seconds(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      rclcpp::get_logger("probabilistic_occupancy_grid_map"), "Failed to lookup transform: %s",
      ex.what());
    return false;
  }
  // transform pointcloud
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(tf_stamped.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = target_frame;
  return true;
}

// used in pointcloud based occupancy grid map
void transformPointcloud(
  const sensor_msgs::msg::PointCloud2 & input, const geometry_msgs::msg::Pose & pose,
  sensor_msgs::msg::PointCloud2 & output)
{
  const auto transform = tier4_autoware_utils::pose2transform(pose);
  Eigen::Matrix4f tf_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  pcl_ros::transformPointCloud(tf_matrix, input, output);
  output.header.stamp = input.header.stamp;
  output.header.frame_id = "";
}

bool cropPointcloudByHeight(
  const sensor_msgs::msg::PointCloud2 & input, const tf2_ros::Buffer & tf2,
  const std::string & target_frame, const float min_height, const float max_height,
  sensor_msgs::msg::PointCloud2 & output)
{
  rclcpp::Clock clock{RCL_ROS_TIME};
  // Transformed pointcloud on target frame
  sensor_msgs::msg::PointCloud2 trans_input_tmp;
  const bool is_target_frame = (input.header.frame_id == target_frame);
  if (!is_target_frame) {
    if (!transformPointcloud(input, tf2, target_frame, trans_input_tmp)) return false;
  }
  const sensor_msgs::msg::PointCloud2 & trans_input = is_target_frame ? input : trans_input_tmp;

  // Apply height filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(trans_input, "x"),
       iter_y(trans_input, "y"), iter_z(trans_input, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (min_height < *iter_z && *iter_z < max_height) {
      pcl_output->push_back(pcl::PointXYZ(*iter_x, *iter_y, *iter_z));
    }
  }

  // Convert to ros msg
  pcl::toROSMsg(*pcl_output, output);
  output.header = trans_input.header;
  return true;
}

geometry_msgs::msg::Pose getPose(
  const std_msgs::msg::Header & source_header, const tf2_ros::Buffer & tf2,
  const std::string & target_frame)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped = tf2.lookupTransform(
    target_frame, source_header.frame_id, source_header.stamp, rclcpp::Duration::from_seconds(0.5));
  pose = tier4_autoware_utils::transform2pose(tf_stamped.transform);
  return pose;
}

geometry_msgs::msg::Pose getPose(
  const builtin_interfaces::msg::Time & stamp, const tf2_ros::Buffer & tf2,
  const std::string & source_frame, const std::string & target_frame)
{
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped =
    tf2.lookupTransform(target_frame, source_frame, stamp, rclcpp::Duration::from_seconds(0.5));
  pose = tier4_autoware_utils::transform2pose(tf_stamped.transform);
  return pose;
}

/**
 * @brief Get the Inverse Pose object
 *
 * @param input
 * @return geometry_msgs::msg::Pose inverted pose
 */
geometry_msgs::msg::Pose getInversePose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  tf2::Quaternion orientation(
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Transform tf(orientation, position);
  const auto inv_tf = tf.inverse();
  geometry_msgs::msg::Pose inv_pose;
  inv_pose.position.x = inv_tf.getOrigin().x();
  inv_pose.position.y = inv_tf.getOrigin().y();
  inv_pose.position.z = inv_tf.getOrigin().z();
  inv_pose.orientation.x = inv_tf.getRotation().x();
  inv_pose.orientation.y = inv_tf.getRotation().y();
  inv_pose.orientation.z = inv_tf.getRotation().z();
  inv_pose.orientation.w = inv_tf.getRotation().w();
  return inv_pose;
}

/**
 * @brief extract Common Pointcloud between obstacle pc and raw pc
 * @param obstacle_pc
 * @param raw_pc
 * @param output_obstacle_pc
 */
bool extractCommonPointCloud(
  const sensor_msgs::msg::PointCloud2 & obstacle_pc, const sensor_msgs::msg::PointCloud2 & raw_pc,
  sensor_msgs::msg::PointCloud2 & output_obstacle_pc)
{
  // Convert to vector of 3d points
  std::vector<MyPoint3d> v_obstacle_pc, v_raw_pc, v_output_obstacle_pc;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(obstacle_pc, "x"),
       iter_y(obstacle_pc, "y"), iter_z(obstacle_pc, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    v_obstacle_pc.push_back(MyPoint3d(*iter_x, *iter_y, *iter_z));
  }
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(raw_pc, "x"), iter_y(raw_pc, "y"),
       iter_z(raw_pc, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    v_raw_pc.push_back(MyPoint3d(*iter_x, *iter_y, *iter_z));
  }

  // sort pointclouds for searching cross points: O(nlogn)
  std::sort(v_obstacle_pc.begin(), v_obstacle_pc.end(), [](auto a, auto b) { return a < b; });
  std::sort(v_raw_pc.begin(), v_raw_pc.end(), [](auto a, auto b) { return a < b; });

  // calc intersection points of two pointclouds: O(n)
  std::set_intersection(
    v_obstacle_pc.begin(), v_obstacle_pc.end(), v_raw_pc.begin(), v_raw_pc.end(),
    std::back_inserter(v_output_obstacle_pc));
  if (v_output_obstacle_pc.size() == 0) {
    return false;
  }

  // Convert to ros msg
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto p : v_output_obstacle_pc) {
    pcl_output->push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }
  pcl::toROSMsg(*pcl_output, output_obstacle_pc);
  output_obstacle_pc.header = obstacle_pc.header;

  return true;
}

/**
 * @brief extract Common Pointcloud between obstacle pc and raw pc
 * @param obstacle_pc
 * @param raw_pc
 * @param voxel_size
 * @param output_obstacle_pc
 */
bool extractApproximateCommonPointCloud(
  const sensor_msgs::msg::PointCloud2 & obstacle_pc, const sensor_msgs::msg::PointCloud2 & raw_pc,
  const float voxel_size, sensor_msgs::msg::PointCloud2 & output_obstacle_pc)
{
  using VoxelKey = std::array<int, 3>;
  std::unordered_map<VoxelKey, size_t, VoxelKeyHash, VoxelKeyEqual> obstacle_voxel_map;
  std::unordered_map<VoxelKey, size_t, VoxelKeyHash, VoxelKeyEqual> raw_voxel_map;

  constexpr float large_num_offset = 100000.0;
  const float & voxel_size_x = voxel_size;
  const float & voxel_size_y = voxel_size;
  const float & voxel_size_z = voxel_size;
  const float inverse_voxel_size_x = 1.0 / voxel_size_x;
  const float inverse_voxel_size_y = 1.0 / voxel_size_y;
  const float inverse_voxel_size_z = 1.0 / voxel_size_z;

  {
    const int x_offset = raw_pc.fields[pcl::getFieldIndex(raw_pc, "x")].offset;
    const int y_offset = raw_pc.fields[pcl::getFieldIndex(raw_pc, "y")].offset;
    const int z_offset = raw_pc.fields[pcl::getFieldIndex(raw_pc, "z")].offset;

    // Process each point in the point cloud
    for (size_t global_offset = 0; global_offset + raw_pc.point_step <= raw_pc.data.size();
         global_offset += raw_pc.point_step) {
      const float & x = *reinterpret_cast<const float *>(&raw_pc.data[global_offset + x_offset]);
      const float & y = *reinterpret_cast<const float *>(&raw_pc.data[global_offset + y_offset]);
      const float & z = *reinterpret_cast<const float *>(&raw_pc.data[global_offset + z_offset]);

      // The reason for adding a large value is that when converting from float to int, values
      // around -1 to 1 are all rounded down to 0. Therefore, to prevent the numbers from becoming
      // negative, a large value is added. It has been tuned to reduce computational costs, and
      // deliberately avoids using round or floor functions.
      VoxelKey key = {
        static_cast<int>((x + large_num_offset) * inverse_voxel_size_x),
        static_cast<int>((y + large_num_offset) * inverse_voxel_size_y),
        static_cast<int>((z + large_num_offset) * inverse_voxel_size_z)};

      if (raw_voxel_map.find(key) == raw_voxel_map.end()) {
        raw_voxel_map[key] = global_offset;
      }
    }
  }
  {
    const int x_offset = obstacle_pc.fields[pcl::getFieldIndex(obstacle_pc, "x")].offset;
    const int y_offset = obstacle_pc.fields[pcl::getFieldIndex(obstacle_pc, "y")].offset;
    const int z_offset = obstacle_pc.fields[pcl::getFieldIndex(obstacle_pc, "z")].offset;

    for (size_t global_offset = 0;
         global_offset + obstacle_pc.point_step <= obstacle_pc.data.size();
         global_offset += obstacle_pc.point_step) {
      const float & x =
        *reinterpret_cast<const float *>(&obstacle_pc.data[global_offset + x_offset]);
      const float & y =
        *reinterpret_cast<const float *>(&obstacle_pc.data[global_offset + y_offset]);
      const float & z =
        *reinterpret_cast<const float *>(&obstacle_pc.data[global_offset + z_offset]);

      // The reason for adding a large value is that when converting from float to int, values
      // around -1 to 1 are all rounded down to 0. Therefore, to prevent the numbers from becoming
      // negative, a large value is added. It has been tuned to reduce computational costs, and
      // deliberately avoids using round or floor functions.
      VoxelKey key = {
        static_cast<int>((x + large_num_offset) * inverse_voxel_size_x),
        static_cast<int>((y + large_num_offset) * inverse_voxel_size_y),
        static_cast<int>((z + large_num_offset) * inverse_voxel_size_z)};

      if (raw_voxel_map.find(key) == raw_voxel_map.end()) {
        obstacle_voxel_map[key] = global_offset;
      }
    }

    // Populate the output point cloud
    size_t output_global_offset = 0;
    output_obstacle_pc.data.resize(obstacle_voxel_map.size() * obstacle_pc.point_step);
    for (const auto & kv : obstacle_voxel_map) {
      std::memcpy(
        &output_obstacle_pc.data[output_global_offset + x_offset],
        &obstacle_pc.data[kv.second + x_offset], sizeof(float));
      std::memcpy(
        &output_obstacle_pc.data[output_global_offset + y_offset],
        &obstacle_pc.data[kv.second + y_offset], sizeof(float));
      std::memcpy(
        &output_obstacle_pc.data[output_global_offset + z_offset],
        &obstacle_pc.data[kv.second + z_offset], sizeof(float));
      output_global_offset += obstacle_pc.point_step;
    }

    // Set the output point cloud metadata
    output_obstacle_pc.header.frame_id = obstacle_pc.header.frame_id;
    output_obstacle_pc.height = 1;
    output_obstacle_pc.fields = obstacle_pc.fields;
    output_obstacle_pc.is_bigendian = obstacle_pc.is_bigendian;
    output_obstacle_pc.point_step = obstacle_pc.point_step;
    output_obstacle_pc.is_dense = obstacle_pc.is_dense;
    output_obstacle_pc.width = static_cast<uint32_t>(
      output_obstacle_pc.data.size() / output_obstacle_pc.height / output_obstacle_pc.point_step);
    output_obstacle_pc.row_step =
      static_cast<uint32_t>(output_obstacle_pc.data.size() / output_obstacle_pc.height);
  }

  return true;
}

/**
 * @brief Convert unsigned char value to closest cost value
 * @param cost Cost value
 * @return Probability
 */
unsigned char getApproximateOccupancyState(const unsigned char & value)
{
  if (value >= occupancy_cost_value::OCCUPIED_THRESHOLD) {
    return occupancy_cost_value::LETHAL_OBSTACLE;
  } else if (value <= occupancy_cost_value::FREE_THRESHOLD) {
    return occupancy_cost_value::FREE_SPACE;
  } else {
    return occupancy_cost_value::NO_INFORMATION;
  }
}

}  // namespace utils
