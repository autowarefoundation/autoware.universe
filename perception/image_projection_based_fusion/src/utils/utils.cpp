// Copyright 2022 TIER IV, Inc.
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

#include "image_projection_based_fusion/utils/utils.hpp"
namespace image_projection_based_fusion
{

std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(
  const tf2_ros::Buffer & tf_buffer, const std::string & target_frame_id,
  const std::string & source_frame_id, const rclcpp::Time & time)
{
  try {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf_buffer.lookupTransform(
      target_frame_id, source_frame_id, time, rclcpp::Duration::from_seconds(0.01));
    return transform_stamped;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("image_projection_based_fusion"), ex.what());
    return std::nullopt;
  }
}

Eigen::Affine3d transformToEigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Affine3d a;
  a.matrix() = tf2::transformToEigen(t).matrix();
  return a;
}

PointCloud closest_cluster(
  PointCloud & cluster, const double cluster_threshold_radius,
  [[maybe_unused]] const double cluster_threshold_distance, const int min_cluster_size)
{
  // suppose cluster's frame_id is base_link

  // sort pointcloud by distance
  auto func = [](const pcl::PointXYZ & p1, const pcl::PointXYZ & p2) {
    return tier4_autoware_utils::calcDistance2d(pcl::PointXYZ(0.0, 0.0, 0.0), p1) <
           tier4_autoware_utils::calcDistance2d(pcl::PointXYZ(0.0, 0.0, 0.0), p2);
  };
  std::sort(cluster.begin(), cluster.end(), func);
  PointCloud out_cluster;
  for (auto & point : cluster) {
    if (out_cluster.empty()) {
      out_cluster.push_back(point);
      continue;
    }
    if (
      tier4_autoware_utils::calcDistance2d(out_cluster.back(), point) < cluster_threshold_radius) {
      out_cluster.push_back(point);
      continue;
    }
    if (out_cluster.size() >= static_cast<std::size_t>(min_cluster_size)) {
      return out_cluster;
    }
    out_cluster.clear();
    out_cluster.push_back(point);
  }
  return out_cluster;
}

void addShapeAndKinematic(
  const pcl::PointCloud<pcl::PointXYZ> & cluster,
  tier4_perception_msgs::msg::DetectedObjectWithFeature & feature_obj)
{
  if (cluster.empty()) {
    return;
  }
  pcl::PointXYZ centroid = pcl::PointXYZ(0.0, 0.0, 0.0);
  float max_z = -1e6;
  float min_z = 1e6;
  for (const auto & point : cluster) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
    max_z = max_z < point.z ? point.z : max_z;
    min_z = min_z > point.z ? point.z : min_z;
  }
  centroid.x = centroid.x / static_cast<double>(cluster.size());
  centroid.y = centroid.y / static_cast<double>(cluster.size());
  centroid.z = centroid.z / static_cast<double>(cluster.size());

  std::vector<cv::Point> cluster2d;
  std::vector<cv::Point> cluster2d_convex;

  for (size_t i = 0; i < cluster.size(); ++i) {
    cluster2d.push_back(
      cv::Point((cluster.at(i).x - centroid.x) * 1000.0, (cluster.at(i).y - centroid.y) * 1000.));
  }
  cv::convexHull(cluster2d, cluster2d_convex);
  if (cluster2d_convex.empty()) {
    return;
  }
  pcl::PointXYZ polygon_centroid = pcl::PointXYZ(0.0, 0.0, 0.0);
  for (size_t i = 0; i < cluster2d_convex.size(); ++i) {
    polygon_centroid.x += static_cast<double>(cluster2d_convex.at(i).x) / 1000.0;
    polygon_centroid.y += static_cast<double>(cluster2d_convex.at(i).y) / 1000.0;
  }
  polygon_centroid.x = polygon_centroid.x / static_cast<double>(cluster2d_convex.size());
  polygon_centroid.y = polygon_centroid.y / static_cast<double>(cluster2d_convex.size());

  autoware_auto_perception_msgs::msg::Shape shape;
  for (size_t i = 0; i < cluster2d_convex.size(); ++i) {
    geometry_msgs::msg::Point32 point;
    point.x = cluster2d_convex.at(i).x / 1000.0;
    point.y = cluster2d_convex.at(i).y / 1000.0;
    point.z = 0.0;
    shape.footprint.points.push_back(point);
  }
  shape.type = autoware_auto_perception_msgs::msg::Shape::POLYGON;
  constexpr float eps = 0.01;
  shape.dimensions.x = 0;
  shape.dimensions.y = 0;
  shape.dimensions.z = std::max((max_z - min_z), eps);
  feature_obj.object.shape = shape;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.x =
    centroid.x + polygon_centroid.x;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.y =
    centroid.y + polygon_centroid.y;
  feature_obj.object.kinematics.pose_with_covariance.pose.position.z =
    min_z + shape.dimensions.z * 0.5;
  feature_obj.object.existence_probability = 1.0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.x = 0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.y = 0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.z = 0;
  feature_obj.object.kinematics.pose_with_covariance.pose.orientation.w = 1;
}

geometry_msgs::msg::Point getCentroid(const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.0f;
  centroid.y = 0.0f;
  centroid.z = 0.0f;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"),
       iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    centroid.x += *iter_x;
    centroid.y += *iter_y;
    centroid.z += *iter_z;
  }
  const size_t size = pointcloud.width * pointcloud.height;
  centroid.x = centroid.x / static_cast<float>(size);
  centroid.y = centroid.y / static_cast<float>(size);
  centroid.z = centroid.z / static_cast<float>(size);
  return centroid;
}

// TODO : change to template
pcl::PointXYZ getClosestPoint(const pcl::PointCloud<pcl::PointXYZ> & cluster)
{
  pcl::PointXYZ closest_point;
  double min_dist = 1e6;
  pcl::PointXYZ orig_point = pcl::PointXYZ(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < cluster.points.size(); ++i) {
    pcl::PointXYZ point = cluster.points.at(i);
    double dist_closest_point = tier4_autoware_utils::calcDistance2d(point, orig_point);
    if (min_dist > dist_closest_point) {
      min_dist = dist_closest_point;
      closest_point = pcl::PointXYZ(point.x, point.y, point.z);
    }
  }
  return closest_point;
}

}  // namespace image_projection_based_fusion
