// Copyright 2020 Tier IV, Inc.
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

#include "dummy_perception_publisher/node.hpp"
#include "dummy_perception_publisher/signed_distance_function.hpp"

#include <pcl/impl/point_types.hpp>

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <functional>
#include <limits>

pcl::PointXYZ getBaseLinkToPoint(
  const tf2::Transform & tf_base_link2moved_object, double x, double y, double z)
{
  tf2::Transform tf_moved_object2point;
  tf2::Transform tf_base_link2point;
  geometry_msgs::msg::Transform ros_moved_object2point;
  ros_moved_object2point.translation.x = x;
  ros_moved_object2point.translation.y = y;
  ros_moved_object2point.translation.z = z;
  ros_moved_object2point.rotation.x = 0;
  ros_moved_object2point.rotation.y = 0;
  ros_moved_object2point.rotation.z = 0;
  ros_moved_object2point.rotation.w = 1;
  tf2::fromMsg(ros_moved_object2point, tf_moved_object2point);
  tf_base_link2point = tf_base_link2moved_object * tf_moved_object2point;
  pcl::PointXYZ point;
  point.x = tf_base_link2point.getOrigin().x();
  point.y = tf_base_link2point.getOrigin().y();
  point.z = tf_base_link2point.getOrigin().z();
  return point;
};

void ObjectCentricPointCloudCreator::create(
  const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const
{
  std::normal_distribution<> x_random(0.0, obj_info.std_dev_x);
  std::normal_distribution<> y_random(0.0, obj_info.std_dev_y);
  std::normal_distribution<> z_random(0.0, obj_info.std_dev_z);
  const double epsilon = 0.001;
  const double step = 0.05;
  const double vertical_theta_step = (1.0 / 180.0) * M_PI;
  const double vertical_min_theta = (-15.0 / 180.0) * M_PI;
  const double vertical_max_theta = (15.0 / 180.0) * M_PI;
  const double horizontal_theta_step = (0.1 / 180.0) * M_PI;
  const double horizontal_min_theta = (-180.0 / 180.0) * M_PI;
  const double horizontal_max_theta = (180.0 / 180.0) * M_PI;

  const auto tf_base_link2moved_object = tf_base_link2map * obj_info.tf_map2moved_object;

  const double min_z = -1.0 * (obj_info.height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  const double max_z = 1.0 * (obj_info.height / 2.0) + tf_base_link2moved_object.getOrigin().z();
  pcl::PointCloud<pcl::PointXYZ> horizontal_candidate_pointcloud;
  pcl::PointCloud<pcl::PointXYZ> horizontal_pointcloud;
  {
    const double y = -1.0 * (obj_info.width / 2.0);
    for (double x = -1.0 * (obj_info.length / 2.0); x <= ((obj_info.length / 2.0) + epsilon);
         x += step) {
      horizontal_candidate_pointcloud.push_back(
        getBaseLinkToPoint(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  {
    const double y = 1.0 * (obj_info.width / 2.0);
    for (double x = -1.0 * (obj_info.length / 2.0); x <= ((obj_info.length / 2.0) + epsilon);
         x += step) {
      horizontal_candidate_pointcloud.push_back(
        getBaseLinkToPoint(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  {
    const double x = -1.0 * (obj_info.length / 2.0);
    for (double y = -1.0 * (obj_info.width / 2.0); y <= ((obj_info.width / 2.0) + epsilon);
         y += step) {
      horizontal_candidate_pointcloud.push_back(
        getBaseLinkToPoint(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  {
    const double x = 1.0 * (obj_info.length / 2.0);
    for (double y = -1.0 * (obj_info.width / 2.0); y <= ((obj_info.width / 2.0) + epsilon);
         y += step) {
      horizontal_candidate_pointcloud.push_back(
        getBaseLinkToPoint(tf_base_link2moved_object, x, y, 0.0));
    }
  }
  // 2D ray tracing
  size_t ranges_size =
    std::ceil((horizontal_max_theta - horizontal_min_theta) / horizontal_theta_step);
  std::vector<double> horizontal_ray_traced_2d_pointcloud;
  horizontal_ray_traced_2d_pointcloud.assign(ranges_size, std::numeric_limits<double>::infinity());
  const int no_data = -1;
  std::vector<int> horizontal_ray_traced_pointcloud_indices;
  horizontal_ray_traced_pointcloud_indices.assign(ranges_size, no_data);
  for (size_t i = 0; i < horizontal_candidate_pointcloud.points.size(); ++i) {
    double angle =
      std::atan2(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
    double range =
      std::hypot(horizontal_candidate_pointcloud.at(i).y, horizontal_candidate_pointcloud.at(i).x);
    if (angle < horizontal_min_theta || angle > horizontal_max_theta) {
      continue;
    }
    int index = (angle - horizontal_min_theta) / horizontal_theta_step;
    if (range < horizontal_ray_traced_2d_pointcloud[index]) {
      horizontal_ray_traced_2d_pointcloud[index] = range;
      horizontal_ray_traced_pointcloud_indices.at(index) = i;
    }
  }

  for (const auto & pointcloud_index : horizontal_ray_traced_pointcloud_indices) {
    if (pointcloud_index != no_data) {
      // generate vertical point
      horizontal_pointcloud.push_back(horizontal_candidate_pointcloud.at(pointcloud_index));
      const double distance = std::hypot(
        horizontal_candidate_pointcloud.at(pointcloud_index).x,
        horizontal_candidate_pointcloud.at(pointcloud_index).y);
      for (double vertical_theta = vertical_min_theta;
           vertical_theta <= vertical_max_theta + epsilon; vertical_theta += vertical_theta_step) {
        const double z = distance * std::tan(vertical_theta);
        if (min_z <= z && z <= max_z + epsilon) {
          pcl::PointXYZ point;
          point.x =
            horizontal_candidate_pointcloud.at(pointcloud_index).x + x_random(random_generator);
          point.y =
            horizontal_candidate_pointcloud.at(pointcloud_index).y + y_random(random_generator);
          point.z = z + z_random(random_generator);
          pointcloud->push_back(point);
        }
      }
    }
  }
}

void ObjectCentricPointCloudCreator::create_multi(
  const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & pointclouds,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointclouds_tmp;
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_tmp(new pcl::PointCloud<pcl::PointXYZ>);

  for (const auto & obj_info : obj_infos) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_shared_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->create(obj_info, tf_base_link2map, random_generator, pointcloud_shared_ptr);
    pointclouds_tmp.push_back(pointcloud_shared_ptr);
  }

  for (size_t i = 0; i < pointclouds_tmp.size(); ++i) {
    for (size_t j = 0; j < pointclouds_tmp.at(i)->size(); ++j) {
      merged_pointcloud_tmp->push_back(pointclouds_tmp.at(i)->at(j));
    }
  }

  if (!enable_ray_tracing_) {
    pointclouds = pointclouds_tmp;
    merged_pointcloud = merged_pointcloud_tmp;
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ray_traced_merged_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> ray_tracing_filter;
  ray_tracing_filter.setInputCloud(merged_pointcloud_tmp);
  ray_tracing_filter.setLeafSize(0.25, 0.25, 0.25);
  ray_tracing_filter.initializeVoxelGrid();
  for (size_t i = 0; i < pointclouds_tmp.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr ray_traced_pointcloud_ptr(
      new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t j = 0; j < pointclouds_tmp.at(i)->size(); ++j) {
      Eigen::Vector3i grid_coordinates = ray_tracing_filter.getGridCoordinates(
        pointclouds_tmp.at(i)->at(j).x, pointclouds_tmp.at(i)->at(j).y,
        pointclouds_tmp.at(i)->at(j).z);
      int grid_state;
      if (ray_tracing_filter.occlusionEstimation(grid_state, grid_coordinates) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("dummy_perception_publisher"), "ray tracing failed");
      }
      if (grid_state == 1) {  // occluded
        continue;
      } else {  // not occluded
        ray_traced_pointcloud_ptr->push_back(pointclouds_tmp.at(i)->at(j));
        ray_traced_merged_pointcloud_ptr->push_back(pointclouds_tmp.at(i)->at(j));
      }
    }
    pointclouds.push_back(ray_traced_pointcloud_ptr);
  }
  merged_pointcloud = ray_traced_merged_pointcloud_ptr;
}

double compute_box_signed_distance(const tf2::Vector3 & p, const ObjectInfo & obj_info)
{
  // As for signd distance field for a box, please refere:
  // https://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
  const auto sd_val_x = std::abs(p.getX()) - 0.5 * obj_info.length;
  const auto sd_val_y = std::abs(p.getY()) - 0.5 * obj_info.width;
  const auto positive_dist_x = std::max(sd_val_x, 0.0);
  const auto positive_dist_y = std::max(sd_val_y, 0.0);
  const auto positive_dist = std::hypot(positive_dist_x, positive_dist_y);
  const auto negative_dist = std::min(std::max(sd_val_x, sd_val_y), 0.0);
  return positive_dist + negative_dist;
}

double compute_boxes_signed_distance(
  const tf2::Vector3 & p, const std::vector<ObjectInfo> & obj_infos)
{
  double min_dist = std::numeric_limits<double>::max();
  for (const auto & obj_info : obj_infos) {
    min_dist = std::min(min_dist, compute_box_signed_distance(p, obj_info));
  }
  return min_dist;
}

double compute_dist_by_spheretrace(
  const tf2::Vector3 & start, const tf2::Vector3 & direction,
  const std::function<double(const tf2::Vector3 & p)> & sd_func)
{
  // As for sphere tracing, please refere:
  // https://computergraphics.stackexchange.com/questions/161/what-is-ray-marching-is-sphere-tracing-the-same-thing/163
  const double eps = 1e-3;
  const size_t max_iter = 20;

  auto tip = start;
  for (size_t itr = 0; itr < max_iter; ++itr) {
    const double dist = sd_func(tip);
    tip = tip + dist * direction;
    bool almost_on_surface = std::abs(dist) < eps;
    if (almost_on_surface) {
      return tf2::tf2Distance(tip, start);
    }
  }
  // ray did not hit the surface.
  return std::numeric_limits<double>::infinity();
};

void show_transform(tf2::Transform tf)
{
  RCLCPP_INFO_STREAM(
    rclcpp::get_logger("ishida_origin"), tf.getOrigin().getX() << ", " << tf.getOrigin().getY());
  RCLCPP_INFO_STREAM(rclcpp::get_logger("ishida"), tf.getRotation().getAngle());
}

void VehicleCentricPointCloudCreator::create(
  const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const
{
  const double horizontal_theta_step = 0.25 * M_PI / 180.0;

  std::normal_distribution<> x_random(0.0, obj_info.std_dev_x);
  std::normal_distribution<> y_random(0.0, obj_info.std_dev_y);
  std::normal_distribution<> z_random(0.0, obj_info.std_dev_z);

  const auto box_sdf = signed_distance_function::BoxSDF(
    obj_info.length, obj_info.width, (tf_base_link2map * obj_info.tf_map2moved_object).inverse());

  double angle = 0.0;
  const size_t n_scan = static_cast<size_t>(std::floor(2 * M_PI / horizontal_theta_step));
  for (size_t i = 0; i < n_scan; ++i) {
    angle += horizontal_theta_step;
    const auto dist = box_sdf.getSphereTracingDist(0.0, 0.0, angle);

    if (std::isfinite(dist)) {
      pcl::PointXYZ point(dist * cos(angle), dist * sin(angle), 0.0);
      pointcloud->push_back(point);
    }
  }
}

void VehicleCentricPointCloudCreator::create_multi(
  const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & pointclouds,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const
{
  for (const auto & obj_info : obj_infos) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_shared_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->create(obj_info, tf_base_link2map, random_generator, pointcloud_shared_ptr);
    pointclouds.push_back(pointcloud_shared_ptr);
  }

  for (size_t i = 0; i < pointclouds.size(); ++i) {
    for (size_t j = 0; j < pointclouds.at(i)->size(); ++j) {
      merged_pointcloud->push_back(pointclouds.at(i)->at(j));
    }
  }
}
