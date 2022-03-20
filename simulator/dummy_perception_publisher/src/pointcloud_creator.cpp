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

void VehicleCentricPointCloudCreator::create(
  const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
  std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const
{
  const double horizontal_theta_step = 0.25 * M_PI / 180.0;
  const auto tf_base_link2moved_object = tf_base_link2map * obj_info.tf_map2moved_object;
  const auto tf_moved_object2base_link = tf_base_link2moved_object.inverse();
  const tf2::Transform tf_rotonly(tf_moved_object2base_link.getRotation());  // For vector rotation

  const auto sdf_box = [&](tf2::Vector3 p) {
    // As for signd distance field for a box, please refere:
    // https://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
    const auto sd_val_x = std::abs(p.getX()) - 0.5 * obj_info.length;
    const auto sd_val_y = std::abs(p.getY()) - 0.5 * obj_info.width;
    const auto positive_dist_x = std::max(sd_val_x, 0.0);
    const auto positive_dist_y = std::max(sd_val_y, 0.0);
    const auto positive_dist = std::hypot(positive_dist_x, positive_dist_y);
    const auto negative_dist = std::min(std::max(sd_val_x, sd_val_y), 0.0);
    return positive_dist + negative_dist;
  };

  const auto compute_dist_by_spheretrace =
    [&](const tf2::Vector3 & start, const tf2::Vector3 & direction) -> double {
    // As for sphere tracing, please refere:
    // https://computergraphics.stackexchange.com/questions/161/what-is-ray-marching-is-sphere-tracing-the-same-thing/163
    const double eps = 1e-3;
    const size_t max_iter = 20;

    auto tip = start;
    for (size_t itr = 0; itr < max_iter; ++itr) {
      const double dist = sdf_box(tip);
      tip = tip + dist * direction;
      bool almost_on_surface = std::abs(dist) < eps;
      if (almost_on_surface) {
        return tf2::tf2Distance(tip, start);
      }
    }
    // ray did not hit the surface.
    return std::numeric_limits<double>::infinity();
  };

  std::normal_distribution<> x_random(0.0, obj_info.std_dev_x);
  std::normal_distribution<> y_random(0.0, obj_info.std_dev_y);
  std::normal_distribution<> z_random(0.0, obj_info.std_dev_z);

  double angle = 0.0;
  const size_t n_scan = static_cast<size_t>(std::floor(2 * M_PI / horizontal_theta_step));
  for (size_t i = 0; i < n_scan; ++i) {
    angle += horizontal_theta_step;

    const tf2::Vector3 dir_wrt_car(cos(angle), sin(angle), 0.0);
    const tf2::Vector3 dir_wrt_obj = tf_rotonly * dir_wrt_car;
    const auto start_ray_pos = tf_moved_object2base_link.getOrigin();
    const tf2::Vector3 pos_noise(
      -x_random(random_generator), -y_random(random_generator), -z_random(random_generator));
    const auto start_ray_pos_with_noise = start_ray_pos + pos_noise;
    const float ray_dist = compute_dist_by_spheretrace(start_ray_pos_with_noise, dir_wrt_obj);
    if (std::isfinite(ray_dist)) {
      const auto ray_tip = start_ray_pos_with_noise + ray_dist * dir_wrt_obj;
      const auto point = getBaseLinkToPoint(
        tf_base_link2moved_object, ray_tip.getX(), ray_tip.getY(), ray_tip.getZ());
      pointcloud->push_back(point);
    }
  }
}
