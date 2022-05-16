// #define LOGGING_PROCESSING_TIME

#include <iostream>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "modularized_particle_filter_msgs/msg/particle_array.hpp"

#include "modularized_particle_filter/correction/correction_util.hpp"
#include "modularized_particle_filter/correction/lidar_pose_corrector.hpp"

namespace
{

inline std::tuple<float, float> scaleDown(float x, float y, float resolution)
{
  return {x * resolution, y * resolution};
}

inline std::tuple<int, int> scaleUpAndRoundToGridUpperLeft(float x, float y, float resolution)
{
  return {
    static_cast<int>(std::floor(x / resolution)),
    static_cast<int>(std::floor(y / resolution))
  };
}

inline std::tuple<float, float> roundToGridUpperLeft(float x, float y, float resolution)
{
  return {
    resolution * std::floor(x / resolution),
    resolution * std::floor(y / resolution)
  };
}

inline int xY2Index(float x, float y, nav_msgs::msg::MapMetaData map_info)
{
  return static_cast<int>((y - map_info.origin.position.y) / map_info.resolution) * map_info.width +
         static_cast<int>((x - map_info.origin.position.x) / map_info.resolution);
}

nav_msgs::msg::OccupancyGrid updateLocalLikelihoodGridMap(
  nav_msgs::msg::MapMetaData likelihood_map_meta_data,
  std::vector<std::pair<int, std::vector<std::pair<int, int>>>> likelihood_jagged_array,
  nav_msgs::msg::OccupancyGrid occupancy_grid,
  modularized_particle_filter_msgs::msg::ParticleArray particles)
{
  geometry_msgs::msg::Point center_xy{};

  auto x_minmax = std::minmax_element(
    particles.particles.begin(), particles.particles.end(),
    [](
      const modularized_particle_filter_msgs::msg::Particle & ps1,
      const modularized_particle_filter_msgs::msg::Particle & ps2) {
      return ps1.pose.position.x < ps2.pose.position.x;
    });
  auto y_minmax = std::minmax_element(
    particles.particles.begin(), particles.particles.end(),
    [](
      const modularized_particle_filter_msgs::msg::Particle & ps1,
      const modularized_particle_filter_msgs::msg::Particle & ps2) {
      return ps1.pose.position.y < ps2.pose.position.y;
    });

  center_xy.x = 0.5 * (x_minmax.first->pose.position.x + x_minmax.second->pose.position.x);
  center_xy.y = 0.5 * (y_minmax.first->pose.position.y + y_minmax.second->pose.position.y);

  float half_length_x = (occupancy_grid.info.width / 2) * occupancy_grid.info.resolution;
  float half_length_y = (occupancy_grid.info.height / 2) * occupancy_grid.info.resolution;

  auto [rounded_min_x, rounded_min_y] = roundToGridUpperLeft(
    static_cast<float>(center_xy.x) - half_length_x,
    static_cast<float>(center_xy.y) - half_length_y,
    occupancy_grid.info.resolution);
  occupancy_grid.info.origin.position.x = rounded_min_x;
  occupancy_grid.info.origin.position.y = rounded_min_y;

  auto [up_scaled_min_x, up_scaled_min_y] = scaleUpAndRoundToGridUpperLeft(
    occupancy_grid.info.origin.position.x,
    occupancy_grid.info.origin.position.y,
    likelihood_map_meta_data.resolution);
  auto [up_scaled_max_x, up_scaled_max_y] = scaleUpAndRoundToGridUpperLeft(
    static_cast<float>(center_xy.x) + half_length_x,
    static_cast<float>(center_xy.y) + half_length_y,
    likelihood_map_meta_data.resolution);

  std::fill(occupancy_grid.data.begin(), occupancy_grid.data.end(), 0);

  for (auto y_itr{likelihood_jagged_array.begin()}, y_itr_end{likelihood_jagged_array.end()};
    y_itr != y_itr_end; y_itr++)
  {
    if (y_itr->first < up_scaled_min_y) {continue;}
    if (up_scaled_max_y < y_itr->first) {break;}

    for (auto x_itr{y_itr->second.begin()}, x_itr_end{y_itr->second.end()};
      x_itr != x_itr_end; x_itr++)
    {
      if (x_itr->first < up_scaled_min_x) {continue;}
      if (up_scaled_max_x < x_itr->first) {break;}

      auto [down_scaled_x, down_scaled_y] = scaleDown(
        x_itr->first, y_itr->first, likelihood_map_meta_data.resolution);
      const int index = xY2Index(
        down_scaled_x, down_scaled_y, occupancy_grid.info);
      if (index < 0 || static_cast<int>(occupancy_grid.data.size()) <= index) {continue;}

      occupancy_grid.data.data()[index] = static_cast<int8_t>(x_itr->second);
    }
  }

  return occupancy_grid;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr, geometry_msgs::msg::Pose pose)
{
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0.0f));
  transform.setRotation(
    tf2::Quaternion(
      pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_ptr{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl_ros::transformPointCloud(*cloud_ptr, *cloud_transformed_ptr, transform);
  return cloud_transformed_ptr;
}

float calculateLocalGridMatchingWeight(
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr,
  nav_msgs::msg::OccupancyGrid occupancy_grid, float grid_x_max, float grid_y_max)
{
  int index{0};
  float weight{0.0f};
  for (const auto & point : cloud_ptr->points) {
    if (
      point.x < occupancy_grid.info.origin.position.x || grid_x_max < point.x)
    {
      continue;
    }
    if (
      point.y < occupancy_grid.info.origin.position.y || grid_y_max < point.y)
    {
      continue;
    }

    index = xY2Index(point.x, point.y, occupancy_grid.info);

    weight += occupancy_grid.data.data()[index];
    // weight += cloud.points[i].intensity * occupancy_grid.data.data()[index];
  }
  return weight;
}

}  // namespace

LidarPoseCorrector::LidarPoseCorrector()
: Node("lidar_pose_corrector"),
  map_resolution_(declare_parameter("map_resolution", 0.2f)),
  particles_buffer_size_(declare_parameter("particles_buffer_size", 50)),
  particles_circular_buffer_(particles_buffer_size_)
{
  particles_circular_buffer_ =
    boost::circular_buffer<modularized_particle_filter_msgs::msg::ParticleArray>(
    particles_buffer_size_);

  occupancy_grid_.info.resolution = map_resolution_;
  occupancy_grid_.info.width = declare_parameter("map_grid_width", 1000);
  occupancy_grid_.info.height = declare_parameter("map_grid_height", 1000);
  occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height);

  prev_time_ = this->now();

  weighted_particle_pub_ =
    this->create_publisher<modularized_particle_filter_msgs::msg::ParticleArray>(
    "weighted_particles", 10);

  map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "map_pointcloud", rclcpp::QoS{1}.transient_local(),
    std::bind(&LidarPoseCorrector::mapPointCloudCallback, this, std::placeholders::_1));
  particle_sub_ =
    this->create_subscription<modularized_particle_filter_msgs::msg::ParticleArray>(
    "predicted_particles", 10,
    std::bind(&LidarPoseCorrector::particleCallback, this, std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "scan_pointcloud", rclcpp::SensorDataQoS().keep_last(0),
    std::bind(&LidarPoseCorrector::scanPointCloudCallback, this, std::placeholders::_1));
}

void LidarPoseCorrector::mapPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg)
{
#if 1   // debug
  auto start = std::chrono::system_clock::now();
#endif  // debug

  pcl::PointCloud<pcl::PointXYZL>::Ptr likelihood_pointcloud{new pcl::PointCloud<pcl::PointXYZL>()};
  pcl::fromROSMsg(*pointcloud_msg, *likelihood_pointcloud);

  pcl::PointXYZL min_point, max_point;
  pcl::getMinMax3D(*likelihood_pointcloud, min_point, max_point);
  auto [rounded_min_x, rounded_min_y] = roundToGridUpperLeft(
    min_point.x, min_point.y,
    map_resolution_);
  auto [rounded_max_x, rounded_max_y] = roundToGridUpperLeft(
    max_point.x, max_point.y,
    map_resolution_);
  likelihood_map_meta_data_.origin.position.x = rounded_min_x;
  likelihood_map_meta_data_.origin.position.y = rounded_min_y;
  const float resolution_half{0.5f * map_resolution_};
  likelihood_map_meta_data_.resolution = resolution_half;
  likelihood_map_meta_data_.width = (rounded_max_x - rounded_min_x) /
    likelihood_map_meta_data_.resolution;
  likelihood_map_meta_data_.height = (rounded_max_y - rounded_min_y) /
    likelihood_map_meta_data_.resolution;

  std::unordered_map<int, std::unordered_map<int, int>> likelihood_map{};
  for (pcl::PointXYZL & point : likelihood_pointcloud->points) {
    for (float x{point.x - resolution_half}; x <= point.x + resolution_half; x += resolution_half) {
      for (float y{point.y - resolution_half}; y <= point.y + resolution_half;
        y += resolution_half)
      {
        if (x < likelihood_map_meta_data_.origin.position.x || rounded_max_x < x) {
          continue;
        }
        if (y < likelihood_map_meta_data_.origin.position.y || rounded_max_y < y) {
          continue;
        }

        auto[up_scaled_x, up_scaled_y] = scaleUpAndRoundToGridUpperLeft(
          x, y,
          likelihood_map_meta_data_.resolution);

        decltype(likelihood_map)::iterator likelihood_map_y_iterator{likelihood_map.find(up_scaled_y)};
        if (likelihood_map_y_iterator != likelihood_map.end()) {
          decltype(likelihood_map_y_iterator->second)::iterator likelihood_map_yx_iterator{
            likelihood_map_y_iterator->second.find(up_scaled_x)};
          if (likelihood_map_yx_iterator != likelihood_map_y_iterator->second.end()) {
            if (likelihood_map_yx_iterator->second < static_cast<int>(point.label)) {
              likelihood_map_yx_iterator->second = point.label;
            }
          } else {
            likelihood_map_y_iterator->second.insert(std::make_pair(up_scaled_x, point.label));
          }
        } else {
          std::unordered_map<int, int> likelihood_map_y {{up_scaled_x, point.label}};
          likelihood_map.insert(std::make_pair(up_scaled_y, likelihood_map_y));
        }
      }
    }
  }

  std::vector<std::pair<int, std::vector<std::pair<int, int>>>> likelihood_map_as_jagged_array{};
  for (std::pair<int, std::unordered_map<int, int>> likelihood_map_y : likelihood_map) {
    std::vector<std::pair<int, int>> likelihood_map_as_jagged_array_y(
      likelihood_map_y.second.begin(), likelihood_map_y.second.end());
    sort(
      likelihood_map_as_jagged_array_y.begin(), likelihood_map_as_jagged_array_y.end(),
      [](std::pair<int, int> lower, std::pair<int, int> upper) {return lower.first < upper.first;});
    likelihood_map_as_jagged_array.push_back(
      std::make_pair(
        likelihood_map_y.first,
        likelihood_map_as_jagged_array_y));
  }
  sort(
    likelihood_map_as_jagged_array.begin(), likelihood_map_as_jagged_array.end(),
    [](std::pair<int, std::vector<std::pair<int, int>>> lower, std::pair<int,
    std::vector<std::pair<int, int>>> upper) {return lower.first < upper.first;});

  likelihood_map_as_jagged_array_opt_ = likelihood_map_as_jagged_array;

#if 1   // debug
  RCLCPP_WARN_STREAM(
    this->get_logger(),
    "likelihood_map prepared: " <<
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start)
      .count() << "[ms]");
#endif  // debug
}

void LidarPoseCorrector::particleCallback(
  const modularized_particle_filter_msgs::msg::ParticleArray::ConstSharedPtr particles)
{
  particles_circular_buffer_.push_front(*particles);
}

void LidarPoseCorrector::scanPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_pointcloud_msg)
{
#ifdef LOGGING_PROCESSING_TIME   // debug
  std::chrono::time_point<std::chrono::system_clock> start{};
#endif  // debug

  if (!likelihood_map_as_jagged_array_opt_.has_value()) {
    RCLCPP_WARN(this->get_logger(), "No map.");
    return;
  }
  // if ((rclcpp::Time(lidar_pointcloud_msg->header.stamp) - prev_time_).seconds() < 1) {
  //   prev_time_ = lidar_pointcloud_msg->header.stamp;
  //   return;
  // }

  std::optional<modularized_particle_filter_msgs::msg::ParticleArray> synced_particles_opt{
    findSyncedParticles(particles_circular_buffer_, lidar_pointcloud_msg->header.stamp)};
  if (!synced_particles_opt.has_value()) {
    RCLCPP_WARN(this->get_logger(), "No synced particles.");
    return;
  }

  modularized_particle_filter_msgs::msg::ParticleArray synced_particles{synced_particles_opt.value()};

#ifdef LOGGING_PROCESSING_TIME   // debug
  start = std::chrono::system_clock::now();
#endif  // debug

  occupancy_grid_ = updateLocalLikelihoodGridMap(
    likelihood_map_meta_data_,
    likelihood_map_as_jagged_array_opt_.value(), occupancy_grid_, synced_particles);

#ifdef LOGGING_PROCESSING_TIME   // debug
  RCLCPP_WARN_STREAM(
    this->get_logger(),
    "updateLocalLikelihoodGridMap: " <<
      std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now() - start)
      .count() << "[ms]");
#endif  // debug

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_pointcloud_ptr{new pcl::PointCloud<pcl::PointXYZ>()};
  pcl::fromROSMsg(*lidar_pointcloud_msg, *lidar_pointcloud_ptr);

#ifdef LOGGING_PROCESSING_TIME  // debug
  double transform_miliseconds{0.0};
  double weighting_miliseconds{0.0};
  double estimate_z_miliseconds{0.0};
  std::chrono::duration<double, std::milli> fp_ms;
#endif  // debug

  float grid_x_max = occupancy_grid_.info.origin.position.x +
    (occupancy_grid_.info.width) * occupancy_grid_.info.resolution;
  float grid_y_max = occupancy_grid_.info.origin.position.y +
    (occupancy_grid_.info.height) * occupancy_grid_.info.resolution;
  for (int i{0}; i < static_cast<int>(synced_particles.particles.size()); i++) {
    geometry_msgs::msg::Pose pose_z0{synced_particles.particles[i].pose};
    pose_z0.position.z = 0.0f;

#ifdef LOGGING_PROCESSING_TIME  // debug
    auto t1 = std::chrono::high_resolution_clock::now();
#endif  // debug
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed_ptr{
      transformPointCloud(lidar_pointcloud_ptr, pose_z0)};
    // RCLCPP_WARN_STREAM(
    //   this->get_logger(),
    //   "point size: " << lidar_pointcloud_ptr->width <<
    //     ", " << cloud_transformed_ptr->width);

#ifdef LOGGING_PROCESSING_TIME  // debug
    auto t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;
    transform_miliseconds += fp_ms.count();
#endif  // debug

#ifdef LOGGING_PROCESSING_TIME  // debug
    t1 = std::chrono::high_resolution_clock::now();
#endif  // debug
    // CalculateLocalGridMatchingWeights time: 375
    synced_particles.particles[i].weight = calculateLocalGridMatchingWeight(
      cloud_transformed_ptr, occupancy_grid_, grid_x_max, grid_y_max);
#ifdef LOGGING_PROCESSING_TIME  // debug
    t2 = std::chrono::high_resolution_clock::now();
    fp_ms = t2 - t1;
    weighting_miliseconds += fp_ms.count();
#endif  // debug

  }

#ifdef LOGGING_PROCESSING_TIME   // debug
  RCLCPP_WARN_STREAM(
    this->get_logger(),
    "transformPointCloud: " << transform_miliseconds << "[ms]");
  RCLCPP_WARN_STREAM(
    this->get_logger(),
    "calculateLocalGridMatchingWeight: " << transform_miliseconds << "[ms]");
#endif  // debug

  auto minmax_weight =
    std::minmax_element(
    synced_particles.particles.begin(), synced_particles.particles.end(),
    [](auto p1, auto p2) {return p1.weight < p2.weight;});
  float num_of_particles_inv{1.0f / synced_particles.particles.size()};
  float dif_weight{minmax_weight.second->weight - minmax_weight.first->weight};
  for (modularized_particle_filter_msgs::msg::Particle & weighted_particle :
    synced_particles.particles)
  {
    if (dif_weight != 0.0f) {
      weighted_particle.weight = (weighted_particle.weight - minmax_weight.first->weight) /
        dif_weight;
    } else {
      weighted_particle.weight = num_of_particles_inv;
    }
  }

  weighted_particle_pub_->publish(synced_particles);
}
