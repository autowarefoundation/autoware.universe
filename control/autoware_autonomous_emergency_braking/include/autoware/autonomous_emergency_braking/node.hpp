// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_
#define AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_

#include "autoware/universe_utils/system/time_keeper.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_system_msgs/msg/autoware_state.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_debug_msgs/msg/float32_stamped.hpp>
#include <tier4_metric_msgs/msg/metric.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/optional.hpp>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
namespace autoware::motion::control::autonomous_emergency_braking
{

using autoware_planning_msgs::msg::Trajectory;
using autoware_system_msgs::msg::AutowareState;
using autoware_vehicle_msgs::msg::VelocityReport;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using autoware::universe_utils::Polygon2d;
using autoware::universe_utils::Polygon3d;
using autoware::vehicle_info_utils::VehicleInfo;
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_updater::Updater;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using Path = std::vector<geometry_msgs::msg::Pose>;
using Vector3 = geometry_msgs::msg::Vector3;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using colorTuple = std::tuple<double, double, double, double>;
using Metric = tier4_metric_msgs::msg::Metric;
using MetricArray = tier4_metric_msgs::msg::MetricArray;

/**
 * @brief Struct to store object data
 */
struct ObjectData
{
  rclcpp::Time stamp;
  geometry_msgs::msg::Point position;
  double velocity{0.0};
  double rss{0.0};
  double distance_to_object{0.0};
  bool is_target{true};
};

/**
 * @brief Class to manage collision data
 */
class CollisionDataKeeper
{
public:
  /**
   * @brief Constructor for CollisionDataKeeper
   * @param clock Shared pointer to the clock
   */
  explicit CollisionDataKeeper(rclcpp::Clock::SharedPtr clock) { clock_ = clock; }

  /**
   * @brief Set timeout values for collision and obstacle data
   * @param collision_keep_time Time to keep collision data
   * @param previous_obstacle_keep_time Time to keep previous obstacle data
   */
  void setTimeout(const double collision_keep_time, const double previous_obstacle_keep_time)
  {
    collision_keep_time_ = collision_keep_time;
    previous_obstacle_keep_time_ = previous_obstacle_keep_time;
  }

  /**
   * @brief Get timeout values for collision and obstacle data
   * @return Pair of collision and obstacle data timeout values
   */
  std::pair<double, double> getTimeout()
  {
    return {collision_keep_time_, previous_obstacle_keep_time_};
  }

  /**
   * @brief Check if object data has expired
   * @param data Optional reference to the object data
   * @param timeout Timeout value to check against
   * @return True if object data has expired, false otherwise
   */
  bool checkObjectDataExpired(std::optional<ObjectData> & data, const double timeout)
  {
    if (!data.has_value()) return true;
    const auto now = clock_->now();
    const auto & prev_obj = data.value();
    const auto & data_time_stamp = prev_obj.stamp;
    if ((now - data_time_stamp).nanoseconds() * 1e-9 > timeout) {
      data = std::nullopt;
      return true;
    }
    return false;
  }

  /**
   * @brief Check if collision data has expired
   * @return True if collision data has expired, false otherwise
   */
  bool checkCollisionExpired()
  {
    return this->checkObjectDataExpired(closest_object_, collision_keep_time_);
  }

  /**
   * @brief Check if previous object data has expired
   * @return True if previous object data has expired, false otherwise
   */
  bool checkPreviousObjectDataExpired()
  {
    return this->checkObjectDataExpired(prev_closest_object_, previous_obstacle_keep_time_);
  }

  /**
   * @brief Get the closest object data
   * @return Object data of the closest object
   */
  [[nodiscard]] std::optional<ObjectData> get() const { return closest_object_; }

  /**
   * @brief Get the previous closest object data
   * @return Object data of the previous closest object
   */
  [[nodiscard]] ObjectData getPreviousObjectData() const
  {
    return (prev_closest_object_.has_value()) ? prev_closest_object_.value() : ObjectData();
  }

  /**
   * @brief Set collision data
   * @param data Object data to set
   */
  void setCollisionData(const ObjectData & data)
  {
    closest_object_ = std::make_optional<ObjectData>(data);
  }

  /**
   * @brief Set previous object data
   * @param data Object data to set
   */
  void setPreviousObjectData(const ObjectData & data)
  {
    prev_closest_object_ = std::make_optional<ObjectData>(data);
  }

  /**
   * @brief Reset the obstacle velocity history
   */
  void resetVelocityHistory() { obstacle_velocity_history_.clear(); }

  /**
   * @brief Update the velocity history with current object velocity
   * @param current_object_velocity Current object velocity
   * @param current_object_velocity_time_stamp Timestamp of the current object velocity
   */
  void updateVelocityHistory(
    const double current_object_velocity, const rclcpp::Time & current_object_velocity_time_stamp)
  {
    // remove old msg from deque
    const auto now = clock_->now();
    obstacle_velocity_history_.erase(
      std::remove_if(
        obstacle_velocity_history_.begin(), obstacle_velocity_history_.end(),
        [&](const auto & velocity_time_pair) {
          const auto & vel_time = velocity_time_pair.second;
          return ((now - vel_time).nanoseconds() * 1e-9 > previous_obstacle_keep_time_);
        }),
      obstacle_velocity_history_.end());
    obstacle_velocity_history_.emplace_back(
      current_object_velocity, current_object_velocity_time_stamp);
  }

  /**
   * @brief Get the median obstacle velocity from history
   * @return Optional median obstacle velocity
   */
  [[nodiscard]] std::optional<double> getMedianObstacleVelocity() const
  {
    if (obstacle_velocity_history_.empty()) return std::nullopt;
    std::vector<double> raw_velocities;
    raw_velocities.reserve(obstacle_velocity_history_.size());
    for (const auto & vel_time_pair : obstacle_velocity_history_) {
      raw_velocities.emplace_back(vel_time_pair.first);
    }

    const size_t med1 = (raw_velocities.size() % 2 == 0) ? (raw_velocities.size()) / 2 - 1
                                                         : (raw_velocities.size()) / 2;
    const size_t med2 = (raw_velocities.size()) / 2;
    std::nth_element(raw_velocities.begin(), raw_velocities.begin() + med1, raw_velocities.end());
    const double vel1 = raw_velocities.at(med1);
    std::nth_element(raw_velocities.begin(), raw_velocities.begin() + med2, raw_velocities.end());
    const double vel2 = raw_velocities.at(med2);
    return (vel1 + vel2) / 2.0;
  }

  /**
   * @brief Calculate object speed from velocity history
   * @param closest_object Closest object data
   * @param path Ego vehicle path
   * @param current_ego_speed Current ego vehicle speed
   * @return Optional calculated object speed
   */
  std::optional<double> calcObjectSpeedFromHistory(
    const ObjectData & closest_object, const Path & path, const double current_ego_speed)
  {
    // in case the object comes from predicted objects info, we reuse the speed.
    if (std::abs(closest_object.velocity) > std::numeric_limits<double>::epsilon()) {
      this->setPreviousObjectData(closest_object);
      this->updateVelocityHistory(closest_object.velocity, closest_object.stamp);
      return this->getMedianObstacleVelocity();
    }

    if (this->checkPreviousObjectDataExpired()) {
      this->setPreviousObjectData(closest_object);
      this->resetVelocityHistory();
      return std::nullopt;
    }

    const auto estimated_velocity_opt = std::invoke([&]() -> std::optional<double> {
      const auto & prev_object = this->getPreviousObjectData();
      const double p_dt =
        (closest_object.stamp.nanoseconds() - prev_object.stamp.nanoseconds()) * 1e-9;
      if (p_dt < std::numeric_limits<double>::epsilon()) return std::nullopt;
      const auto & nearest_collision_point = closest_object.position;
      const auto & prev_collision_point = prev_object.position;

      const double p_dx = nearest_collision_point.x - prev_collision_point.x;
      const double p_dy = nearest_collision_point.y - prev_collision_point.y;
      const double p_dist = std::hypot(p_dx, p_dy);
      const double p_yaw = std::atan2(p_dy, p_dx);
      const double p_vel = p_dist / p_dt;

      const auto nearest_idx =
        autoware::motion_utils::findNearestIndex(path, nearest_collision_point);
      const auto & nearest_path_pose = path.at(nearest_idx);
      // When the ego moves backwards, the direction of movement axis is reversed
      const auto & traj_yaw = (current_ego_speed > 0.0)
                                ? tf2::getYaw(nearest_path_pose.orientation)
                                : tf2::getYaw(nearest_path_pose.orientation) + M_PI;
      const auto estimated_velocity =
        p_vel * std::cos(p_yaw - traj_yaw) + std::abs(current_ego_speed);

      // Current RSS distance calculation does not account for negative velocities
      return estimated_velocity;
    });

    if (!estimated_velocity_opt.has_value()) {
      return std::nullopt;
    }

    const auto & estimated_velocity = estimated_velocity_opt.value();
    this->setPreviousObjectData(closest_object);
    this->updateVelocityHistory(estimated_velocity, closest_object.stamp);
    return this->getMedianObstacleVelocity();
  }

private:
  std::optional<ObjectData> prev_closest_object_{std::nullopt};
  std::optional<ObjectData> closest_object_{std::nullopt};
  double collision_keep_time_{0.0};
  double previous_obstacle_keep_time_{0.0};

  std::deque<std::pair<double, rclcpp::Time>> obstacle_velocity_history_;
  rclcpp::Clock::SharedPtr clock_;
};

/**
 * @brief Autonomous Emergency Braking (AEB) node
 */
class AEB : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for AEB
   * @param node_options Options for the node
   */
  explicit AEB(const rclcpp::NodeOptions & node_options);

  // subscriber
  autoware::universe_utils::InterProcessPollingSubscriber<PointCloud2> sub_point_cloud_{
    this, "~/input/pointcloud", autoware::universe_utils::SingleDepthSensorQoS()};
  autoware::universe_utils::InterProcessPollingSubscriber<VelocityReport> sub_velocity_{
    this, "~/input/velocity"};
  autoware::universe_utils::InterProcessPollingSubscriber<Imu> sub_imu_{this, "~/input/imu"};
  autoware::universe_utils::InterProcessPollingSubscriber<Trajectory> sub_predicted_traj_{
    this, "~/input/predicted_trajectory"};
  autoware::universe_utils::InterProcessPollingSubscriber<PredictedObjects> predicted_objects_sub_{
    this, "~/input/objects"};
  autoware::universe_utils::InterProcessPollingSubscriber<AutowareState> sub_autoware_state_{
    this, "/autoware/state"};
  // publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacle_pointcloud_;
  rclcpp::Publisher<MarkerArray>::SharedPtr debug_marker_publisher_;
  rclcpp::Publisher<MarkerArray>::SharedPtr virtual_wall_publisher_;
  rclcpp::Publisher<autoware::universe_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  rclcpp::Publisher<tier4_debug_msgs::msg::Float32Stamped>::SharedPtr debug_rss_distance_publisher_;
  rclcpp::Publisher<MetricArray>::SharedPtr metrics_pub_;
  // timer
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::shared_ptr<autoware::universe_utils::TimeKeeper> time_keeper_{nullptr};

  // callback
  /**
   * @brief Callback for point cloud messages
   * @param input_msg Shared pointer to the point cloud message
   */
  void onPointCloud(const PointCloud2::ConstSharedPtr input_msg);

  /**
   * @brief Callback for IMU messages
   * @param input_msg Shared pointer to the IMU message
   */
  void onImu(const Imu::ConstSharedPtr input_msg);

  /**
   * @brief Timer callback function
   */
  void onTimer();

  /**
   * @brief Callback for parameter updates
   * @param parameters Vector of updated parameters
   * @return Set parameters result
   */
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Fetch the latest data from subscribers
   * @return True if data fetch was successful, false otherwise
   */
  bool fetchLatestData();

  /**
   * @brief Diagnostic check for collisions
   * @param stat Diagnostic status wrapper
   */
  void onCheckCollision(DiagnosticStatusWrapper & stat);

  /**
   * @brief Check for collisions
   * @param debug_markers Marker array for debugging
   * @return True if a collision is detected, false otherwise
   */
  bool checkCollision(MarkerArray & debug_markers);

  /**
   * @brief Check if there is a collision with the closest object
   * @param current_v Current velocity of the ego vehicle
   * @param closest_object Data of the closest object
   * @return True if a collision is detected, false otherwise
   */
  bool hasCollision(const double current_v, const ObjectData & closest_object);

  /**
   * @brief Generate the ego vehicle path
   * @param curr_v Current velocity of the ego vehicle
   * @param curr_w Current angular velocity of the ego vehicle
   * @return Generated ego path
   */
  Path generateEgoPath(const double curr_v, const double curr_w);

  /**
   * @brief Generate the ego vehicle path from the predicted trajectory
   * @param predicted_traj Predicted trajectory of the ego vehicle
   * @return Optional generated ego path
   */
  std::optional<Path> generateEgoPath(const Trajectory & predicted_traj);

  /**
   * @brief Generate the footprint of the path with extra width margin
   * @param path Ego vehicle path
   * @param extra_width_margin Extra width margin for the footprint
   * @param polygons vector to be filled with the polygons
   * @return Vector of polygons representing the path footprint
   */
  void generatePathFootprint(
    const Path & path, const double extra_width_margin, std::vector<Polygon2d> & polygons);

  /**
   * @brief Generate the footprint of the path with extra width margin
   * @param path Ego vehicle path
   * @param extra_width_margin Extra width margin for the footprint
   * @return Vector of polygons representing the path footprint
   */
  std::vector<Polygon2d> generatePathFootprint(const Path & path, const double extra_width_margin);

  /**
   * @brief Create object data using point cloud clusters
   * @param ego_path Ego vehicle path
   * @param ego_polys Polygons representing the ego vehicle footprint
   * @param speed_calc_ego_polys Polygons representing the expanded ego vehicle footprint for speed
   * calculation area
   * @param stamp Timestamp of the data
   * @param objects Vector to store the created object data
   * @param obstacle_points_ptr Pointer to the point cloud of obstacles
   */
  void getClosestObjectsOnPath(
    const Path & ego_path, const rclcpp::Time & stamp,
    const PointCloud::Ptr points_belonging_to_cluster_hulls, std::vector<ObjectData> & objects);

  /**
   * @brief Create object data using point cloud clusters
   * @param obstacle_points_ptr Pointer to the point cloud of obstacles
   * @param points_belonging_to_cluster_hulls output: pointer to the point cloud of points belonging
   * to cluster hulls
   */
  void getPointsBelongingToClusterHulls(
    const PointCloud::Ptr obstacle_points_ptr,
    const PointCloud::Ptr points_belonging_to_cluster_hulls, MarkerArray & debug_markers);

  /**
   * @brief Create object data using predicted objects
   * @param ego_path Ego vehicle path
   * @param ego_polys Polygons representing the ego vehicle footprint
   * @param objects Vector to store the created object data
   */
  void createObjectDataUsingPredictedObjects(
    const Path & ego_path, const std::vector<Polygon2d> & ego_polys,
    std::vector<ObjectData> & objects);

  /**
   * @brief Crop the point cloud with the ego vehicle footprint path
   * @param ego_polys Polygons representing the ego vehicle footprint
   * @param filtered_objects Pointer to the filtered point cloud of obstacles
   */
  void cropPointCloudWithEgoFootprintPath(
    const std::vector<Polygon2d> & ego_polys, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_objects);

  /**
   * @brief Add a marker for debugging
   * @param current_time Current time
   * @param path Ego vehicle path
   * @param polygons Polygons representing the ego vehicle footprint
   * @param objects Vector of object data
   * @param closest_object Optional data of the closest object
   * @param debug_colors Tuple of RGBA colors
   * @param ns Namespace for the marker
   * @param debug_markers Marker array for debugging
   */
  void addMarker(
    const rclcpp::Time & current_time, const Path & path, const std::vector<Polygon2d> & polygons,
    const std::vector<ObjectData> & objects, const std::optional<ObjectData> & closest_object,
    const colorTuple & debug_colors, const std::string & ns, MarkerArray & debug_markers);

  /**
   * @brief Add a marker of convex hulls for debugging
   * @param current_time Current time
   * @param hulls vector of polygons of the convex hulls
   * @param debug_colors Tuple of RGBA colors
   * @param ns Namespace for the marker
   * @param debug_markers Marker array for debugging
   */
  void addClusterHullMarkers(
    const rclcpp::Time & current_time, const std::vector<Polygon3d> & hulls,
    const colorTuple & debug_colors, const std::string & ns, MarkerArray & debug_markers);

  /**
   * @brief Add a collision marker for debugging
   * @param data Data of the collision object
   * @param debug_markers Marker array for debugging
   */
  void addCollisionMarker(const ObjectData & data, MarkerArray & debug_markers);

  /**
   * @brief Add an info marker stop wall in front of the ego vehicle
   * @param markers Data of the closest object
   */
  void addVirtualStopWallMarker(MarkerArray & markers);

  /**
   * @brief Calculate object speed from history
   * @param closest_object Data of the closest object
   * @param path Ego vehicle path
   * @param current_ego_speed Current speed of the ego vehicle
   * @return Optional calculated object speed
   */
  std::optional<double> calcObjectSpeedFromHistory(
    const ObjectData & closest_object, const Path & path, const double current_ego_speed);

  // Member variables
  PointCloud2::SharedPtr obstacle_ros_pointcloud_ptr_{nullptr};
  VelocityReport::ConstSharedPtr current_velocity_ptr_{nullptr};
  Vector3::SharedPtr angular_velocity_ptr_{nullptr};
  Trajectory::ConstSharedPtr predicted_traj_ptr_{nullptr};
  PredictedObjects::ConstSharedPtr predicted_objects_ptr_{nullptr};
  AutowareState::ConstSharedPtr autoware_state_{nullptr};

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  // vehicle info
  VehicleInfo vehicle_info_;

  // diag
  Updater updater_{this};

  // Member variables
  bool publish_debug_pointcloud_;
  bool publish_debug_markers_;
  bool use_predicted_trajectory_;
  bool use_imu_path_;
  bool limit_imu_path_lat_dev_;
  bool limit_imu_path_length_;
  bool use_pointcloud_data_;
  bool use_predicted_object_data_;
  bool use_object_velocity_calculation_;
  bool check_autoware_state_;
  double imu_path_lat_dev_threshold_;
  double path_footprint_extra_margin_;
  double speed_calculation_expansion_margin_;
  double detection_range_min_height_;
  double detection_range_max_height_margin_;
  double voxel_grid_x_;
  double voxel_grid_y_;
  double voxel_grid_z_;
  double min_generated_imu_path_length_;
  double max_generated_imu_path_length_;
  double expand_width_;
  double longitudinal_offset_margin_;
  double t_response_;
  double a_ego_min_;
  double a_obj_min_;
  double cluster_tolerance_;
  double cluster_minimum_height_;
  int minimum_cluster_size_;
  int maximum_cluster_size_;
  double imu_prediction_time_horizon_;
  double imu_prediction_time_interval_;
  double mpc_prediction_time_horizon_;
  double mpc_prediction_time_interval_;
  CollisionDataKeeper collision_data_keeper_;
  // Parameter callback
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
};
}  // namespace autoware::motion::control::autonomous_emergency_braking

#endif  // AUTOWARE__AUTONOMOUS_EMERGENCY_BRAKING__NODE_HPP_
