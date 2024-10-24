// Copyright 2022 Tier IV, Inc.
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

#include "autoware/planning_validator/planning_validator.hpp"

#include "autoware/planning_validator/utils.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>

#include <boost/geometry/algorithms/intersects.hpp>

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

namespace autoware::planning_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

PlanningValidator::PlanningValidator(const rclcpp::NodeOptions & options)
: Node("planning_validator", options)
{
  using std::placeholders::_1;

  sub_traj_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&PlanningValidator::onTrajectory, this, _1));

  pub_traj_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_status_ = create_publisher<PlanningValidatorStatus>("~/output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);
  pub_processing_time_ms_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  debug_pose_publisher_ = std::make_shared<PlanningValidatorDebugMarkerPublisher>(this);

  setupParameters();

  logger_configure_ = std::make_unique<autoware::universe_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ =
    std::make_unique<autoware::universe_utils::PublishedTimePublisher>(this);
}

void PlanningValidator::setupParameters()
{
  const auto type = declare_parameter<int>("invalid_trajectory_handling_type");
  if (type == 0) {
    invalid_trajectory_handling_type_ = InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS;
  } else if (type == 1) {
    invalid_trajectory_handling_type_ = InvalidTrajectoryHandlingType::STOP_PUBLISHING;
  } else if (type == 2) {
    invalid_trajectory_handling_type_ = InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT;
  } else {
    throw std::invalid_argument{
      "unsupported invalid_trajectory_handling_type (" + std::to_string(type) + ")"};
  }
  publish_diag_ = declare_parameter<bool>("publish_diag");
  diag_error_count_threshold_ = declare_parameter<int>("diag_error_count_threshold");
  display_on_terminal_ = declare_parameter<bool>("display_on_terminal");

  {
    auto & p = validation_params_;
    const std::string t = "thresholds.";
    p.interval_threshold = declare_parameter<double>(t + "interval");
    p.relative_angle_threshold = declare_parameter<double>(t + "relative_angle");
    p.curvature_threshold = declare_parameter<double>(t + "curvature");
    p.lateral_acc_threshold = declare_parameter<double>(t + "lateral_acc");
    p.longitudinal_max_acc_threshold = declare_parameter<double>(t + "longitudinal_max_acc");
    p.longitudinal_min_acc_threshold = declare_parameter<double>(t + "longitudinal_min_acc");
    p.steering_threshold = declare_parameter<double>(t + "steering");
    p.steering_rate_threshold = declare_parameter<double>(t + "steering_rate");
    p.velocity_deviation_threshold = declare_parameter<double>(t + "velocity_deviation");
    p.distance_deviation_threshold = declare_parameter<double>(t + "distance_deviation");
    p.longitudinal_distance_deviation_threshold =
      declare_parameter<double>(t + "longitudinal_distance_deviation");

    const std::string ps = "parameters.";
    p.forward_trajectory_length_acceleration =
      declare_parameter<double>(ps + "forward_trajectory_length_acceleration");
    p.forward_trajectory_length_margin =
      declare_parameter<double>(ps + "forward_trajectory_length_margin");
  }

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to get vehicle info. use default value.");
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
  }
}

void PlanningValidator::setStatus(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg)
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
  } else if (validation_status_.invalid_count < diag_error_count_threshold_) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(validation_status_.invalid_count) + " < " +
                          std::to_string(diag_error_count_threshold_) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void PlanningValidator::setupDiag()
{
  diag_updater_ = std::make_shared<Updater>(this);
  auto & d = diag_updater_;
  d->setHardwareID("planning_validator");

  std::string ns = "trajectory_validation_";
  d->add(ns + "size", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_size, "invalid trajectory size is found");
  });
  d->add(ns + "finite", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_finite_value, "infinite value is found");
  });
  d->add(ns + "interval", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_interval, "points interval is too long");
  });
  d->add(ns + "relative_angle", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_relative_angle, "relative angle is too large");
  });
  d->add(ns + "curvature", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_curvature, "curvature is too large");
  });
  d->add(ns + "lateral_acceleration", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_lateral_acc, "lateral acceleration is too large");
  });
  d->add(ns + "acceleration", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_longitudinal_max_acc, "acceleration is too large");
  });
  d->add(ns + "deceleration", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_longitudinal_min_acc, "deceleration is too large");
  });
  d->add(ns + "steering", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_steering, "expected steering is too large");
  });
  d->add(ns + "steering_rate", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_steering_rate, "expected steering rate is too large");
  });
  d->add(ns + "velocity_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_velocity_deviation, "velocity deviation is too large");
  });
  d->add(ns + "distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_distance_deviation, "distance deviation is too large");
  });
  d->add(ns + "longitudinal_distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_longitudinal_distance_deviation,
      "longitudinal distance deviation is too large");
  });
  d->add(ns + "forward_trajectory_length", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_forward_trajectory_length,
      "trajectory length is too short");
  });
  d->add(ns + "trajectory_collision", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_no_collision, "collision is detected");
  });
}

bool PlanningValidator::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s);
    return false;
  };

  if (!current_kinematics_) {
    return waiting("current_kinematics_");
  }
  if (!current_objects_) {
    return waiting("current_objects_");
  }
  if (!current_trajectory_) {
    return waiting("current_trajectory_");
  }
  return true;
}

void PlanningValidator::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  stop_watch_.tic(__func__);

  current_trajectory_ = msg;

  // receive data
  current_kinematics_ = sub_kinematics_.takeData();
  current_objects_ = sub_obj_.takeData();

  if (!isDataReady()) return;

  if (publish_diag_ && !diag_updater_) {
    setupDiag();  // run setup after all data is ready.
  }

  debug_pose_publisher_->clearMarkers();

  validate(*current_trajectory_);

  diag_updater_->force_update();

  publishTrajectory();

  // for debug
  publishProcessingTime(stop_watch_.toc(__func__));
  publishDebugInfo();
  displayStatus();
}

void PlanningValidator::publishTrajectory()
{
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(validation_status_)) {
    pub_traj_->publish(*current_trajectory_);
    published_time_publisher_->publish_if_subscribed(pub_traj_, current_trajectory_->header.stamp);
    previous_published_trajectory_ = current_trajectory_;
    return;
  }

  //  ----- invalid factor is found. Publish previous trajectory. -----

  if (invalid_trajectory_handling_type_ == InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS) {
    pub_traj_->publish(*current_trajectory_);
    published_time_publisher_->publish_if_subscribed(pub_traj_, current_trajectory_->header.stamp);
    RCLCPP_ERROR(get_logger(), "Caution! Invalid Trajectory published.");
    return;
  }

  if (invalid_trajectory_handling_type_ == InvalidTrajectoryHandlingType::STOP_PUBLISHING) {
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Trajectory is not published.");
    return;
  }

  if (invalid_trajectory_handling_type_ == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
    if (previous_published_trajectory_) {
      pub_traj_->publish(*previous_published_trajectory_);
      published_time_publisher_->publish_if_subscribed(
        pub_traj_, previous_published_trajectory_->header.stamp);
      RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use previous trajectory.");
      return;
    }
  }

  // trajectory is not published.
  RCLCPP_ERROR(
    get_logger(),
    "Invalid Trajectory detected, no valid trajectory found in the past. Trajectory is not "
    "published.");
  return;
}

void PlanningValidator::publishProcessingTime(const double processing_time_ms)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = processing_time_ms;
  pub_processing_time_ms_->publish(msg);
}

void PlanningValidator::publishDebugInfo()
{
  validation_status_.stamp = get_clock()->now();
  pub_status_->publish(validation_status_);

  if (!isAllValid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = current_kinematics_->pose.pose;
    shiftPose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->pushVirtualWall(front_pose);
    debug_pose_publisher_->pushWarningMsg(front_pose, "INVALID PLANNING");
  }
  debug_pose_publisher_->publish();
}

void PlanningValidator::validate(const Trajectory & trajectory)
{
  auto & s = validation_status_;

  const auto terminateValidation = [&](const auto & ss) {
    RCLCPP_ERROR_STREAM(get_logger(), ss);
    s.invalid_count += 1;
  };

  s.is_valid_size = checkValidSize(trajectory);
  if (!s.is_valid_size) {
    return terminateValidation(
      "trajectory has invalid point size (" + std::to_string(trajectory.points.size()) +
      "). Stop validation process, raise an error.");
  }

  s.is_valid_finite_value = checkValidFiniteValue(trajectory);
  if (!s.is_valid_finite_value) {
    return terminateValidation(
      "trajectory has invalid value (NaN, Inf, etc). Stop validation process, raise an error.");
  }

  s.is_valid_interval = checkValidInterval(trajectory);
  s.is_valid_longitudinal_max_acc = checkValidMaxLongitudinalAcceleration(trajectory);
  s.is_valid_longitudinal_min_acc = checkValidMinLongitudinalAcceleration(trajectory);
  s.is_valid_velocity_deviation = checkValidVelocityDeviation(trajectory);
  s.is_valid_distance_deviation = checkValidDistanceDeviation(trajectory);
  s.is_valid_longitudinal_distance_deviation = checkValidLongitudinalDistanceDeviation(trajectory);
  s.is_valid_forward_trajectory_length = checkValidForwardTrajectoryLength(trajectory);

  // use resampled trajectory because the following metrics can not be evaluated for closed points.
  // Note: do not interpolate to keep original trajectory shape.
  constexpr auto min_interval = 1.0;
  const auto resampled = resampleTrajectory(trajectory, min_interval);

  s.is_valid_relative_angle = checkValidRelativeAngle(resampled);
  s.is_valid_curvature = checkValidCurvature(resampled);
  s.is_valid_lateral_acc = checkValidLateralAcceleration(resampled);
  s.is_valid_steering = checkValidSteering(resampled);
  s.is_valid_steering_rate = checkValidSteeringRate(resampled);
  s.is_valid_no_collision = checkValidTrajectoryCollision(resampled);

  s.invalid_count = isAllValid(s) ? 0 : s.invalid_count + 1;
}

bool PlanningValidator::checkValidSize(const Trajectory & trajectory)
{
  validation_status_.trajectory_size = trajectory.points.size();
  return trajectory.points.size() >= 2;
}

bool PlanningValidator::checkValidFiniteValue(const Trajectory & trajectory)
{
  for (const auto & p : trajectory.points) {
    if (!checkFinite(p)) return false;
  }
  return true;
}

bool PlanningValidator::checkValidInterval(const Trajectory & trajectory)
{
  const auto [max_interval_distance, i] = calcMaxIntervalDistance(trajectory);
  validation_status_.max_interval_distance = max_interval_distance;

  if (max_interval_distance > validation_params_.interval_threshold) {
    if (i > 0) {
      const auto & p = trajectory.points;
      debug_pose_publisher_->pushPoseMarker(p.at(i - 1), "trajectory_interval");
      debug_pose_publisher_->pushPoseMarker(p.at(i), "trajectory_interval");
    }
    return false;
  }

  return true;
}

bool PlanningValidator::checkValidRelativeAngle(const Trajectory & trajectory)
{
  const auto [max_relative_angle, i] = calcMaxRelativeAngles(trajectory);
  validation_status_.max_relative_angle = max_relative_angle;

  if (max_relative_angle > validation_params_.relative_angle_threshold) {
    const auto & p = trajectory.points;
    if (i < p.size() - 3) {
      debug_pose_publisher_->pushPoseMarker(p.at(i), "trajectory_relative_angle", 0);
      debug_pose_publisher_->pushPoseMarker(p.at(i + 1), "trajectory_relative_angle", 1);
      debug_pose_publisher_->pushPoseMarker(p.at(i + 2), "trajectory_relative_angle", 2);
    }
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidCurvature(const Trajectory & trajectory)
{
  const auto [max_curvature, i] = calcMaxCurvature(trajectory);
  validation_status_.max_curvature = max_curvature;
  if (max_curvature > validation_params_.curvature_threshold) {
    const auto & p = trajectory.points;
    if (i > 0 && i < p.size() - 1) {
      debug_pose_publisher_->pushPoseMarker(p.at(i - 1), "trajectory_curvature");
      debug_pose_publisher_->pushPoseMarker(p.at(i), "trajectory_curvature");
      debug_pose_publisher_->pushPoseMarker(p.at(i + 1), "trajectory_curvature");
    }
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidLateralAcceleration(const Trajectory & trajectory)
{
  const auto [max_lateral_acc, i] = calcMaxLateralAcceleration(trajectory);
  validation_status_.max_lateral_acc = max_lateral_acc;
  if (max_lateral_acc > validation_params_.lateral_acc_threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i), "lateral_acceleration");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidMinLongitudinalAcceleration(const Trajectory & trajectory)
{
  const auto [min_longitudinal_acc, i] = getMinLongitudinalAcc(trajectory);
  validation_status_.min_longitudinal_acc = min_longitudinal_acc;

  if (min_longitudinal_acc < validation_params_.longitudinal_min_acc_threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "min_longitudinal_acc");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidMaxLongitudinalAcceleration(const Trajectory & trajectory)
{
  const auto [max_longitudinal_acc, i] = getMaxLongitudinalAcc(trajectory);
  validation_status_.max_longitudinal_acc = max_longitudinal_acc;

  if (max_longitudinal_acc > validation_params_.longitudinal_max_acc_threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "max_longitudinal_acc");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidSteering(const Trajectory & trajectory)
{
  const auto [max_steering, i] = calcMaxSteeringAngles(trajectory, vehicle_info_.wheel_base_m);
  validation_status_.max_steering = max_steering;

  if (max_steering > validation_params_.steering_threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "max_steering");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidSteeringRate(const Trajectory & trajectory)
{
  const auto [max_steering_rate, i] = calcMaxSteeringRates(trajectory, vehicle_info_.wheel_base_m);
  validation_status_.max_steering_rate = max_steering_rate;

  if (max_steering_rate > validation_params_.steering_rate_threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "max_steering_rate");
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidVelocityDeviation(const Trajectory & trajectory)
{
  // TODO(horibe): set appropriate thresholds for index search
  const auto idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, current_kinematics_->pose.pose);

  validation_status_.velocity_deviation = std::abs(
    trajectory.points.at(idx).longitudinal_velocity_mps -
    current_kinematics_->twist.twist.linear.x);

  if (validation_status_.velocity_deviation > validation_params_.velocity_deviation_threshold) {
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidDistanceDeviation(const Trajectory & trajectory)
{
  // TODO(horibe): set appropriate thresholds for index search
  const auto idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, current_kinematics_->pose.pose);

  validation_status_.distance_deviation = autoware::universe_utils::calcDistance2d(
    trajectory.points.at(idx), current_kinematics_->pose.pose);

  if (validation_status_.distance_deviation > validation_params_.distance_deviation_threshold) {
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidLongitudinalDistanceDeviation(const Trajectory & trajectory)
{
  if (trajectory.points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "Trajectory size is invalid to calculate distance deviation.");
    return false;
  }

  const auto ego_pose = current_kinematics_->pose.pose;
  const size_t idx =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(trajectory.points, ego_pose);

  if (0 < idx && idx < trajectory.points.size() - 1) {
    return true;  // ego-nearest point exists between trajectory points.
  }

  // Check if the valid longitudinal deviation for given segment index
  const auto HasValidLongitudinalDeviation = [&](const size_t seg_idx, const bool is_last) {
    auto long_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      trajectory.points, seg_idx, ego_pose.position);

    // for last, need to remove distance for the last segment.
    if (is_last) {
      const auto size = trajectory.points.size();
      long_offset -= autoware::universe_utils::calcDistance2d(
        trajectory.points.at(size - 1), trajectory.points.at(size - 2));
    }

    validation_status_.longitudinal_distance_deviation = long_offset;
    return std::abs(validation_status_.longitudinal_distance_deviation) <
           validation_params_.longitudinal_distance_deviation_threshold;
  };

  // Make sure the trajectory is far AHEAD from ego.
  if (idx == 0) {
    const auto seg_idx = 0;
    return HasValidLongitudinalDeviation(seg_idx, false);
  }

  // Make sure the trajectory is far BEHIND from ego.
  if (idx == trajectory.points.size() - 1) {
    const auto seg_idx = trajectory.points.size() - 2;
    return HasValidLongitudinalDeviation(seg_idx, true);
  }

  return true;
}

bool PlanningValidator::checkValidForwardTrajectoryLength(const Trajectory & trajectory)
{
  const auto ego_speed = std::abs(current_kinematics_->twist.twist.linear.x);
  if (ego_speed < 1.0 / 3.6) {
    return true;  // Ego is almost stopped.
  }

  const auto forward_length = autoware::motion_utils::calcSignedArcLength(
    trajectory.points, current_kinematics_->pose.pose.position, trajectory.points.size() - 1);

  const auto acc = validation_params_.forward_trajectory_length_acceleration;
  const auto forward_length_required = ego_speed * ego_speed / (2.0 * std::abs(acc)) -
                                       validation_params_.forward_trajectory_length_margin;

  validation_status_.forward_trajectory_length_required = forward_length_required;
  validation_status_.forward_trajectory_length_measured = forward_length;

  return forward_length > forward_length_required;
}

bool PlanningValidator::checkValidTrajectoryCollision(const Trajectory & trajectory)
{
  const auto ego_speed = std::abs(current_kinematics_->twist.twist.linear.x);
  if (ego_speed < 1.0 / 3.6) {
    return true;  // Ego is almost stopped.
  }

  const bool is_collision = checkCollision(
    *current_objects_, trajectory, current_kinematics_->pose.pose.position, vehicle_info_);
  return is_collision;
}

// TODO(Sugahara): move to utils
bool PlanningValidator::checkCollision(
  const PredictedObjects & predicted_objects, const Trajectory & trajectory,
  const geometry_msgs::msg::Point & current_ego_position, const VehicleInfo & vehicle_info,
  const double collision_check_distance_threshold)
{
  // TODO(Sugahara): Detect collision if collision is detected in consecutive frames

  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> resampled_trajectory;
  resampled_trajectory.reserve(trajectory.points.size());
  size_t index = 0;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points[i];
    const double dist_to_point = autoware::motion_utils::calcSignedArcLength(
      trajectory.points, current_ego_position, size_t(i));

    if (dist_to_point > 0.0) {
      resampled_trajectory.push_back(point);
      index = i;
      break;
    }
  }

  constexpr double min_interval = 0.5;
  for (size_t i = index + 1; i < trajectory.points.size(); ++i) {
    const auto & current_point = trajectory.points[i];
    if (current_point.longitudinal_velocity_mps < 0.1) {
      continue;
    }
    const double distance_to_previous_point =
      universe_utils::calcDistance2d(resampled_trajectory.back(), current_point);
    if (distance_to_previous_point > min_interval) {
      resampled_trajectory.push_back(current_point);
    }
  }
  motion_utils::calculate_time_from_start(resampled_trajectory, current_ego_position);

  // generate vehicle footprint along the trajectory
  std::vector<Polygon2d> vehicle_footprints;
  vehicle_footprints.reserve(resampled_trajectory.size());
  for (const auto & point : resampled_trajectory) {
    vehicle_footprints.push_back(createVehicleFootprintPolygon(point.pose, vehicle_info));
  }

  const double time_tolerance = 0.1;  // time tolerance threshold
  for (const auto & object : predicted_objects.objects) {
    // Calculate distance between object position and nearest point on trajectory
    const auto & object_position = object.kinematics.initial_pose_with_covariance.pose.position;
    const size_t nearest_index =
      autoware::motion_utils::findNearestIndex(resampled_trajectory, object_position);
    const double object_distance_to_nearest_point = autoware::universe_utils::calcDistance2d(
      resampled_trajectory[nearest_index], object_position);

    // Skip collision check if object is too far from trajectory
    if (object_distance_to_nearest_point > collision_check_distance_threshold) {
      continue;
    }

    // Select path with highest confidence from object's predicted paths
    const auto selected_predicted_path =
      [&object]() -> const autoware_perception_msgs::msg::PredictedPath * {
      const auto max_confidence_it = std::max_element(
        object.kinematics.predicted_paths.begin(), object.kinematics.predicted_paths.end(),
        [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

      return max_confidence_it != object.kinematics.predicted_paths.end() ? &(*max_confidence_it)
                                                                          : nullptr;
    }();

    if (!selected_predicted_path) {
      continue;
    }

    // Generate polygons for predicted object positions
    std::vector<Polygon2d> predicted_object_polygons;
    predicted_object_polygons.reserve(selected_predicted_path->path.size());

    const double predicted_time_step =
      selected_predicted_path->time_step.sec + selected_predicted_path->time_step.nanosec * 1e-9;

    for (const auto & pose : selected_predicted_path->path) {
      predicted_object_polygons.push_back(
        autoware::universe_utils::toPolygon2d(pose, object.shape));
    }

    // Check for collision (considering time)
    for (size_t i = 0; i < resampled_trajectory.size(); ++i) {
      const auto & trajectory_point = resampled_trajectory[i];
      const double trajectory_time =
        trajectory_point.time_from_start.sec + trajectory_point.time_from_start.nanosec * 1e-9;

      for (size_t j = 0; j < selected_predicted_path->path.size(); ++j) {
        const double predicted_time = j * predicted_time_step;

        if (std::fabs(trajectory_time - predicted_time) > time_tolerance) {
          continue;
        }

        const double distance = autoware::universe_utils::calcDistance2d(
          trajectory_point.pose.position, selected_predicted_path->path[j].position);

        if (distance >= 10.0) {
          continue;
        }

        if (boost::geometry::intersects(vehicle_footprints[i], predicted_object_polygons[j])) {
          // Collision detected
          std::cerr << "Collision detected at " << std::endl;
          std::cerr << "  trajectory_time: " << trajectory_time << std::endl;
          std::cerr << "  predicted_time: " << predicted_time << std::endl;
          std::cerr << "object velocity: "
                    << object.kinematics.initial_twist_with_covariance.twist.linear.x << "m/s"
                    << std::endl;
          return false;
        }
      }
    }
  }

  return true;
}

// TODO(Sugahara): move to utils
Polygon2d PlanningValidator::createVehicleFootprintPolygon(
  const geometry_msgs::msg::Pose & pose, const VehicleInfo & vehicle_info)
{
  using autoware::universe_utils::Point2d;
  const double length = vehicle_info.vehicle_length_m;
  const double width = vehicle_info.vehicle_width_m;
  const double rear_overhang = vehicle_info.rear_overhang_m;
  const double yaw = tf2::getYaw(pose.orientation);

  // Calculate relative positions of vehicle corners
  std::vector<Point2d> footprint_points{
    Point2d(length - rear_overhang, width / 2.0), Point2d(length - rear_overhang, -width / 2.0),
    Point2d(-rear_overhang, -width / 2.0), Point2d(-rear_overhang, width / 2.0)};

  Polygon2d footprint_polygon;
  footprint_polygon.outer().reserve(footprint_points.size() + 1);

  for (const auto & point : footprint_points) {
    Point2d transformed_point;
    transformed_point.x() = pose.position.x + point.x() * std::cos(yaw) - point.y() * std::sin(yaw);
    transformed_point.y() = pose.position.y + point.x() * std::sin(yaw) + point.y() * std::cos(yaw);
    footprint_polygon.outer().push_back(transformed_point);
  }

  footprint_polygon.outer().push_back(footprint_polygon.outer().front());

  return footprint_polygon;
}

bool PlanningValidator::isAllValid(const PlanningValidatorStatus & s) const
{
  // TODO(Sugahara): Add s.is_valid_no_collision after verifying that:
  // 1. The false value of is_valid_no_collision correctly identifies path problems
  // 2. Adding this check won't incorrectly invalidate otherwise valid paths
  // want to avoid false negatives where good paths are marked invalid just because
  // isAllValid becomes false
  return s.is_valid_size && s.is_valid_finite_value && s.is_valid_interval &&
         s.is_valid_relative_angle && s.is_valid_curvature && s.is_valid_lateral_acc &&
         s.is_valid_longitudinal_max_acc && s.is_valid_longitudinal_min_acc &&
         s.is_valid_steering && s.is_valid_steering_rate && s.is_valid_velocity_deviation &&
         s.is_valid_distance_deviation && s.is_valid_longitudinal_distance_deviation &&
         s.is_valid_forward_trajectory_length;
}

void PlanningValidator::displayStatus()
{
  if (!display_on_terminal_) return;

  const auto warn = [this](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(s.is_valid_size, "planning trajectory size is invalid, too small.");
  warn(s.is_valid_curvature, "planning trajectory curvature is too large!!");
  warn(s.is_valid_finite_value, "planning trajectory has invalid value!!");
  warn(s.is_valid_interval, "planning trajectory interval is too long!!");
  warn(s.is_valid_lateral_acc, "planning trajectory lateral acceleration is too high!!");
  warn(s.is_valid_longitudinal_max_acc, "planning trajectory acceleration is too high!!");
  warn(s.is_valid_longitudinal_min_acc, "planning trajectory deceleration is too high!!");
  warn(s.is_valid_relative_angle, "planning trajectory yaw angle varies too fast!!");
  warn(s.is_valid_steering, "planning trajectory expected steering angle is too high!!");
  warn(s.is_valid_steering_rate, "planning trajectory expected steering angle rate is too high!!");
  warn(s.is_valid_velocity_deviation, "planning trajectory velocity deviation is too high!!");
  warn(s.is_valid_distance_deviation, "planning trajectory is too far from ego!!");
  warn(
    s.is_valid_longitudinal_distance_deviation,
    "planning trajectory is too far from ego in longitudinal direction!!");
  warn(s.is_valid_forward_trajectory_length, "planning trajectory forward length is not enough!!");
  warn(
    s.is_valid_no_collision,
    "planning trajectory has collision!! but this validation is not utilized for trajectory "
    "validation.");
}

}  // namespace autoware::planning_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_validator::PlanningValidator)
