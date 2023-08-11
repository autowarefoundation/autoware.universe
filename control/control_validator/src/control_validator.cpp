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

#include "control_validator/control_validator.hpp"

#include "control_validator/utils.hpp"

#include <motion_utils/motion_utils.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>
#include <utility>

namespace control_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;
using geometry_msgs::msg::Pose;

ControlValidator::ControlValidator(const rclcpp::NodeOptions & options)
: Node("control_validator", options)
{
  using std::placeholders::_1;

  sub_kinematics_ = create_subscription<Odometry>(
    "~/input/kinematics", 1,
    [this](const Odometry::ConstSharedPtr msg) { current_kinematics_ = msg; });
  sub_reference_traj_ = create_subscription<Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&ControlValidator::onReferenceTrajectory, this, _1));
  sub_predicted_traj_ = create_subscription<Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&ControlValidator::onPredictedTrajectory, this, _1));

  pub_traj_ = create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  pub_status_ = create_publisher<ControlValidatorStatus>("~/output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);

  debug_pose_publisher_ = std::make_shared<ControlValidatorDebugMarkerPublisher>(this);

  setupParameters();

  if (publish_diag_) {
    setupDiag();
  }
}

void ControlValidator::setupParameters()
{
  const auto type = declare_parameter<int>("invalid_trajectory_handling_type");
  if (type == 0) {
    invalid_predicted_trajectory_handling_type_ =
      InvalidPredictedTrajectoryHandlingType::PUBLISH_AS_IT_IS;
  } else if (type == 1) {
    invalid_predicted_trajectory_handling_type_ =
      InvalidPredictedTrajectoryHandlingType::STOP_PUBLISHING;
  } else if (type == 2) {
    invalid_predicted_trajectory_handling_type_ =
      InvalidPredictedTrajectoryHandlingType::USE_PREVIOUS_RESULT;
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
    p.max_distance_deviation_threshold = declare_parameter<double>(t + "max_distance_deviation");
  }

  try {
    vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to get vehicle info. use default value.");
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
  }
}

void ControlValidator::setStatus(
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

void ControlValidator::setupDiag()
{
  auto & d = diag_updater_;
  d.setHardwareID("control_validator");

  std::string ns = "control_validation_";
  d.add(ns + "max_distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_max_distance_deviation,
      "control output is deviated from trajectory");
  });
}

bool ControlValidator::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s);
    return false;
  };

  if (!current_kinematics_) {
    return waiting("current_kinematics_");
  }
  if (!current_reference_trajectory_) {
    return waiting("current_reference_trajectory_");
  }
  if (!current_predicted_trajectory_) {
    return waiting("current_predicted_trajectory_");
  }
  return true;
}

void ControlValidator::onReferenceTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_reference_trajectory_ = msg;

  return;
}

void ControlValidator::onPredictedTrajectory(const Trajectory::ConstSharedPtr msg)
{
  current_predicted_trajectory_ = msg;

  if (!isDataReady()) return;

  debug_pose_publisher_->clearMarkers();

  validate(*current_predicted_trajectory_);

  diag_updater_.force_update();

  publishPredictedTrajectory();

  // for debug
  publishDebugInfo();
  displayStatus();
}

void ControlValidator::publishPredictedTrajectory()
{
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(validation_status_)) {
    pub_traj_->publish(*current_predicted_trajectory_);
    previous_published_predicted_trajectory_ = current_predicted_trajectory_;
    return;
  }

  //  ----- invalid factor is found. Publish previous trajectory. -----

  if (
    invalid_predicted_trajectory_handling_type_ ==
    InvalidPredictedTrajectoryHandlingType::PUBLISH_AS_IT_IS) {
    pub_traj_->publish(*current_reference_trajectory_);
    RCLCPP_ERROR(get_logger(), "Caution! Invalid Trajectory published.");
    return;
  }

  if (
    invalid_predicted_trajectory_handling_type_ ==
    InvalidPredictedTrajectoryHandlingType::STOP_PUBLISHING) {
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Trajectory is not published.");
    return;
  }

  if (
    invalid_predicted_trajectory_handling_type_ ==
    InvalidPredictedTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
    if (previous_published_predicted_trajectory_) {
      pub_traj_->publish(*previous_published_predicted_trajectory_);
      RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use previous trajectory.");
      return;
    }
  }

  // predicted_trajectory is not published.
  RCLCPP_ERROR(
    get_logger(),
    "Invalid Predicted_Trajectory detected, no valid trajectory found in the past. Trajectory is "
    "not "
    "published.");
  return;
}

void ControlValidator::publishDebugInfo()
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

void ControlValidator::validate(const Trajectory & predicted_trajectory)
{
  if (predicted_trajectory.points.size() < 2) {
    RCLCPP_ERROR(get_logger(), "predicted_trajectory size is less than 2. Cannot validate.");
    return;
  }

  auto & s = validation_status_;

  s.is_valid_max_distance_deviation = checkValidMaxDistanceDeviation(predicted_trajectory);

  s.invalid_count = isAllValid(s) ? 0 : s.invalid_count + 1;
}

bool ControlValidator::checkValidMaxDistanceDeviation(const Trajectory & predicted_trajectory)
{
  const auto alined_predicted_trajectory =
    alignTrajectoryWithReferenceTrajectory(*current_reference_trajectory_, predicted_trajectory);

  validation_status_.max_distance_deviation =
    calcMaxLateralDistance(*current_reference_trajectory_, alined_predicted_trajectory);

  if (
    validation_status_.max_distance_deviation >
    validation_params_.max_distance_deviation_threshold) {
    return false;
  }
  return true;
}

Trajectory ControlValidator::alignTrajectoryWithReferenceTrajectory(
  const Trajectory & trajectory, const Trajectory & predicted_trajectory) const
{
  auto update_trajectory_point = [](
                                   bool condition, Trajectory & modified_trajectory,
                                   const Pose & reference_pose, bool is_front,
                                   const Trajectory & predicted_trajectory) {
    if (condition) {
      const auto point_to_interpolate =
        motion_utils::calcInterpolatedPoint(predicted_trajectory, reference_pose);

      if (is_front) {
        // Replace the front point with the new point
        modified_trajectory.points.front() = point_to_interpolate;
      } else {
        // Replace the back point with the new point
        modified_trajectory.points.back() = point_to_interpolate;
      }
    }
  };

  const auto last_seg_length = motion_utils::calcSignedArcLength(
    trajectory.points, trajectory.points.size() - 2, trajectory.points.size() - 1);

  // If no overlapping between trajectory and predicted_trajectory, return empty trajectory
  // predicted_trajectory:   p1------------------pN
  // trajectory:                                     t1------------------tN
  //     OR
  // predicted_trajectory:                           p1------------------pN
  // trajectory:             t1------------------tN
  const bool & is_pN_before_t1 =
    motion_utils::calcLongitudinalOffsetToSegment(
      trajectory.points, 0, predicted_trajectory.points.back().pose.position) < 0.0;
  const bool & is_p1_behind_tN = motion_utils::calcLongitudinalOffsetToSegment(
                                   trajectory.points, trajectory.points.size() - 2,
                                   predicted_trajectory.points.front().pose.position) -
                                   last_seg_length >
                                 0.0;
  const bool is_no_overlapping = (is_pN_before_t1 || is_p1_behind_tN);

  if (is_no_overlapping) {
    return Trajectory();
  }

  Trajectory modified_predicted_trajectory = predicted_trajectory;

  // If first point of predicted_trajectory is in front of start of trajectory, erase points which
  // are in front of trajectory start point and insert pNew along the predicted_trajectory
  // predicted_trajectory:   　　　　p1-----p2-----p3----//------pN
  // trajectory:                               t1--------//------tN
  // ↓
  // predicted_trajectory:   　　　　        tNew--p3----//------pN
  // trajectory:                               t1--------//------tN

  bool is_predicted_trajectory_first_point_front = false;
  for (auto it = predicted_trajectory.points.begin(); it != predicted_trajectory.points.end();
       ++it) {
    if (
      motion_utils::calcLongitudinalOffsetToSegment(trajectory.points, 0, it->pose.position) <
      0.0) {
      modified_predicted_trajectory.points.erase(modified_predicted_trajectory.points.begin());
    } else {
      break;
    }
    is_predicted_trajectory_first_point_front = true;
  }

  update_trajectory_point(
    is_predicted_trajectory_first_point_front, modified_predicted_trajectory,
    trajectory.points.front().pose, true, predicted_trajectory);

  // If last point of predicted_trajectory is behind of end of trajectory, erase points which are
  // behind trajectory last point and insert pNew along the predicted_trajectory
  // predicted_trajectory:   　　　　p1-----//------pN-2-----pN-1-----pN
  // trajectory:                     t1-----//-----tN-1--tN
  // ↓
  // predicted_trajectory:           p1-----//------pN-2-pNew
  // trajectory:                     t1-----//-----tN-1--tN
  bool is_predicted_trajectory_last_point_behind = false;
  for (auto it = predicted_trajectory.points.rbegin(); it != predicted_trajectory.points.rend();
       ++it) {
    if (
      motion_utils::calcLongitudinalOffsetToSegment(
        trajectory.points, trajectory.points.size() - 2, it->pose.position) -
        last_seg_length >
      0.0) {
      modified_predicted_trajectory.points.pop_back();
    } else {
      break;
    }
    is_predicted_trajectory_last_point_behind = true;
  }
  update_trajectory_point(
    is_predicted_trajectory_last_point_behind, modified_predicted_trajectory,
    trajectory.points.back().pose, false, predicted_trajectory);

  return modified_predicted_trajectory;
}

double calcMaxLateralDistance(
  const Trajectory & reference_trajectory, const Trajectory & predicted_trajectory)
{
  double max_dist = 0;
  for (const auto & point : predicted_trajectory.points) {
    const auto p0 = tier4_autoware_utils::getPoint(point);
    // find nearest segment
    const size_t nearest_segment_idx =
      motion_utils::findNearestSegmentIndex(reference_trajectory.points, p0);
    double temp_dist =
      motion_utils::calcLateralOffset(reference_trajectory.points, p0, nearest_segment_idx);
    if (max_dist > temp_dist) {
      max_dist = temp_dist;
    }
  }
  return max_dist;
}

bool ControlValidator::isAllValid(const ControlValidatorStatus & s)
{
  // TODO(Suagahara): add more control validation
  return s.is_valid_max_distance_deviation;
}

void ControlValidator::displayStatus()
{
  if (!display_on_terminal_) return;

  const auto warn = [this](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN(get_logger(), "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(s.is_valid_max_distance_deviation, "planning trajectory is too far from ego!!");
}

}  // namespace control_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(control_validator::ControlValidator)
