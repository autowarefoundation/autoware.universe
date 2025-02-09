// Copyright 2021 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_INTERFACE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_INTERFACE_HPP_

#include "autoware/behavior_path_planner_common/data_manager.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_visitor.hpp"
#include "autoware/behavior_path_planner_common/marker_utils/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/behavior_path_planner_common/turn_signal_decider.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/path_with_lane_id.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/objects_of_interest_marker_interface/objects_of_interest_marker_interface.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/rtc_interface/rtc_interface.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/ros/marker_helper.hpp>
#include <autoware/universe_utils/ros/uuid_helper.hpp>
#include <autoware/universe_utils/system/time_keeper.hpp>
#include <magic_enum.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/avoidance_debug_msg_array.hpp>
#include <tier4_rtc_msgs/msg/state.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <algorithm>
#include <any>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware::objects_of_interest_marker_interface::ColorName;
using autoware::objects_of_interest_marker_interface::ObjectsOfInterestMarkerInterface;
using autoware::planning_factor_interface::PlanningFactorInterface;
using autoware::rtc_interface::RTCInterface;
using autoware::universe_utils::calcOffsetPose;
using autoware::universe_utils::generateUUID;
using autoware_internal_planning_msgs::msg::PathWithLaneId;
using tier4_planning_msgs::msg::AvoidanceDebugMsgArray;
using tier4_planning_msgs::msg::PlanningFactor;
using tier4_planning_msgs::msg::SafetyFactorArray;
using tier4_rtc_msgs::msg::State;
using unique_identifier_msgs::msg::UUID;
using visualization_msgs::msg::MarkerArray;
using PlanResult = PathWithLaneId::SharedPtr;

enum class ModuleStatus {
  IDLE = 0,
  RUNNING = 1,
  WAITING_APPROVAL = 2,
  SUCCESS = 3,
  FAILURE = 4,
};

class SceneModuleInterface
{
public:
  SceneModuleInterface(
    const std::string & name, rclcpp::Node & node,
    std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
  : name_{name},
    logger_{node.get_logger().get_child(name)},
    clock_{node.get_clock()},
    rtc_interface_ptr_map_(std::move(rtc_interface_ptr_map)),
    objects_of_interest_marker_interface_ptr_map_(
      std::move(objects_of_interest_marker_interface_ptr_map)),
    planning_factor_interface_{planning_factor_interface},
    time_keeper_(std::make_shared<universe_utils::TimeKeeper>())
  {
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      uuid_map_.emplace(module_name, generateUUID());
    }
  }

  SceneModuleInterface(const SceneModuleInterface &) = delete;
  SceneModuleInterface(SceneModuleInterface &&) = delete;
  SceneModuleInterface & operator=(const SceneModuleInterface &) = delete;
  SceneModuleInterface & operator=(SceneModuleInterface &&) = delete;
  virtual ~SceneModuleInterface() = default;

  virtual void updateModuleParams(const std::any & parameters) = 0;

  virtual void acceptVisitor(const std::shared_ptr<SceneModuleVisitor> & visitor) const = 0;

  /**
   * @brief Return true if the module has request for execution (not necessarily feasible)
   */
  virtual bool isExecutionRequested() const = 0;

  /**
   * @brief Return true if the execution is available (e.g. safety check for lane change)
   */
  virtual bool isExecutionReady() const = 0;

  /**
   * @brief update data for planning. Note that the call of this function does not mean
   *        that the module executed. It should only updates the data necessary for
   *        planCandidate (e.g., resampling of path).
   */
  virtual void updateData() {}

  /**
   * @brief After executing run(), update the module-specific status and/or data used for internal
   *        processing that are not defined in ModuleStatus.
   */
  virtual void postProcess() {}

  /**
   * @brief Execute module. Once this function is executed,
   *        it will continue to run as long as it is in the RUNNING state.
   */
  virtual BehaviorModuleOutput run()
  {
    updateData();
    const auto output = isWaitingApproval() ? planWaitingApproval() : plan();
    try {
      autoware::motion_utils::validateNonEmpty(output.path.points);
    } catch (const std::exception & ex) {
      throw std::invalid_argument("[" + name_ + "]" + ex.what());
    }
    return output;
  }

  /**
   * @brief Set the current_state_ based on updateState output.
   */
  void updateCurrentState()
  {
    const auto print = [this](const auto & from, const auto & to) {
      RCLCPP_DEBUG(
        getLogger(), "[%s] Transit from %s to %s.", name_.c_str(), from.data(), to.data());
    };

    const auto & from = current_state_;
    current_state_ = updateState();
    print(magic_enum::enum_name(from), magic_enum::enum_name(current_state_));
  }

  /**
   * @brief Called on the first time when the module goes into RUNNING.
   */
  void onEntry();

  /**
   * @brief Called when the module exit from RUNNING.
   */
  void onExit()
  {
    RCLCPP_DEBUG(getLogger(), "%s %s", name_.c_str(), __func__);

    if (getCurrentStatus() == ModuleStatus::SUCCESS) {
      updateRTCStatusForSuccess();
    }

    clearWaitingApproval();
    unlockNewModuleLaunch();
    unlockOutputPath();

    processOnExit();
  }

  void publishObjectsOfInterestMarker()
  {
    for (const auto & [module_name, ptr] : objects_of_interest_marker_interface_ptr_map_) {
      if (ptr) {
        ptr->publishMarkerArray();
      }
    }
  }

  void lockRTCCommand()
  {
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      if (ptr) {
        ptr->lockCommandUpdate();
      }
    }
  }

  void unlockRTCCommand()
  {
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      if (ptr) {
        ptr->unlockCommandUpdate();
      }
    }
  }

  /**
   * @brief set previous module's output as input for this module
   */
  void setPreviousModuleOutput(const BehaviorModuleOutput & previous_module_output)
  {
    previous_module_output_ = previous_module_output;
  }

  std::shared_ptr<universe_utils::TimeKeeper> getTimeKeeper() const { return time_keeper_; }

  /**
   * @brief set planner data
   */
  virtual void setData(const std::shared_ptr<const PlannerData> & data) { planner_data_ = data; }

  void lockOutputPath() { is_locked_output_path_ = true; }

  void unlockOutputPath() { is_locked_output_path_ = false; }

  bool isWaitingApproval() const
  {
    return is_waiting_approval_ || current_state_ == ModuleStatus::WAITING_APPROVAL;
  }

  virtual bool isCurrentRouteLaneletToBeReset() const { return false; }

  bool isLockedNewModuleLaunch() const { return is_locked_new_module_launch_; }

  PlanResult getPathCandidate() const { return path_candidate_; }

  PlanResult getPathReference() const { return path_reference_; }

  MarkerArray getInfoMarkers() const { return info_marker_; }

  MarkerArray getDebugMarkers() const { return debug_marker_; }

  MarkerArray getDrivableLanesMarkers() const { return drivable_lanes_marker_; }

  virtual MarkerArray getModuleVirtualWall() { return MarkerArray(); }

  ModuleStatus getCurrentStatus() const { return current_state_; }

  std::string name() const { return name_; }

  PoseWithDetailOpt getStopPose() const
  {
    if (!stop_pose_) {
      return {};
    }

    const auto & base_link2front = planner_data_->parameters.base_link2front;
    return PoseWithDetail(
      calcOffsetPose(stop_pose_.value().pose, base_link2front, 0.0, 0.0),
      stop_pose_.value().detail);
  }

  PoseWithDetailOpt getSlowPose() const
  {
    if (!slow_pose_) {
      return {};
    }

    const auto & base_link2front = planner_data_->parameters.base_link2front;
    return PoseWithDetail(
      calcOffsetPose(slow_pose_.value().pose, base_link2front, 0.0, 0.0),
      slow_pose_.value().detail);
  }

  PoseWithDetailOpt getDeadPose() const
  {
    if (!dead_pose_) {
      return {};
    }

    const auto & base_link2front = planner_data_->parameters.base_link2front;
    return PoseWithDetail(
      calcOffsetPose(dead_pose_.value().pose, base_link2front, 0.0, 0.0),
      dead_pose_.value().detail);
  }

  void resetWallPoses() const
  {
    stop_pose_ = std::nullopt;
    slow_pose_ = std::nullopt;
    dead_pose_ = std::nullopt;
  }

  rclcpp::Logger getLogger() const { return logger_; }

private:
  bool existRegisteredRequest() const
  {
    return std::any_of(
      rtc_interface_ptr_map_.begin(), rtc_interface_ptr_map_.end(),
      [&](const auto & rtc) { return rtc.second->isRegistered(uuid_map_.at(rtc.first)); });
  }

  bool existApprovedRequest() const
  {
    return std::any_of(
      rtc_interface_ptr_map_.begin(), rtc_interface_ptr_map_.end(), [&](const auto & rtc) {
        if (!rtc.second->isRegistered(uuid_map_.at(rtc.first))) {
          return false;
        }

        if (rtc.second->isTerminated(uuid_map_.at(rtc.first))) {
          return false;
        }

        return rtc.second->isActivated(uuid_map_.at(rtc.first));
      });
  }

  bool existNotApprovedRequest() const
  {
    return std::any_of(
      rtc_interface_ptr_map_.begin(), rtc_interface_ptr_map_.end(), [&](const auto & rtc) {
        return rtc.second->isRegistered(uuid_map_.at(rtc.first)) &&
               !rtc.second->isActivated(uuid_map_.at(rtc.first)) &&
               !rtc.second->isTerminated(uuid_map_.at(rtc.first));
      });
  }

  bool canTransitWaitingApprovalState() const
  {
    if (!existRegisteredRequest()) {
      return false;
    }
    return existNotApprovedRequest();
  }

  bool canTransitWaitingApprovalToRunningState() const
  {
    if (!existRegisteredRequest()) {
      return true;
    }
    return existApprovedRequest();
  }

  /**
   * @brief Return SUCCESS if plan is not needed or plan is successfully finished,
   *        FAILURE if plan has failed, RUNNING if plan is on going.
   *        These condition is to be implemented in each modules.
   */
  ModuleStatus updateState()
  {
    auto log_debug_throttled = [&](std::string_view message) -> void {
      RCLCPP_DEBUG(getLogger(), "%s", message.data());
    };
    if (current_state_ == ModuleStatus::IDLE) {
      auto init_state = setInitState();
      RCLCPP_DEBUG(
        getLogger(), "transiting from IDLE to %s", magic_enum::enum_name(init_state).data());
      return init_state;
    }

    if (current_state_ == ModuleStatus::RUNNING) {
      if (canTransitSuccessState()) {
        log_debug_throttled("transiting from RUNNING to SUCCESS");
        return ModuleStatus::SUCCESS;
      }

      if (canTransitFailureState()) {
        log_debug_throttled("transiting from RUNNING to FAILURE");
        return ModuleStatus::FAILURE;
      }

      if (canTransitWaitingApprovalState()) {
        log_debug_throttled("transiting from RUNNING to WAITING_APPROVAL");
        return ModuleStatus::WAITING_APPROVAL;
      }

      log_debug_throttled("transiting from RUNNING to RUNNING");
      return ModuleStatus::RUNNING;
    }

    if (current_state_ == ModuleStatus::WAITING_APPROVAL) {
      if (canTransitSuccessState()) {
        log_debug_throttled("transiting from WAITING_APPROVAL to SUCCESS");
        return ModuleStatus::SUCCESS;
      }

      if (canTransitFailureState()) {
        log_debug_throttled("transiting from WAITING_APPROVAL to FAILURE");
        return ModuleStatus::FAILURE;
      }

      if (canTransitWaitingApprovalToRunningState()) {
        log_debug_throttled("transiting from WAITING_APPROVAL to RUNNING");
        return ModuleStatus::RUNNING;
      }

      log_debug_throttled("transiting from WAITING_APPROVAL to WAITING APPROVAL");
      return ModuleStatus::WAITING_APPROVAL;
    }

    if (current_state_ == ModuleStatus::SUCCESS) {
      log_debug_throttled("already SUCCESS");
      return ModuleStatus::SUCCESS;
    }

    if (current_state_ == ModuleStatus::FAILURE) {
      log_debug_throttled("already FAILURE");
      return ModuleStatus::FAILURE;
    }

    log_debug_throttled("already IDLE");
    return ModuleStatus::IDLE;
  }

  std::string name_;

  ModuleStatus current_state_{ModuleStatus::IDLE};

  BehaviorModuleOutput previous_module_output_;

  bool is_locked_new_module_launch_{false};

  bool is_locked_output_path_{false};

protected:
  /**
   * @brief State transition condition ANY -> SUCCESS
   */
  virtual bool canTransitSuccessState() = 0;

  /**
   * @brief State transition condition ANY -> FAILURE
   */
  virtual bool canTransitFailureState() = 0;

  /**
   * @brief Explicitly set the initial state
   */
  virtual ModuleStatus setInitState() const { return ModuleStatus::RUNNING; }

  /**
   * @brief Get candidate path. This information is used for external judgement.
   */
  virtual CandidateOutput planCandidate() const = 0;

  /**
   * @brief Calculate path. This function is called with the plan is approved.
   */
  virtual BehaviorModuleOutput plan() = 0;

  /**
   * @brief Calculate path under waiting_approval condition.
   *        The default implementation is just to return the reference path.
   */
  virtual BehaviorModuleOutput planWaitingApproval()
  {
    path_candidate_ = std::make_shared<PathWithLaneId>(planCandidate().path_candidate);
    path_reference_ = std::make_shared<PathWithLaneId>(getPreviousModuleOutput().reference_path);

    return getPreviousModuleOutput();
  }

  /**
   * @brief Module unique entry process.
   */
  virtual void processOnEntry() {}

  /**
   * @brief Module unique exit process.
   */
  virtual void processOnExit() {}

  virtual void updateRTCStatus(const double start_distance, const double finish_distance)
  {
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      if (ptr) {
        const auto state = !ptr->isRegistered(uuid_map_.at(module_name)) || isWaitingApproval()
                             ? State::WAITING_FOR_EXECUTION
                             : State::RUNNING;
        ptr->updateCooperateStatus(
          uuid_map_.at(module_name), isExecutionReady(), state, start_distance, finish_distance,
          clock_->now());
      }
    }
  }

  void updateRTCStatusForSuccess()
  {
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      if (ptr) {
        if (ptr->isRegistered(uuid_map_.at(module_name))) {
          ptr->updateCooperateStatus(
            uuid_map_.at(module_name), true, State::SUCCEEDED,
            std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(),
            clock_->now());
        }
      }
    }
  }

  void setObjectsOfInterestData(
    const geometry_msgs::msg::Pose & obj_pose,
    const autoware_perception_msgs::msg::Shape & obj_shape, const ColorName & color_name)
  {
    for (const auto & [module_name, ptr] : objects_of_interest_marker_interface_ptr_map_) {
      if (ptr) {
        ptr->insertObjectData(obj_pose, obj_shape, color_name);
      }
    }
  }

  /**
   * @brief Return true if the activation command is received from the RTC interface.
   *        If no RTC interface is registered, return true.
   */
  bool isActivated() const
  {
    if (rtc_interface_ptr_map_.empty()) {
      return true;
    }

    if (!existRegisteredRequest()) {
      return false;
    }
    return existApprovedRequest();
  }

  void removeRTCStatus()
  {
    for (const auto & [module_name, ptr] : rtc_interface_ptr_map_) {
      if (ptr) {
        ptr->clearCooperateStatus();
      }
    }
  }

  void set_longitudinal_planning_factor(const PathWithLaneId & path)
  {
    if (stop_pose_.has_value()) {
      planning_factor_interface_->add(
        path.points, getEgoPose(), stop_pose_.value().pose, PlanningFactor::STOP,
        SafetyFactorArray{}, true, 0.0, 0.0, stop_pose_.value().detail);
      return;
    }

    if (!slow_pose_.has_value()) {
      return;
    }

    planning_factor_interface_->add(
      path.points, getEgoPose(), slow_pose_.value().pose, PlanningFactor::SLOW_DOWN,
      SafetyFactorArray{}, true, 0.0, 0.0, slow_pose_.value().detail);
  }

  void setDrivableLanes(const std::vector<DrivableLanes> & drivable_lanes);

  BehaviorModuleOutput getPreviousModuleOutput() const { return previous_module_output_; }

  bool isOutputPathLocked() const { return is_locked_output_path_; }

  void lockNewModuleLaunch() { is_locked_new_module_launch_ = true; }

  void unlockNewModuleLaunch() { is_locked_new_module_launch_ = false; }

  void waitApproval() { is_waiting_approval_ = true; }

  void clearWaitingApproval() { is_waiting_approval_ = false; }

  void resetPathCandidate() { path_candidate_.reset(); }

  void resetPathReference() { path_reference_.reset(); }

  geometry_msgs::msg::Point getEgoPosition() const
  {
    return planner_data_->self_odometry->pose.pose.position;
  }

  geometry_msgs::msg::Pose getEgoPose() const { return planner_data_->self_odometry->pose.pose; }

  geometry_msgs::msg::Twist getEgoTwist() const
  {
    return planner_data_->self_odometry->twist.twist;
  }

  double getEgoSpeed() const
  {
    return std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  }

  rclcpp::Logger logger_;

  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<const PlannerData> planner_data_;

  bool is_waiting_approval_{false};

  std::unordered_map<std::string, UUID> uuid_map_;

  PlanResult path_candidate_;
  PlanResult path_reference_;

  std::unordered_map<std::string, std::shared_ptr<RTCInterface>> rtc_interface_ptr_map_;

  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>
    objects_of_interest_marker_interface_ptr_map_;

  mutable std::shared_ptr<PlanningFactorInterface> planning_factor_interface_;

  mutable PoseWithDetailOpt stop_pose_{std::nullopt};

  mutable PoseWithDetailOpt slow_pose_{std::nullopt};

  mutable PoseWithDetailOpt dead_pose_{std::nullopt};

  mutable MarkerArray info_marker_;

  mutable MarkerArray debug_marker_;

  mutable MarkerArray drivable_lanes_marker_;

  mutable std::shared_ptr<universe_utils::TimeKeeper> time_keeper_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_PLANNER_COMMON__INTERFACE__SCENE_MODULE_INTERFACE_HPP_
