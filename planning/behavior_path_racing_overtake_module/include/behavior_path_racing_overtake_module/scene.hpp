// Copyright 2021 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__SCENE_HPP_
#define BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__SCENE_HPP_

#include "behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "behavior_path_planner_common/utils/path_shifter/path_shifter.hpp"
#include "behavior_path_racing_overtake_module/data_structs.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <tier4_planning_msgs/msg/lateral_offset.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <optional>

namespace behavior_path_planner
{

enum class OverTakeState
{
  DEFAULT,         // 0
  APPROACH,        // 1
  OVERTAKING,      // 2
  AFTER_OVERTAKE,  // 3
  BACK_TO_CENTER,  // 4
};

enum class OverTakeObjectType
{
  NON_BLOCKING,
  BLOCKING_TOO_CLOSE,
  BLOCKING_CLOSE,
  BLOCKING_FAR,
};

struct OverTakeObjectAndType
{
  PredictedObject object;
  OverTakeObjectType object_type;
};

using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::OccupancyGrid;
using tier4_planning_msgs::msg::LateralOffset;

namespace racing_overtake_state
{

using PlannerDataPtr = std::shared_ptr<const PlannerData>;

class Context;

class RacingOverTakeState
{
protected:
  Context* context_;

public:
  virtual ~RacingOverTakeState() = default;
  virtual void update(PlannerDataPtr planner_data) = 0;
  virtual std::string getName() const = 0;
  void setContext(Context* context);
  // virtual void getPath() = 0;
};

class ModuleNotLaunched : public RacingOverTakeState
{
public:
  void update(PlannerDataPtr planner_data) override;
  std::string getName() const override
  {
    return "ModuleNotLaunched";
  }
};

class Approach : public RacingOverTakeState
{
public:
  void update(PlannerDataPtr planner_data) override;
  std::string getName() const override
  {
    return "Approach";
  }
};

class Overtaking : public RacingOverTakeState
{
public:
  void update(PlannerDataPtr) override;
  std::string getName() const override
  {
    return "Overtaking";
  }
};

// class AfterOvertake : public RacingOverTakeState
// {
// };

class Context
{
  std::unique_ptr<RacingOverTakeState> state_;

public:
  explicit Context(std::unique_ptr<RacingOverTakeState> state = std::make_unique<ModuleNotLaunched>());
  void update(PlannerDataPtr planner_data);
  void transitionTo(std::unique_ptr<RacingOverTakeState> state);
  std::unique_ptr<RacingOverTakeState> getState();
};

}  // namespace racing_overtake_state

class RacingOvertakeModule : public SceneModuleInterface
{
public:
  RacingOvertakeModule(const std::string& name, rclcpp::Node& node,
                       const std::shared_ptr<RacingOvertakeParameters>& parameters,
                       const std::unordered_map<std::string, std::shared_ptr<RTCInterface>>& rtc_interface_ptr_map,
                       std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>>&
                           objects_of_interest_marker_interface_ptr_map);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  bool isReadyForNextRequest(const double& min_request_time_sec, bool override_requests = false) const noexcept;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void updateModuleParams(const std::any& /*parameters*/) override
  {
  }

  void acceptVisitor([[maybe_unused]] const std::shared_ptr<SceneModuleVisitor>& visitor) const override
  {
  }

private:
  // racing_overtake_state::Context context_;

  bool canTransitSuccessState() override;

  bool canTransitFailureState() override
  {
    return false;
  }

  std::optional<OverTakeObjectAndType> getClosestFrontObject(const PathWithLaneId& path) const;
  ShiftLine getOverTakeShiftLine(const PathWithLaneId& path, const PredictedObject& object) const;

  std::shared_ptr<RacingOvertakeParameters> parameters_;

  PathShifter path_shifter_;

  // vector of object
  // std::vector<autoware_auto_perception_msgs::msg::PredictedObject> near_objects_;
  mutable rclcpp::Time last_requested_shift_change_time_{ clock_->now() };

  std::optional<OverTakeObjectAndType> closest_front_object_ = std::nullopt;

  PathWithLaneId center_path_;
  double shift_length_ = 0.0;
  double base_offset_ = 0.0;
  OverTakeState overtake_state_ = OverTakeState::DEFAULT;
  OverTakeState prev_overtake_state_ = OverTakeState::DEFAULT;
  std::optional<ShiftLine> shift_line_for_overtake_ = std::nullopt;
};

}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__SCENE_HPP_
