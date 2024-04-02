// Copyright 2024 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__STATE_HPP_
#define BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__STATE_HPP_

#include "behavior_path_planner_common/data_manager.hpp"
#include "behavior_path_racing_overtake_module/data_structs.hpp"

#include "autoware_auto_perception_msgs/msg/predicted_object.hpp"
#include "autoware_auto_planning_msgs/msg/path_with_lane_id.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <memory>
#include <string>

namespace behavior_path_planner::racing_overtake::state
{
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using geometry_msgs::msg::Pose;
using PlannerDataPtr = std::shared_ptr<const PlannerData>;

class Context;

class RacingOverTakeState
{
protected:
  Context * context_;

public:
  explicit RacingOverTakeState(Context * context) : context_(context){};
  virtual ~RacingOverTakeState() = default;
  virtual void update(PlannerDataPtr planner_data) = 0;
  virtual std::string getName() const = 0;
  void setContext(Context * context);
  virtual void getPath(PlannerDataPtr planner_data, BehaviorModuleOutput * previous_output) = 0;
};

class ModuleNotLaunched : public RacingOverTakeState
{
public:
  explicit ModuleNotLaunched(Context * context) : RacingOverTakeState(context){};
  void update(PlannerDataPtr planner_data) override;
  std::string getName() const override;
  void getPath(PlannerDataPtr planner_data, BehaviorModuleOutput * output) override;
};

class Approach : public RacingOverTakeState
{
private:
  const double current_course_shift_length_;
  double after_overtake_shift_length_ = 0.0;

public:
  explicit Approach(Context * context, double current_course_shift_length = 0.0)
  : RacingOverTakeState(context), current_course_shift_length_(current_course_shift_length){};
  void update(PlannerDataPtr planner_data) override;
  std::string getName() const override;
  void getPath(PlannerDataPtr planner_data, BehaviorModuleOutput * output) override;
};

class Overtaking : public RacingOverTakeState
{
private:
  bool is_first_time_to_plan_ = true;
  PathWithLaneId overtaking_path_;
  Pose overtake_finish_pose_;

  const double current_course_shift_length_;
  double after_overtake_shift_length_ = 0.0;

public:
  explicit Overtaking(Context * context, double current_course_shift_length = 0.0)
  : RacingOverTakeState(context), current_course_shift_length_(current_course_shift_length){};
  void update(PlannerDataPtr) override;
  std::string getName() const override;
  void getPath(PlannerDataPtr planner_data, BehaviorModuleOutput * output) override;
};

class AfterOvertake : public RacingOverTakeState
{
private:
  const double after_overtake_shift_length_;

public:
  AfterOvertake(Context * context, double after_overtake_shift_length)
  : RacingOverTakeState(context), after_overtake_shift_length_(after_overtake_shift_length){};
  void update(PlannerDataPtr planner_data) override;
  std::string getName() const override;
  void getPath(PlannerDataPtr planner_data, BehaviorModuleOutput * output) override;
};

class BackToCenter : public RacingOverTakeState
{
private:
  const double after_overtake_shift_length_;
  PathWithLaneId back_to_center_path_;
  Pose back_to_center_end_pose_;
  bool is_first_time_to_plan_ = true;

public:
  BackToCenter(Context * context, const double after_overtake_shift_length)
  : RacingOverTakeState(context), after_overtake_shift_length_(after_overtake_shift_length){};
  void update(PlannerDataPtr planner_data) override;
  std::string getName() const override;
  void getPath(PlannerDataPtr planner_data, BehaviorModuleOutput * output) override;
};

class Context
{
  RacingOvertakeParameters parameters_;
  std::unique_ptr<RacingOverTakeState> state_;
  std::unique_ptr<RacingOverTakeState> previous_state_;

public:
  explicit Context(const RacingOvertakeParameters & parameters);

  void updateState(PlannerDataPtr planner_data);

  template <class State, class... StateArgs>
  void transitionTo(StateArgs &&... args)
  {
    previous_state_ = std::move(state_);
    state_ = std::make_unique<State>(this, std::forward<StateArgs>(args)...);
  }
  RacingOverTakeState & getState() const;
  RacingOverTakeState & getPreviousState() const;

  const RacingOvertakeParameters & getParameters() const;
  void updateParameters(const RacingOvertakeParameters & parameters);
};

}  // namespace behavior_path_planner::racing_overtake::state

#endif  // BEHAVIOR_PATH_RACING_OVERTAKE_MODULE__STATE_HPP_
