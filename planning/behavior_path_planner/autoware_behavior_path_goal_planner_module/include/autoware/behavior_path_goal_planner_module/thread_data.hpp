// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__THREAD_DATA_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__THREAD_DATA_HPP_

#include "autoware/behavior_path_goal_planner_module/goal_searcher_base.hpp"
#include "autoware/behavior_path_goal_planner_module/pull_over_planner/pull_over_planner_base.hpp"
#include "autoware/behavior_path_planner_common/utils/path_safety_checker/path_safety_checker_parameters.hpp"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <vector>

namespace autoware::behavior_path_planner
{

#define DEFINE_SETTER_WITH_MUTEX(TYPE, NAME)                  \
public:                                                       \
  void set_##NAME(const TYPE & value)                         \
  {                                                           \
    const std::lock_guard<std::recursive_mutex> lock(mutex_); \
    NAME##_ = value;                                          \
  }

#define DEFINE_GETTER_WITH_MUTEX(TYPE, NAME)                  \
public:                                                       \
  TYPE get_##NAME() const                                     \
  {                                                           \
    const std::lock_guard<std::recursive_mutex> lock(mutex_); \
    return NAME##_;                                           \
  }

#define DEFINE_SETTER_GETTER_WITH_MUTEX(TYPE, NAME) \
  DEFINE_SETTER_WITH_MUTEX(TYPE, NAME)              \
  DEFINE_GETTER_WITH_MUTEX(TYPE, NAME)

class PathDecisionState
{
public:
  enum class DecisionKind {
    NOT_DECIDED,
    DECIDING,
    DECIDED,
  };

  DecisionKind state{DecisionKind::NOT_DECIDED};
  rclcpp::Time stamp{};
  bool is_stable_safe{false};
  std::optional<rclcpp::Time> safe_start_time{std::nullopt};
};

class ThreadSafeData
{
public:
  ThreadSafeData(std::recursive_mutex & mutex, rclcpp::Clock::SharedPtr clock)
  : mutex_(mutex), clock_(clock)
  {
  }

  bool incrementPathIndex()
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!pull_over_path_) {
      return false;
    }

    if (pull_over_path_->incrementPathIndex()) {
      last_path_idx_increment_time_ = clock_->now();
      return true;
    }
    return false;
  }

  void set_pull_over_path(const PullOverPath & path)
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    set_pull_over_path_no_lock(path);
  }

  void set_pull_over_path(const std::shared_ptr<PullOverPath> & path)
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    set_pull_over_path_no_lock(path);
  }

  template <typename... Args>
  void set(Args... args)
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    (..., set_no_lock(args));
  }

  void clearPullOverPath()
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = nullptr;
  }

  bool foundPullOverPath() const
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!pull_over_path_) {
      return false;
    }

    return true;
  }

  PullOverPlannerType getPullOverPlannerType() const
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    return pull_over_path_->type();
  };

  void reset()
  {
    const std::lock_guard<std::recursive_mutex> lock(mutex_);
    pull_over_path_ = nullptr;
    pull_over_path_candidates_.clear();
    goal_candidates_.clear();
    modified_goal_pose_ = std::nullopt;
    last_path_update_time_ = std::nullopt;
    last_path_idx_increment_time_ = std::nullopt;
    closest_start_pose_ = std::nullopt;
    last_previous_module_output_ = std::nullopt;
    prev_data_ = PathDecisionState{};
  }

  DEFINE_GETTER_WITH_MUTEX(std::shared_ptr<PullOverPath>, pull_over_path)
  DEFINE_GETTER_WITH_MUTEX(std::shared_ptr<PullOverPath>, lane_parking_pull_over_path)
  DEFINE_GETTER_WITH_MUTEX(std::optional<rclcpp::Time>, last_path_update_time)
  DEFINE_GETTER_WITH_MUTEX(std::optional<rclcpp::Time>, last_path_idx_increment_time)

  DEFINE_SETTER_GETTER_WITH_MUTEX(std::vector<PullOverPath>, pull_over_path_candidates)
  DEFINE_SETTER_GETTER_WITH_MUTEX(GoalCandidates, goal_candidates)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<GoalCandidate>, modified_goal_pose)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<Pose>, closest_start_pose)
  DEFINE_SETTER_GETTER_WITH_MUTEX(std::optional<BehaviorModuleOutput>, last_previous_module_output)
  DEFINE_SETTER_GETTER_WITH_MUTEX(
    utils::path_safety_checker::CollisionCheckDebugMap, collision_check)
  DEFINE_SETTER_GETTER_WITH_MUTEX(PredictedObjects, static_target_objects)
  DEFINE_SETTER_GETTER_WITH_MUTEX(PredictedObjects, dynamic_target_objects)
  DEFINE_SETTER_GETTER_WITH_MUTEX(PathDecisionState, prev_data)

private:
  void set_pull_over_path_no_lock(const PullOverPath & path)
  {
    pull_over_path_ = std::make_shared<PullOverPath>(path);
    if (path.type() != PullOverPlannerType::FREESPACE) {
      lane_parking_pull_over_path_ = std::make_shared<PullOverPath>(path);
    }

    last_path_update_time_ = clock_->now();
  }

  void set_pull_over_path_no_lock(const std::shared_ptr<PullOverPath> & path)
  {
    pull_over_path_ = path;
    if (path->type() != PullOverPlannerType::FREESPACE) {
      lane_parking_pull_over_path_ = path;
    }
    last_path_update_time_ = clock_->now();
  }

  void set_no_lock(const GoalCandidates & arg) { goal_candidates_ = arg; }
  void set_no_lock(const std::vector<PullOverPath> & arg) { pull_over_path_candidates_ = arg; }
  void set_no_lock(const std::shared_ptr<PullOverPath> & arg) { set_pull_over_path_no_lock(arg); }
  void set_no_lock(const PullOverPath & arg) { set_pull_over_path_no_lock(arg); }
  void set_no_lock(const GoalCandidate & arg) { modified_goal_pose_ = arg; }
  void set_no_lock(const BehaviorModuleOutput & arg) { last_previous_module_output_ = arg; }
  void set_no_lock(const utils::path_safety_checker::CollisionCheckDebugMap & arg)
  {
    collision_check_ = arg;
  }

  std::shared_ptr<PullOverPath> pull_over_path_{nullptr};
  std::shared_ptr<PullOverPath> lane_parking_pull_over_path_{nullptr};
  std::vector<PullOverPath> pull_over_path_candidates_;
  GoalCandidates goal_candidates_{};
  std::optional<GoalCandidate> modified_goal_pose_;
  std::optional<rclcpp::Time> last_path_update_time_;
  std::optional<rclcpp::Time> last_path_idx_increment_time_;
  std::optional<Pose> closest_start_pose_{};
  std::optional<BehaviorModuleOutput> last_previous_module_output_{};
  utils::path_safety_checker::CollisionCheckDebugMap collision_check_{};
  PredictedObjects static_target_objects_{};
  PredictedObjects dynamic_target_objects_{};
  PathDecisionState prev_data_{};

  std::recursive_mutex & mutex_;
  rclcpp::Clock::SharedPtr clock_;
};

#undef DEFINE_SETTER_WITH_MUTEX
#undef DEFINE_GETTER_WITH_MUTEX
#undef DEFINE_SETTER_GETTER_WITH_MUTEX

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__THREAD_DATA_HPP_
