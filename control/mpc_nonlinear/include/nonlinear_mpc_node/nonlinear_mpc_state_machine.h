/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MPC_NONLINEAR_NONLINEAR_MPC_STATE_MACHINE_H
#define MPC_NONLINEAR_NONLINEAR_MPC_STATE_MACHINE_H

#include <memory>
#include <map>
#include "utils/nmpc_utils.hpp"
#include "utils_act/act_utils.hpp"

namespace ns_states
{
enum class motionStateEnums : int
{
  isAtCompleteStop = 0, // there is no trajectory to follow
  isStoppedWillMove = 1, // vehicle is in stopped state will start moving
  willStop = -1, // vehicle will be moving
  isMoving = 2, // vehicle is moving.
  isInEmergency = -2, // vehicle is in emergency state.
};

// Forward declaration
class VehicleMotionFSM;

// Vehicle Motion State.
class VehicleMotionState
{
 public:
  virtual void onEntry(VehicleMotionFSM *vecMotionFSM) = 0;

  virtual void toggle(VehicleMotionFSM *vecMotionFSM) = 0;

  virtual void onExit(VehicleMotionFSM *vecMotionFSM) = 0;

  virtual motionStateEnums getStateType() = 0;

  virtual ~VehicleMotionState() = default;

};

// Vehicle is in complete stop
class VehicleIsAtCompleteStop : public VehicleMotionState
{
 public:
  VehicleIsAtCompleteStop(VehicleIsAtCompleteStop &&) = delete;

  VehicleIsAtCompleteStop &&operator=(VehicleIsAtCompleteStop &&) = delete;

  void onEntry(VehicleMotionFSM *vecMotionFSM) override;

  void toggle(VehicleMotionFSM *vecMotionFSM) override;

  void onExit(VehicleMotionFSM *vecMotionFSM) override;

  static VehicleMotionState &getInstance();

  motionStateEnums getStateType() override;

 private:
  VehicleIsAtCompleteStop() = default;

  VehicleIsAtCompleteStop(const VehicleIsAtCompleteStop &other);

  VehicleIsAtCompleteStop &operator=(const VehicleIsAtCompleteStop &other);

  ~VehicleIsAtCompleteStop() override = default;

};

// Vehicle is stopping and can move.
class VehicleIsStoppedWillMove : public VehicleMotionState
{
 public:

  VehicleIsStoppedWillMove(VehicleIsStoppedWillMove &&) = delete;

  VehicleIsStoppedWillMove &&operator=(VehicleIsStoppedWillMove &&) = delete;

  void onEntry(VehicleMotionFSM *vecMotionFSM) override;

  void toggle(VehicleMotionFSM *vecMotionFSM) override;

  void onExit(VehicleMotionFSM *vecMotionFSM) override;

  static VehicleIsStoppedWillMove &getInstance();

  motionStateEnums getStateType() override;

 private:
  VehicleIsStoppedWillMove() = default;

  VehicleIsStoppedWillMove(const VehicleIsStoppedWillMove &other);

  VehicleIsStoppedWillMove &operator=(const VehicleIsStoppedWillMove &other);

  ~VehicleIsStoppedWillMove() override = default;

};

// Vehicle will stop.
class VehicleWillStop : public VehicleMotionState
{
 public:

  VehicleWillStop(VehicleWillStop &&) = delete;

  VehicleWillStop &&operator=(VehicleWillStop &&) = delete;

  void onEntry(VehicleMotionFSM *vecMotionFSM) override;

  void toggle(VehicleMotionFSM *vecMotionFSM) override;

  void onExit(VehicleMotionFSM *vecMotionFSM) override;

  static VehicleMotionState &getInstance();

  motionStateEnums getStateType() override;

 private:
  VehicleWillStop() = default;

  VehicleWillStop(const VehicleWillStop &other);

  VehicleWillStop &operator=(const VehicleWillStop &other);

  ~VehicleWillStop() override = default;

};

// Vehicle is moving.
class VehicleIsMoving : public VehicleMotionState
{
 public:
  VehicleIsMoving(VehicleIsMoving &&) = delete;

  VehicleIsMoving &&operator=(VehicleIsMoving &&) = delete;

  void onEntry(VehicleMotionFSM *vecMotionFSM) override;

  void toggle(VehicleMotionFSM *vecMotionFSM) override;

  void onExit(VehicleMotionFSM *vecMotionFSM) override;

  static VehicleMotionState &getInstance();

  motionStateEnums getStateType() override;

 private:
  VehicleIsMoving() = default;

  VehicleIsMoving(const VehicleIsMoving &other);

  VehicleIsMoving &operator=(const VehicleIsMoving &other);

  ~VehicleIsMoving() override = default;

};

// Vehicle is in complete stop
class VehicleIsInEmergency : public VehicleMotionState
{
 public:
  VehicleIsInEmergency(VehicleIsInEmergency &&) = delete;

  VehicleIsInEmergency &&operator=(VehicleIsInEmergency &&) = delete;

  void onEntry(VehicleMotionFSM *vecMotionFSM) override;

  void toggle(VehicleMotionFSM *vecMotionFSM) override;

  void onExit(VehicleMotionFSM *vecMotionFSM) override;

  static VehicleMotionState &getInstance();

  motionStateEnums getStateType() override;

 private:
  VehicleIsInEmergency() = default;

  VehicleIsInEmergency(const VehicleIsInEmergency &other);

  VehicleIsInEmergency &operator=(const VehicleIsInEmergency &other);

  ~VehicleIsInEmergency() override = default;

};

// -- Finite State Machine
class VehicleMotionFSM
{
 public:

  // Constructors and destructors.
  VehicleMotionFSM() = default;

  VehicleMotionFSM(double const &stop_entry_ego_speed,
                   double const &stop_entry_target_speed,
                   double const &keep_stopping_distance,
                   double const &will_stop_distance);

  VehicleMotionFSM(VehicleMotionFSM const &other);

  VehicleMotionFSM &operator=(VehicleMotionFSM const &other);

//        VehicleMotionFSM &operator=(VehicleMotionFSM &&other) noexcept;//
//        explicit VehicleMotionFSM(VehicleMotionState &&other);

  // Public methods.
  motionStateEnums getCurrentState();

  // Switch states given an array of [distance_to_stop, vcurrent, vnext]
  void toggle(std::array<double, 3> const &dist_v0_vnext);

  void setState(VehicleMotionState &newState);

  void printCurrentStateMsg();

  std::string fsmMessage();

  // Set flags.
  void setReinitializationFlag(bool const &is_reinitialization_required);

  [[nodiscard]] bool isReinitializationRequired() const;

  void setEmergencyFlag(bool const &is_vehicle_in_emergency);

  [[nodiscard]] bool isVehicleInEmergencyStopping() const;

  // Public members.
  // State transition variables ; an array of [distance_to_stop, vcurrent, vnext]
  // Transition condition variables.
  double stop_state_entry_ego_speed_{0.2}; // [m/s]
  double stop_state_entry_target_speed_{0.5}; // [m]
  double stop_state_keep_stopping_dist_{0.5}; // [m]
  double will_stop_dist_{2.0}; // [m]
  std::array<double, 3> state_transition_vars_{};

  std::map<motionStateEnums, std::string> enum_texts{
    {motionStateEnums::isAtCompleteStop, "Vehicle is at complete stop."},
    {motionStateEnums::isStoppedWillMove, "Vehicle is stopping and will be moving."},
    {motionStateEnums::willStop, "Vehicle will stop."},
    {motionStateEnums::isMoving, "Vehicle is moving."},
    {motionStateEnums::isInEmergency, "Vehicle is in EMERGENCY state."}};

 private:
  VehicleMotionState *current_state_{&VehicleIsStoppedWillMove::getInstance()};
  bool is_reInitialization_required_{false};
  bool is_vehicle_in_emergency_{false};

};

template<typename Enumeration>
auto as_integer(Enumeration const value) -> typename std::underlying_type<Enumeration>::type
{
  return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}
} // namespace ns_states
#endif //MPC_NONLINEAR_NONLINEAR_MPC_STATE_MACHINE_H
