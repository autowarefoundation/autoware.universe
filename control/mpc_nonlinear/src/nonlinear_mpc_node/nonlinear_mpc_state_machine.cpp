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


#include <cmath>
#include "nonlinear_mpc_node/nonlinear_mpc_state_machine.h"

ns_states::VehicleMotionFSM::VehicleMotionFSM(double const &stop_entry_ego_speed,
																							double const &stop_entry_target_speed,
																							double const &keep_stopping_distance,
																							double const &will_stop_distance)
	: stop_state_entry_ego_speed_{stop_entry_ego_speed},
		stop_state_entry_target_speed_{stop_entry_target_speed},
		stop_state_keep_stopping_dist_{keep_stopping_distance},
		will_stop_dist_{will_stop_distance}
{

	current_state_ = &VehicleIsStoppedWillMove::getInstance();
	// current_state_.reset(&VehicleStopped::getInstance());
}

ns_states::VehicleMotionFSM::VehicleMotionFSM(const ns_states::VehicleMotionFSM &other)
	: stop_state_entry_ego_speed_{other.stop_state_entry_ego_speed_},
		stop_state_entry_target_speed_{other.stop_state_entry_target_speed_},
		stop_state_keep_stopping_dist_{other.stop_state_keep_stopping_dist_},
		will_stop_dist_{other.will_stop_dist_}
{

	current_state_ = &VehicleIsStoppedWillMove::getInstance();
}

ns_states::VehicleMotionFSM &ns_states::VehicleMotionFSM::operator=(const ns_states::VehicleMotionFSM &other)
{
	if (this != &other)
	{
		current_state_ = &VehicleIsStoppedWillMove::getInstance();
	}

	return *this;
}

void ns_states::VehicleMotionFSM::setState(ns_states::VehicleMotionState &new_state)
{
	current_state_->onExit(this);
	current_state_ = &new_state;
	current_state_->onEntry(this);

}

void ns_states::VehicleMotionFSM::toggle(std::array<double, 3> const &dist_v0_vnext)
{

	state_transition_vars_ = dist_v0_vnext;
	current_state_->toggle(this);

	// DEBUG
	// ns_utils::print(" State name", enum_texts[current_state_->getStateType()]);
	// end of DEBUG
}

void ns_states::VehicleMotionFSM::printCurrentStateMsg()
{
	ns_utils::print(" State name", enum_texts[current_state_->getStateType()]);

}

ns_states::motionStateEnums ns_states::VehicleMotionFSM::getCurrentState()
{

	return current_state_->getStateType();
}

void ns_states::VehicleMotionFSM::setReinitializationFlag(const bool &is_reinitialization_required)
{
	is_reInitialization_required_ = is_reinitialization_required;

}

bool ns_states::VehicleMotionFSM::isReinitializationRequired() const
{
	return is_reInitialization_required_;
}

void ns_states::VehicleMotionFSM::setEmergencyFlag(const bool &is_vehicle_in_emergency)
{
	is_vehicle_in_emergency_ = is_vehicle_in_emergency;

	if (is_vehicle_in_emergency_)
	{
		current_state_ = &VehicleIsInEmergency::getInstance();
	}

}

bool ns_states::VehicleMotionFSM::isVehicleInEmergencyStopping() const
{
	return is_vehicle_in_emergency_;
}

std::string ns_states::VehicleMotionFSM::fsmMessage()
{
	return enum_texts[current_state_->getStateType()];
}

//ns_states::VehicleMotionFSM &ns_states::VehicleMotionFSM::operator=(ns_states::VehicleMotionFSM &&other)
//noexcept
//{
//
//    if (this != &other)
//    {
//        current_state_.reset(&VehicleStopped::getInstance());
//    }
//    return *this;
//}
//
//ns_states::VehicleMotionFSM::VehicleMotionFSM(ns_states::VehicleMotionState &&other)
//{
//    current_state_.reset(&VehicleStopped::getInstance());
//
//}


// --------------------- Vehicle isAtCompleteStop -------------------------------
ns_states::VehicleMotionState &ns_states::VehicleIsAtCompleteStop::getInstance()
{
	static VehicleIsAtCompleteStop singleton;
	return singleton;
}

void ns_states::VehicleIsAtCompleteStop::onEntry(ns_states::VehicleMotionFSM *vecMotionFSM)
{

	vecMotionFSM->setReinitializationFlag(true);
}

void ns_states::VehicleIsAtCompleteStop::toggle(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	auto const &dist_to_stop_point = vecMotionFSM->state_transition_vars_[0];
	auto const &ego_speed = std::fabs(vecMotionFSM->state_transition_vars_[1]);
	// auto &&next_point_speed_ref = std::fabs(vecMotionFSM->state_transition_vars_[2]);

	if (dist_to_stop_point > vecMotionFSM->stop_state_keep_stopping_dist_ &&
		ego_speed < vecMotionFSM->stop_state_entry_ego_speed_)
	{
		vecMotionFSM->setState(VehicleIsStoppedWillMove::getInstance());
	}

}

void ns_states::VehicleIsAtCompleteStop::onExit(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.
}

ns_states::motionStateEnums ns_states::VehicleIsAtCompleteStop::getStateType()
{
	return ns_states::motionStateEnums::isAtCompleteStop;
}

// --------------------- Vehicle isStoppedWillMove -------------------------------
ns_states::VehicleIsStoppedWillMove &ns_states::VehicleIsStoppedWillMove::getInstance()
{
	static VehicleIsStoppedWillMove singleton;
	return singleton;
}

void ns_states::VehicleIsStoppedWillMove::onEntry(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.
}

void ns_states::VehicleIsStoppedWillMove::toggle(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	auto const &dist_to_stop_point = vecMotionFSM->state_transition_vars_[0];
	auto const &ego_speed = std::fabs(vecMotionFSM->state_transition_vars_[1]);
	// auto const&next_point_speed_ref = std::fabs(vecMotionFSM->state_transition_vars_[2]);

	if (dist_to_stop_point > vecMotionFSM->stop_state_keep_stopping_dist_ &&
		ego_speed > vecMotionFSM->stop_state_entry_ego_speed_)
	{
		vecMotionFSM->setState(VehicleIsMoving::getInstance());
	}

}

void ns_states::VehicleIsStoppedWillMove::onExit(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.
}

ns_states::motionStateEnums ns_states::VehicleIsStoppedWillMove::getStateType()
{
	return ns_states::motionStateEnums::isStoppedWillMove;
}

// --------------------- Vehicle willStop -------------------------------
ns_states::VehicleMotionState &ns_states::VehicleWillStop::getInstance()
{
	static VehicleWillStop singleton;
	return singleton;
}

void ns_states::VehicleWillStop::onEntry(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.
}

void ns_states::VehicleWillStop::toggle(ns_states::VehicleMotionFSM *vecMotionFSM)
{

	auto const &dist_to_stop_point = vecMotionFSM->state_transition_vars_[0];
	auto const &ego_speed = std::fabs(vecMotionFSM->state_transition_vars_[1]);
	auto const &next_point_speed_ref = std::fabs(vecMotionFSM->state_transition_vars_[2]);

	if (dist_to_stop_point < vecMotionFSM->will_stop_dist_ &&
		ego_speed < vecMotionFSM->stop_state_entry_ego_speed_ &&
		next_point_speed_ref < vecMotionFSM->stop_state_entry_target_speed_)
	{
		vecMotionFSM->setState(VehicleIsAtCompleteStop::getInstance());
	}
}

void ns_states::VehicleWillStop::onExit(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.
}

ns_states::motionStateEnums ns_states::VehicleWillStop::getStateType()
{
	return ns_states::motionStateEnums::willStop;
}

// --------------------- Vehicle isMoving -------------------------------
ns_states::VehicleMotionState &ns_states::VehicleIsMoving::getInstance()
{
	static VehicleIsMoving singleton;
	return singleton;
}

ns_states::motionStateEnums ns_states::VehicleIsMoving::getStateType()
{
	return ns_states::motionStateEnums::isMoving;
}

void ns_states::VehicleIsMoving::onEntry(ns_states::VehicleMotionFSM *vecMotionFSM)
{

}

void ns_states::VehicleIsMoving::toggle(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	auto const &dist_to_stop_point = vecMotionFSM->state_transition_vars_[0];
//    auto const&ego_speed = std::fabs(vecMotionFSM->state_transition_vars_[1]);
//    auto const&next_point_speed_ref = std::fabs(vecMotionFSM->state_transition_vars_[2]);

	if (dist_to_stop_point < vecMotionFSM->will_stop_dist_)
	{
		vecMotionFSM->setState(VehicleWillStop::getInstance());
	}

}

void ns_states::VehicleIsMoving::onExit(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.
}

// --------------------- Vehicle isInEmergency -------------------------------
ns_states::VehicleMotionState &ns_states::VehicleIsInEmergency::getInstance()
{
	static VehicleIsInEmergency singleton;
	return singleton;
}

void ns_states::VehicleIsInEmergency::onEntry(ns_states::VehicleMotionFSM *vecMotionFSM)
{

	vecMotionFSM->setReinitializationFlag(true);
}

void ns_states::VehicleIsInEmergency::toggle(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// auto const&dist_to_stop_point = vecMotionFSM->state_transition_vars_[0];
	auto const &ego_speed = std::fabs(vecMotionFSM->state_transition_vars_[1]);
	// auto const&next_point_speed_ref = std::fabs(vecMotionFSM->state_transition_vars_[2]);

	if (ego_speed < vecMotionFSM->stop_state_entry_ego_speed_)
	{
		vecMotionFSM->setState(VehicleIsAtCompleteStop::getInstance());
	}

}

void ns_states::VehicleIsInEmergency::onExit(ns_states::VehicleMotionFSM *vecMotionFSM)
{
	// Will be implemented when needed.

}

ns_states::motionStateEnums ns_states::VehicleIsInEmergency::getStateType()
{
	return ns_states::motionStateEnums::isInEmergency;
}