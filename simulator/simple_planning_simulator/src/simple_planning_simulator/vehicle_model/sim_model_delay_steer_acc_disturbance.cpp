// Copyright 2021 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc_disturbance.hpp"

#include "simple_planning_simulator/vehicle_model/sim_model_delay_steer_acc.hpp"

#include <algorithm>

SimModelDelaySteerAcc_Dist::SimModelDelaySteerAcc_Dist(float64_t vx_lim,
                                                       float64_t steer_lim,
                                                       float64_t vx_rate_lim,
                                                       float64_t steer_rate_lim,
                                                       float64_t wheelbase,
                                                       float64_t acc_delay,
                                                       float64_t acc_time_constant,
                                                       float64_t steer_delay,
                                                       float64_t steer_time_constant)

        : SimModelInterface(6 /* dim x */, 2 /* dim u */),
          MIN_TIME_CONSTANT(0.03),
          vx_lim_(vx_lim),
          vx_rate_lim_(vx_rate_lim),
          steer_lim_(steer_lim),
          steer_rate_lim_(steer_rate_lim),
          wheelbase_(wheelbase),
          acc_delay_(acc_delay),
          acc_time_constant_(std::max(acc_time_constant, MIN_TIME_CONSTANT)),
          steer_delay_(steer_delay),
          steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT))
{
    // initializeInputQueue(dt);
}

float64_t SimModelDelaySteerAcc_Dist::getX()
{
    return state_(IDX::X);
}

float64_t SimModelDelaySteerAcc_Dist::getY()
{
    return state_(IDX::Y);
}

float64_t SimModelDelaySteerAcc_Dist::getYaw()
{
    return state_(IDX::YAW);
}

float64_t SimModelDelaySteerAcc_Dist::getVx()
{
    return state_(IDX::VX);
}

float64_t SimModelDelaySteerAcc_Dist::getVy()
{
    return 0.0;
}

float64_t SimModelDelaySteerAcc_Dist::getAx()
{
    return state_(IDX::ACCX);
}

float64_t SimModelDelaySteerAcc_Dist::getWz()
{
    return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}

float64_t SimModelDelaySteerAcc_Dist::getSteer()
{
    return state_(IDX::STEER);
}

void SimModelDelaySteerAcc_Dist::update(const float64_t &dt)
{
    Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);

//    acc_input_queue_.push_back(input_(IDX_U::ACCX_DES));
//    delayed_input(IDX_U::ACCX_DES) = acc_input_queue_.front();
//    acc_input_queue_.pop_front();
//
//    steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
//    delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
//    steer_input_queue_.pop_front();

    const double raw_acc_command = input_(IDX_U::ACCX_DES);
    const double raw_steer_command = input_(IDX_U::STEER_DES);

    // --------- DISTURBANCE GENERATOR MODIFICATIONS -------------------------


    delayed_input(IDX_U::ACCX_DES) = raw_acc_command;// acc_delayed + acc_slope_dist;
    delayed_input(IDX_U::STEER_DES) = raw_steer_command; //steer_delayed;


    // --------- DISTURBANCE GENERATOR MODIFICATIONS -------------------------
//    delayed_input(IDX_U::STEER_DES) = steer_delayed;
//    delayed_input(IDX_U::ACCX_DES) = acc_delayed + acc_slope_dist;


    // --------- DISTURBANCE GENERATOR MODIFICATIONS -------------------------
    // Integrate the state
    updateRungeKutta(dt, delayed_input);

    state_(IDX::VX) = std::max(-vx_lim_, std::min(state_(IDX::VX), vx_lim_));

    // Debugs
    ns_utils::print("In the vehicle disturbance vehicle model \n");
}

//void SimModelDelaySteerAcc_Dist::initializeInputQueue(const float64_t &dt)
//{
//    auto acc_input_queue_size = static_cast<size_t>(round(acc_delay_ / dt));
//    acc_input_queue_.resize(acc_input_queue_size);
//    std::fill(acc_input_queue_.begin(), acc_input_queue_.end(), 0.0);
//
//    auto steer_input_queue_size = static_cast<size_t>(round(steer_delay_ / dt));
//    steer_input_queue_.resize(steer_input_queue_size);
//    std::fill(steer_input_queue_.begin(), steer_input_queue_.end(), 0.0);
//}

Eigen::VectorXd SimModelDelaySteerAcc_Dist::calcModel(
        const Eigen::VectorXd &state, const Eigen::VectorXd &input)
{
    auto sat = [](float64_t val, float64_t u, float64_t l)
    { return std::max(std::min(val, u), l); };

    const float64_t vel = sat(state(IDX::VX), vx_lim_, -vx_lim_);
    const float64_t acc = sat(state(IDX::ACCX), vx_rate_lim_, -vx_rate_lim_);
    const float64_t yaw = state(IDX::YAW);
    const float64_t steer = state(IDX::STEER);
    const float64_t acc_des = sat(input(IDX_U::ACCX_DES), vx_rate_lim_, -vx_rate_lim_);
    const float64_t steer_des = sat(input(IDX_U::STEER_DES), steer_lim_, -steer_lim_);
    float64_t steer_rate = -(steer - steer_des) / steer_time_constant_;
    steer_rate = sat(steer_rate, steer_rate_lim_, -steer_rate_lim_);

    Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
    d_state(IDX::X) = vel * cos(yaw);
    d_state(IDX::Y) = vel * sin(yaw);
    d_state(IDX::YAW) = vel * std::tan(steer) / wheelbase_;
    d_state(IDX::VX) = acc;
    d_state(IDX::STEER) = steer_rate;
    d_state(IDX::ACCX) = -(acc - acc_des) / acc_time_constant_;

    return d_state;
}