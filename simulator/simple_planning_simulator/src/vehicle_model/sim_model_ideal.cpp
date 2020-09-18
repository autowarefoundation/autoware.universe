
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "simple_planning_simulator/vehicle_model/sim_model_ideal.hpp"

SimModelIdealTwist::SimModelIdealTwist() : SimModelInterface(3 /* dim x */, 2 /* dim u */){};

double SimModelIdealTwist::getX() { return state_(IDX::X); };
double SimModelIdealTwist::getY() { return state_(IDX::Y); };
double SimModelIdealTwist::getYaw() { return state_(IDX::YAW); };
double SimModelIdealTwist::getVx() { return input_(IDX_U::VX_DES); };
double SimModelIdealTwist::getWz() { return input_(IDX_U::WZ_DES); };
double SimModelIdealTwist::getSteer() { return 0.0; };
void SimModelIdealTwist::update(const double & dt) { updateRungeKutta(dt, input_); }
Eigen::VectorXd SimModelIdealTwist::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double yaw = state(IDX::YAW);
  const double vx = input(IDX_U::VX_DES);
  const double wz = input(IDX_U::WZ_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = wz;

  return d_state;
};

SimModelIdealSteer::SimModelIdealSteer(double wheelbase)
: SimModelInterface(3 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase){};

double SimModelIdealSteer::getX() { return state_(IDX::X); };
double SimModelIdealSteer::getY() { return state_(IDX::Y); };
double SimModelIdealSteer::getYaw() { return state_(IDX::YAW); };
double SimModelIdealSteer::getVx() { return input_(IDX_U::VX_DES); };
double SimModelIdealSteer::getWz()
{
  return input_(IDX_U::VX_DES) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
};
double SimModelIdealSteer::getSteer() { return input_(IDX_U::STEER_DES); };
void SimModelIdealSteer::update(const double & dt) { updateRungeKutta(dt, input_); }
Eigen::VectorXd SimModelIdealSteer::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double yaw = state(IDX::YAW);
  const double vx = input(IDX_U::VX_DES);
  const double steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
};

SimModelIdealAccel::SimModelIdealAccel(double wheelbase)
: SimModelInterface(4 /* dim x */, 2 /* dim u */), wheelbase_(wheelbase){};

double SimModelIdealAccel::getX() { return state_(IDX::X); };
double SimModelIdealAccel::getY() { return state_(IDX::Y); };
double SimModelIdealAccel::getYaw() { return state_(IDX::YAW); };
double SimModelIdealAccel::getVx() { return state_(IDX::VX); };
double SimModelIdealAccel::getWz()
{
  return state_(IDX::VX) * std::tan(input_(IDX_U::STEER_DES)) / wheelbase_;
};
double SimModelIdealAccel::getSteer() { return input_(IDX_U::STEER_DES); };
void SimModelIdealAccel::update(const double & dt)
{
  updateRungeKutta(dt, input_);
  if (state_(IDX::VX) < 0) {
    state_(IDX::VX) = 0;
  }
}

Eigen::VectorXd SimModelIdealAccel::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  const double vx = state(IDX::VX);
  const double yaw = state(IDX::YAW);
  const double ax = input(IDX_U::AX_DES);
  const double steer = input(IDX_U::STEER_DES);

  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::VX) = ax;
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;

  return d_state;
};
