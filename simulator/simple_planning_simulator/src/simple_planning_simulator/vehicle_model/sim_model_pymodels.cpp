// Copyright 2021 The Autoware Foundation.
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

#include "simple_planning_simulator/vehicle_model/sim_model_pymodels.hpp"

#include <algorithm>

#include "pymodel_interconnected_model.hpp"

SimModelPymodels::SimModelPymodels(double dt)
: SimModelInterface(5 /* dim x */, 2 /* dim u */)
{
  
  // TODO this should be in config file not hardcoded here
  // Think of a way how to differentiate between "simple" model and "base + error" model
  std::vector<std::tuple<char*, char*, char*>> model_descriptors = {
    {
      (char*)"control_analysis_pipeline.autoware_models.vehicle.kinematic",
      (char*)nullptr,
      (char*)"KinematicModel"
    },
    {
      (char*)"control_analysis_pipeline.autoware_models.steering.example_base_error",
      (char*)"$HOME/autoware_model_params/base_model_save",
      (char*)"BaseError"
    },
    {
      (char*)"control_analysis_pipeline.autoware_models.drive.drive_example",
      (char*)nullptr,
      (char*)"DriveExample"
    }
  };
  
  vehicle.addSubmodel(model_desc[0]);
  vehicle.addSubmodel(model_desc[1]); 
  vehicle.addSubmodel(model_desc[2]); 

  vehicle.generateConnections(input_names, state_names);

  std::cout << dt << std::endl;
  std::cout << "Python model loaded successfully " << std::endl;
}

double SimModelPymodels::getX()
{
  return state_(IDX::X);
}
double SimModelPymodels::getY()
{
  return state_(IDX::Y);
}
double SimModelPymodels::getYaw()
{
  return state_(IDX::YAW);
}
double SimModelPymodels::getVx()
{
  return state_(IDX::VX);
}
double SimModelPymodels::getVy()
{
  return 0.0;
}
double SimModelPymodels::getAx()
{
  return current_ax_;
}
double SimModelPymodels::getWz()
{
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / 1.5;
}
double SimModelPymodels::getSteer()
{
  return state_(IDX::STEER);
}
void SimModelPymodels::update(const double & dt)
{
  // Eigen::VectorXd to std::vector<double> for model input
  std::vector<double> vehicle_input_(input_.data(), input_.data() + input_.size());

  // Eigen::VectorXd to std::vector<double> for model state
  std::vector<double> new_state(state_.data(), state_.data() + state_.size());
  // set model state
  vehicle.initState(new_state);

  // model forward
  std::vector<double> vehicle_state_ = vehicle.updatePymodel(vehicle_input_);

  // std::vector<double> to Eigen::VectorXd
  for (size_t i = 0; i < vehicle_state_.size(); i++) state_[i] = vehicle_state_[i];

  // Calculate
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}

Eigen::VectorXd SimModelPymodels::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  // Not used for this model -> we can get rid of this later
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vx = sat(state(IDX::VX), 50, -50);
  const double steer = sat(state(IDX::STEER), 0, -0);
  const double yaw = state(IDX::YAW);
  const double delay_input_vx = input(IDX_U::VX_DES);
  const double delay_vx_des = sat(delay_input_vx, 50, -50);
  const double vx_rate = sat(-(vx - delay_vx_des), 0, -0);


  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = 0 * vx * cos(yaw);
  d_state(IDX::Y) = 0 * vx * sin(yaw);
  d_state(IDX::YAW) = 0 * vx * std::tan(steer) / 1.5;
  d_state(IDX::VX) = 0 * vx_rate;
  d_state(IDX::STEER) = 0.0;  //Use python models to update steer

  return d_state;
}
