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

#include "learned_model/pymodel_interconnected_model.hpp"

#include <algorithm>

SimModelPymodels::SimModelPymodels(double dt) : SimModelInterface(7 /* dim x */, 2 /* dim u */)
{
  // TODO this should be in config file not hardcoded here
  // Think of a way how to differentiate between "simple" model and "base + error" model
  std::vector<std::tuple<char *, char *, char *>> model_descriptors = {
    {(char *)"control_analysis_pipeline.autoware_models.vehicle.kinematic", (char *)nullptr,
     (char *)"KinematicModel"},
    {(char *)"control_analysis_pipeline.autoware_models.steering.example_base_error",
     (char *)"$HOME/autoware_model_params/base_model_save", (char *)"BaseError"},
    {(char *)"control_analysis_pipeline.autoware_models.drive.drive_example", (char *)nullptr,
     (char *)"DriveExample"}};

  vehicle.addSubmodel(model_descriptors[0]);
  vehicle.addSubmodel(model_descriptors[1]);
  vehicle.addSubmodel(model_descriptors[2]);

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
  return state_(IDX::VY);
}
double SimModelPymodels::getAx()
{
  return current_ax_;
}
double SimModelPymodels::getWz()
{
  return state_(IDX::YAW_RATE);
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