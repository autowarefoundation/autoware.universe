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

SimModelPymodels::SimModelPymodels(double dt, 
                                  std::vector<std::string> model_python_paths, 
                                  std::vector<std::string> model_param_paths, 
                                  std::vector<std::string> model_class_names
                                  ) : SimModelInterface(7 /* dim x */, 2 /* dim u */)
{
  for (size_t i = 0; i < model_python_paths.size(); i++){
    std::tuple<std::string, std::string, std::string> descriptor = {
      model_python_paths[i], model_param_paths[i], model_class_names[i]
    };
    vehicle.addSubmodel(descriptor);
  }

  vehicle.generateConnections(input_names, state_names);

  vehicle.dtSet(dt);

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