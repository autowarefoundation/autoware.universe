// Copyright 2024 The Autoware Foundation.
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

#include "learned_model/simple_pymodel.hpp"

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <vector>

namespace py = pybind11;

SimplePymodel::SimplePymodel(
  std::string pymodel_import_name_, std::string param_file_path, std::string py_class_name)
{
  // Import model class
  pymodel_import_name = pymodel_import_name_;
  if (!pymodel_import_name.empty()) {
    // Import python module
    py::module_ imported_module = py::module_::import(pymodel_import_name.c_str());
    // Initialize model class from imported module
    py_model_class = imported_module.attr(py_class_name.c_str())();
  } else {
    return;
  }

  // Load model parameters and reset the model
  if (!param_file_path.empty()) {
    py::object load_params_succ = py_model_class.attr("load_params")(param_file_path.c_str());
    py_model_class.attr("reset")();
  }

  // Get string names of states of python model, convert them to C++ string and store them in
  // pymodel_state_names
  py::list pymodel_state_names_ = py_model_class.attr("get_state_names")();
  num_outputs_py = pymodel_state_names_.size();
  for (int STATE_IDX = 0; STATE_IDX < num_outputs_py; STATE_IDX++) {
    pymodel_state_names.push_back(PyBytes_AS_STRING(
      PyUnicode_AsEncodedString(pymodel_state_names_[STATE_IDX].ptr(), "UTF-8", "strict")));
  }

  // Get string names of actions (inputs) of python model, convert them to C++ string and store
  // them in pymodel_input_names
  py::list pymodel_input_names_ = py_model_class.attr("get_action_names")();
  num_inputs_py = pymodel_input_names_.size();
  for (int INPUT_IDX = 0; INPUT_IDX < num_inputs_py; INPUT_IDX++) {
    pymodel_input_names.push_back(PyBytes_AS_STRING(
      PyUnicode_AsEncodedString(pymodel_input_names_[INPUT_IDX].ptr(), "UTF-8", "strict")));
  }
}

std::vector<double> SimplePymodel::getNextState(
  std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next)
{
  // get inputs and states of the python model from the vector of signals
  std::vector<double> py_inputs(num_inputs_py);
  std::vector<double> py_state(num_outputs_py);
  py_inputs = fillVectorUsingMap(py_inputs, model_signals_vec, map_sig_vec_to_pyin, true);
  py_state = fillVectorUsingMap(py_state, model_signals_vec, map_pyout_to_sig_vec, true);

  // forward pass through the base model
  py::tuple res = py_model_class.attr("forward")(py_inputs, py_state);
  std::vector<double> py_state_next = res.cast<std::vector<double>>();

  // map outputs from python model to required outputs
  std::vector<double> next_state =
    fillVectorUsingMap(py_state_next, model_signals_vec_next, map_pyout_to_sig_vec, false);

  return next_state;
}

void SimplePymodel::dtSet(double dt)
{
  py_model_class.attr("dtSet")(dt);
}

std::vector<char *> SimplePymodel::getInputNames()
{
  return pymodel_input_names;
}

std::vector<char *> SimplePymodel::getStateNames()
{
  return pymodel_state_names;
}

void SimplePymodel::mapInputs(std::vector<char *> signals_vec_names)
{
  // index in "map_sig_vec_to_pyin" is index in "py_inputs" and value in "map_sig_vec_to_pyin" is
  // index in "signals_vec_names"
  map_sig_vec_to_pyin = createConnectionsMap(signals_vec_names, pymodel_input_names);
}

void SimplePymodel::mapOutputs(std::vector<char *> signals_vec_names)
{
  // index in "map_pyout_to_sig_vec" is index in "pymodel_outputs" and value in
  // "map_pyout_to_sig_vec" is index in "signals_vec_names"
  map_pyout_to_sig_vec = createConnectionsMap(signals_vec_names, pymodel_state_names);
}
