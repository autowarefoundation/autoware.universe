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
// #include <Python.h>
// #include <dlfcn.h>
#include <pybind11/embed.h>
namespace py = pybind11;

SimModelPymodels::SimModelPymodels(
  double vx_lim, double steer_lim, double vx_rate_lim, double steer_rate_lim, double wheelbase,
  double dt, double vx_delay, double vx_time_constant, double steer_delay,
  double steer_time_constant, double steer_dead_band)
: SimModelInterface(5 /* dim x */, 2 /* dim u */),
  MIN_TIME_CONSTANT(0.03),
  vx_lim_(vx_lim),
  vx_rate_lim_(vx_rate_lim),
  steer_lim_(steer_lim),
  steer_rate_lim_(steer_rate_lim),
  wheelbase_(wheelbase),
  vx_delay_(vx_delay),
  vx_time_constant_(std::max(vx_time_constant, MIN_TIME_CONSTANT)),
  steer_delay_(steer_delay),
  steer_time_constant_(std::max(steer_time_constant, MIN_TIME_CONSTANT)),
  steer_dead_band_(steer_dead_band)
{
  
  // Initialize python library
  //dlopen("libpython3.10.so", RTLD_GLOBAL | RTLD_NOW); // Manually load libpython3.10.so as we need it for python.h.

  
  /*
  More about the line above here:
    https://stackoverflow.com/questions/60719987/embedding-python-which-uses-numpy-in-c-doesnt-work-in-library-dynamically-loa
    https://mail.python.org/pipermail/new-bugs-announce/2008-November/003322.html
    https://stackoverflow.com/questions/67891197/ctypes-cpython-39-x86-64-linux-gnu-so-undefined-symbol-pyfloat-type-in-embedd
    https://man7.org/linux/man-pages/man3/dlopen.3.html
  */
  // Py_Initialize();
  // PyObject *py_module, *py_module_dict, *python_class;

  // char *model_save_path = (char*)"/home/tomas/f1tenth/ws_testing/base_model_save"; 

  // Import python module
  // py_module = PyImport_Import(PyUnicode_FromString((char*)"control_analysis_pipeline.model.base_model.base_model_simple_steering_hysteresis"));

  // if (py_module == nullptr) {
  //     PyErr_Print();
  //     return;
  // }

  //  py_module_dict = PyModule_GetDict(py_module); 
  //  if (py_module_dict == nullptr) {
  //      PyErr_Print();
  //      return;
  //  }

  // python_class = PyDict_GetItemString(py_module_dict, "SimpleSteeringHyst");
  // if (python_class == nullptr) {
  //     PyErr_Print();
  //     return;
  // }

  // if (PyCallable_Check(python_class)) {
  //     steer_base_model = PyObject_CallObject(python_class, nullptr);
  //     Py_DECREF(python_class);
  // } else {
  //     Py_DECREF(python_class);
  //     return;
  // }


  // TODO call function to get model signal names
  // From the names create a mapping table for correctly pass arguments to the python models

  // PyObject *py_sig_state_names = PyObject_CallMethodObjArgs(steer_base_model, PyUnicode_FromString((char*)"get_sig_state_names"), NULL);

  // char *my_result;

  // PyObject * temp_bytes = PyUnicode_AsEncodedString(PyList_GetItem(py_sig_state_names, 0), "UTF-8", "strict");
  // my_result = PyBytes_AS_STRING(temp_bytes);
  // my_result = strdup(my_result);
  // Py_DECREF(temp_bytes);

  // wcstombs(buffer, PyList_GetItem(py_sig_state_names, 0), sizeof(buffer));

  // int len_ = PyList_Size(py_sig_state_names);
  // std::cout << "STATE NAMES:  -" <<  my_result << "-" << std::endl;
  // std::cout << "STEER:  " << len_ << std::endl;
  // std::cout << "STEER:  " << strcmp(my_result, "STEER") << std::endl;
  // std::cout << "STEER:  " << strcmp(my_result, STATE_NAMES[4]) << std::endl;
  // std::cout << "STEERR:  " << strcmp(my_result, "STEERR") << std::endl;


  // Load model params
  // PyObject *load_params_succ = PyObject_CallMethodObjArgs(steer_base_model, PyUnicode_FromString((char*)"load_params"), PyUnicode_FromString(model_save_path), NULL);

  // if(!PyBool_Check(load_params_succ)){
  //     return;
  // }

  // Reset the python model after loading the parameter file
  // PyObject_CallMethodObjArgs(steer_base_model, PyUnicode_FromString((char*)"reset"), NULL);
  

  // Set number model states and inputs
  // num_states_steering = 1;
  // num_inputs_steering = 1;

  // // Initialize state and input vectors
  // state_steering = Eigen::VectorXd::Zero(num_states_steering);
  // input_steering = Eigen::VectorXd::Zero(num_inputs_steering);
  
  
  // py_state_steering = PyList_New(num_states_steering);
  // py_input_steering = PyList_New(num_inputs_steering);

  // for (int state_idx = 0; state_idx < num_states_steering; state_idx++){
  //     PyList_SetItem(py_state_steering, state_idx, PyFloat_FromDouble(state_steering(state_idx)));
  // }

  // for (int input_idx = 0; input_idx < num_inputs_steering; input_idx++){
  //     PyList_SetItem(py_input_steering, input_idx, PyFloat_FromDouble(input_steering(input_idx)));
  // }

  py::scoped_interpreter guard{}; // start the interpreter and keep it alive

  py::print("Testing py::print"); // use the Python API

  // py::module_ sys = py::module_::import("control_analysis_pipeline.model.base_model.base_model_simple_steering_hysteresis");

  py::module_ torch = py::module_::import("torch");

  // py::object result = torch.attr("add")(1, 2);

  std::cout << dt << std::endl;
  std::cout << "OKAAAAAYYYYYYY" << std::endl;
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
  return state_(IDX::VX) * std::tan(state_(IDX::STEER)) / wheelbase_;
}
double SimModelPymodels::getSteer()
{
  return state_(IDX::STEER);
}
void SimModelPymodels::update(const double & dt)
{
  Eigen::VectorXd delayed_input = Eigen::VectorXd::Zero(dim_u_);
  //std::cout << dt << std::endl;

  // Model the delay of speed
  vx_input_queue_.push_back(input_(IDX_U::VX_DES));
  delayed_input(IDX_U::VX_DES) = vx_input_queue_.front();
  vx_input_queue_.pop_front();

  // Model the delay of steering  --  TODO remove this after implementing python models
  steer_input_queue_.push_back(input_(IDX_U::STEER_DES));
  delayed_input(IDX_U::STEER_DES) = steer_input_queue_.front();
  steer_input_queue_.pop_front();

  // Simulate the system (car kinematics and dynamics)
  // do not use deadzone_delta_steer (Steer IF does not exist in this model)
  updateRungeKutta(dt, delayed_input);

  // Simulate the steering system using python model

  // TODO: define inputs to the model in a more general way
  /*
  Following inputs to the steering system can be used:

    - IDX_U::STEER_DES
    - IDX::VX
  */
  // input_steering(0) = 0;


  // for (int input_idx = 0; input_idx < num_inputs_steering; input_idx++){
  //     PyList_SetItem(py_input_steering, input_idx, PyFloat_FromDouble(input_steering(input_idx)));
  // }

  // Call forward() of the python model
  // PyObject *value = PyObject_CallMethodObjArgs(steer_base_model, PyUnicode_FromString((char*)"forward"), py_input_steering, py_state_steering, NULL);
  // py_state_steering = PyTuple_GetItem(value, 0);  // The first output of the model forward is the next state
  // PyObject *py_delayed_input = PyTuple_GetItem(value, 1);  // The second output of the model forward is delayed input
  
  // State vector from python to C++
  // for (int state_idx = 0; state_idx < num_states_steering; state_idx++){
  //     state_steering(state_idx) = PyFloat_AsDouble(PyList_GetItem(PyList_GetItem(py_state_steering, 0), state_idx));
  // }
  //std::cout << "Required steer:  " << input_(IDX_U::STEER_DES) << "   State steering:  " << state_steering(0) << std::endl;
  
  // TODO: define outputs of the model in a more general way
  /*
  Following outputs from the steering system can be used:
    - IDX::STEER
  */
  state_(IDX::STEER) = 0;

  // Calculate
  current_ax_ = (input_(IDX_U::VX_DES) - prev_vx_) / dt;
  prev_vx_ = input_(IDX_U::VX_DES);
}

Eigen::VectorXd SimModelPymodels::calcModel(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input)
{
  auto sat = [](double val, double u, double l) { return std::max(std::min(val, u), l); };

  const double vx = sat(state(IDX::VX), vx_lim_, -vx_lim_);
  const double steer = sat(state(IDX::STEER), steer_lim_, -steer_lim_);
  const double yaw = state(IDX::YAW);
  const double delay_input_vx = input(IDX_U::VX_DES);
  const double delay_vx_des = sat(delay_input_vx, vx_lim_, -vx_lim_);
  const double vx_rate = sat(-(vx - delay_vx_des) / vx_time_constant_, vx_rate_lim_, -vx_rate_lim_);


  Eigen::VectorXd d_state = Eigen::VectorXd::Zero(dim_x_);
  d_state(IDX::X) = vx * cos(yaw);
  d_state(IDX::Y) = vx * sin(yaw);
  d_state(IDX::YAW) = vx * std::tan(steer) / wheelbase_;
  d_state(IDX::VX) = vx_rate;
  d_state(IDX::STEER) = 0.0;  //Use python models to update steer

  return d_state;
}
