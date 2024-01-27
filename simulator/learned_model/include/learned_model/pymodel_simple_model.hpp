#ifndef LEARNED_MODEL__PYMODEL_SIMPLE_MODEL_HPP_
#define LEARNED_MODEL__PYMODEL_SIMPLE_MODEL_HPP_

#include "learned_model/model_connections_helpers.hpp"
#include "learned_model/pymodel_interface.hpp"

#include <pybind11/embed.h>
#include <pybind11/stl.h>

namespace py = pybind11;

/**
 * @class SimModelPymodels
 * @brief This class is an interface between C++ and python models.
 */
class PymodelSimpleModel : public PymodelInterface
{
private:
  std::string pymodel_import_name;

  int num_inputs_py;
  int num_outputs_py;

  py::object py_model_class;

  std::vector<int>
    map_sig_vec_to_pyin;  // index in "map_sig_vec_to_pyin" is index in "py_inputs" and value in
                          // "map_sig_vec_to_pyin" is index in "signals_vec_names"
  std::vector<int>
    map_pyout_to_sig_vec;  // index in "map_pyout_to_sig_vec" is index in "pymodel_outputs" and
                           // value in "map_pyout_to_sig_vec" is index in "signals_vec_names"

  std::vector<char *> pymodel_input_names;
  std::vector<char *> pymodel_state_names;

public:
  /**
   * @brief constructor
   * @param [in] pymodel_import_name_ path to python model
   * @param [in] param_file_path path to saved parameter file of the python sub-model
   * @param [in] py_class_name name of the python class
   */
  PymodelSimpleModel(std::string pymodel_import_name_, std::string param_file_path, std::string py_class_name)
  {
    // Import model class
    pymodel_import_name = pymodel_import_name_;
    if (!pymodel_import_name.empty()) {
      // Import python module
      py::module_ imported_module = py::module_::import(pymodel_import_name.c_str());
      // Initialize model class from imported module
      py_model_class = imported_module.attr(py_class_name.c_str())();
    } else {
      // TODO throw exception/error
      return;
    }

    // Load model parameters and reset the model
    if (!param_file_path.empty()) {
      py::object load_params_succ = py_model_class.attr("load_params")(param_file_path.c_str());
      py_model_class.attr("reset")();
      std::cout << "Params loaded" << std::endl;
    } else {
      // TODO warning that using default model params
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

    std::cout << "PymodelInterface import name:  " << pymodel_import_name << std::endl;
  }

  /**
   * @brief calculate the next state of a python model
   * @param [in] model_signals_vec all available inputs from PSIM
   */
  std::vector<double> getNextState(
    std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next) override
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

  /**
   * @brief get names of inputs of python model
   */
  std::vector<char *> getInputNames() override { return pymodel_input_names; }

  /**
   * @brief get names of states of python model
   */
  std::vector<char *> getStateNames() override { return pymodel_state_names; }

  /**
   * @brief create a map from model signal vector to python model inputs
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  void mapInputs(std::vector<char *> signals_vec_names) override
  {
    // index in "map_sig_vec_to_pyin" is index in "py_inputs" and value in "map_sig_vec_to_pyin" is
    // index in "signals_vec_names"
    map_sig_vec_to_pyin = createConnectionsMap(signals_vec_names, pymodel_input_names);
  }

  /**
   * @brief create a map from python outputs to model signal vector
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  void mapOutputs(std::vector<char *> signals_vec_names) override
  {
    // index in "map_pyout_to_sig_vec" is index in "pymodel_outputs" and value in
    // "map_pyout_to_sig_vec" is index in "signals_vec_names"
    map_pyout_to_sig_vec = createConnectionsMap(signals_vec_names, pymodel_state_names);
  }

private:
  std::vector<int> createConnectionsMap(
    std::vector<char *> connection_names_1, std::vector<char *> connection_names_2)
  {
    std::vector<int> connection_map;
    // index in "connection_map" is index in "connection_names_2" and value in "connection_map" is
    // index in "connection_names_1"
    for (auto name_2 : connection_names_2) {
      int mapped_idx = -1;  // -1 means that we cannot create a connection between some signals
      for (size_t NAME_1_IDX = 0; NAME_1_IDX < connection_names_1.size(); NAME_1_IDX++) {
        if (strcmp(name_2, connection_names_1[NAME_1_IDX]) == 0) {  // 0 means strings are the same
                                                                    // and we can create connection
          mapped_idx = NAME_1_IDX;
          break;
        }
      }
      connection_map.push_back(mapped_idx);
    }
    return connection_map;
  }

  std::vector<double> fillVectorUsingMap(
    std::vector<double> vector1, std::vector<double> vector2, std::vector<int> map, bool inverse)
  {
    // index in "map" is index in "vector1" and value in "map" is index in "vector2"
    // inverse = 0 from 1 -> 2; inverse = 1 from 2 -> 1
    for (size_t idx = 0; idx < map.size(); idx++) {
      if (map[idx] == -1) continue;
      inverse ? vector1[idx] = vector2[map[idx]] : vector2[map[idx]] = vector1[idx];
    }
    return inverse ? vector1 : vector2;
  }
};

#endif  // LEARNED_MODEL__PYMODEL_SIMPLE_MODEL_HPP_