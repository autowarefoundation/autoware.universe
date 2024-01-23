#ifndef LEARNED_MODEL__SIM_PYMODEL_INTERCONNECTED_MODEL_
#define LEARNED_MODEL__SIM_PYMODEL_INTERCONNECTED_MODEL_

// #include "pymodel_base_error.hpp"
#include "pymodel_interface.hpp"
#include "pymodel_simple_model.hpp"
#include "model_connections_helpers.hpp"

#include <dlfcn.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <algorithm>

namespace py = pybind11;

class InterconnectedModel
{
  int num_signals;

  std::vector<double> model_signals_vec;
  std::vector<char *> signals_vec_names;

  std::vector<std::unique_ptr<PymodelInterface>> submodels;

  // index in "map_in_to_sig_vec" is index in "py_inputs" and value in "map_in_to_sig_vec" is index in "all_variables_names"
  std::vector<int> map_in_to_sig_vec;  

  // index in "map_sig_vec_to_out" is index in "pymodel_outputs" and value in "map_sig_vec_to_out" is index in "all_variables_names"
  std::vector<int> map_sig_vec_to_out;  

public:
  py::scoped_interpreter guard{};  // start the interpreter and keep it alive

  /**
   * @brief constructor
   */
  InterconnectedModel()
  {
    // Initialize python library
    // Manually load libpython3.10.so as we need it for python.h.
    dlopen("libpython3.10.so", RTLD_GLOBAL | RTLD_NOW);
    /*
    More about the line above here:
      https://stackoverflow.com/questions/60719987/embedding-python-which-uses-numpy-in-c-doesnt-work-in-library-dynamically-loa
      https://mail.python.org/pipermail/new-bugs-announce/2008-November/003322.html
      https://stackoverflow.com/questions/67891197/ctypes-cpython-39-x86-64-linux-gnu-so-undefined-symbol-pyfloat-type-in-embedd
      https://man7.org/linux/man-pages/man3/dlopen.3.html
    */
  }

private:
  std::vector<int> createConnectionsMap(std::vector<char *> connection_names_1, std::vector<char *> connection_names_2)
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

  void mapInputs(std::vector<char *> in_names)
  {
    // index in "map_in_to_sig_vec" is index in "in_names" and value in "map_in_to_sig_vec" is index in "signals_vec_names"
    map_in_to_sig_vec = createConnectionsMap(signals_vec_names, in_names);
  }

  void mapOutputs(std::vector<char *> out_names)
  {
    // index in "map_sig_vec_to_out" is index in "out_names" and value in "map_sig_vec_to_out" is index in "signals_vec_names"
    map_sig_vec_to_out = createConnectionsMap(signals_vec_names, out_names);
  }

  void addNamesToSigVec(const std::vector<char *> & names)
  {
    for (char * name : names) {
      if (std::find(signals_vec_names.begin(), signals_vec_names.end(), name) == signals_vec_names.end()) {
        signals_vec_names.push_back(name);
      }
    }
  }
  
  void getSignalNames()
  {
    for (auto & submodel : submodels) {
      addNamesToSigVec(submodel->getInputNames());
      addNamesToSigVec(submodel->getStateNames());
    }
  }

public:
  /**
   * @brief automatically create connections between PSIM and all of the sub-models
   * @param [in] in_names string names of inputs available from PSIM
   * @param [in] out_names string names of outputs required by PSIM
   */
  void generateConnections(std::vector<char *> in_names, std::vector<char *> out_names)
  {
    getSignalNames();
    num_signals = signals_vec_names.size();
    for (int i = 0; i < num_signals; i++) model_signals_vec.push_back(0);

    for (auto & submodel : submodels) {
      submodel->mapInputs(signals_vec_names);
      submodel->mapOutputs(signals_vec_names);
    }
    mapInputs(in_names);
    mapOutputs(out_names);
  }

  /**
   * @brief add a sub-model consisting of base + error model
   * @param [in] submodel_desc descriptor of the sub-model
   */
  void addSubmodel(std::tuple<char *, char *, char *> submodel_desc)
  {
    const auto [lib_path, param_path, class_name] = submodel_desc;
    auto new_model = new PymodelSimpleModel(lib_path, param_path, class_name);
    submodels.push_back(std::unique_ptr<PymodelSimpleModel>(new_model));
  }

  // /**
  //  * @brief add a sub-model consisting of base + error sub-model
  //  * @param [in] base_desc descriptor of base sub-model
  //  * @param [in] error_desc descriptor of error sub-model
  //  */
  // void addSubmodelBaseError(
  //   std::tuple<char *, char *, char *> base_desc, std::tuple<char *, char *, char *> error_desc)
  // {
  //   submodels.push_back(
  //     std::unique_ptr<SimPymodelBaseError>(new SimPymodelBaseError(base_desc, error_desc)));
  // }

  /**
   * @brief set a new model state if it was changed using PSIM interface (mainly position and orientation)
   * @param [in] new_state new state set by PSIM
   */
  void initState(std::vector<double> new_state)
  {
    bool state_changed_externally = false;

    for (size_t PSIM_STATE_IDX = 0; PSIM_STATE_IDX < new_state.size(); PSIM_STATE_IDX++) {
      if (abs(model_signals_vec[map_sig_vec_to_out[PSIM_STATE_IDX]] - new_state[PSIM_STATE_IDX]) > 1e-6) {
        state_changed_externally = true;
        break;
      }
    }

    if (state_changed_externally) {
      std::cout << "Reseting model" << std::endl;

      std::fill(model_signals_vec.begin(), model_signals_vec.end(), 0.0);

      for (size_t PSIM_STATE_IDX = 0; PSIM_STATE_IDX < new_state.size(); PSIM_STATE_IDX++) {
        model_signals_vec[map_sig_vec_to_out[PSIM_STATE_IDX]] = new_state[PSIM_STATE_IDX];
      }
    }
  }

  /**
   * @brief compute next step of the PSIM model using python sub-models
   * @param [in] psim_input vector of input values provided by PSIM
   */
  std::vector<double> updatePymodel(std::vector<double> psim_input)
  {
    // map input to vector of all variables
    for (size_t PSIM_INPUT_IDX = 0; PSIM_INPUT_IDX < psim_input.size(); PSIM_INPUT_IDX++) {
      model_signals_vec[map_in_to_sig_vec[PSIM_INPUT_IDX]] = psim_input[PSIM_INPUT_IDX];
    }

    // Compute forward pass through all models (order should not matter)
    std::vector<double> model_signals_vec_next(num_signals);
    for (auto & submodel : submodels) {
      model_signals_vec_next = submodel->getNextState(model_signals_vec, model_signals_vec_next);
    }

    // Map vector of all variables to
    std::vector<double> psim_next_state;
    for (size_t PSIM_STATE_IDX = 0; PSIM_STATE_IDX < map_sig_vec_to_out.size(); PSIM_STATE_IDX++) {
      psim_next_state.push_back(model_signals_vec_next[map_sig_vec_to_out[PSIM_STATE_IDX]]);
    }

    // Update vector of all variables
    model_signals_vec = model_signals_vec_next;

    return psim_next_state;
  }
};

#endif  // LEARNED_MODEL__SIM_PYMODEL_INTERCONNECTED_MODEL_