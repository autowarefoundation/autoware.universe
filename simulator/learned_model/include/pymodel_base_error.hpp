#ifndef LEARNED_MODEL__SIM_PYMODEL_BASE_ERROR_HPP_
#define LEARNED_MODEL__SIM_PYMODEL_BASE_ERROR_HPP_

#include "pymodel_simple_model.hpp"
#include "pymodel_interface.hpp"


class SimPymodelBaseError : public PymodelInterface
{
private:
    PymodelSimpleModel base_model;
    PymodelSimpleModel error_model;

public:
  SimPymodelBaseError(std::tuple<char*, char*, char*> base_desc, std::tuple<char*, char*, char*> error_desc) 
  : base_model(std::get<0>(base_desc), std::get<1>(base_desc), std::get<2>(base_desc)),
    error_model(std::get<0>(error_desc), std::get<1>(error_desc), std::get<2>(error_desc))
  {
  }

  /**
   * @brief create a map from model signal vector to python model inputs
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  void mapInputs(std::vector<char*> signals_vec_names) override
  {
    base_model.mapInputs(signals_vec_names);
    error_model.mapInputs(signals_vec_names);
  }

  /**
   * @brief create a map from python outputs to model signal vector
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  void mapOutputs(std::vector<char*> signals_vec_names) override
  {
    base_model.mapOutputs(signals_vec_names);
    error_model.mapOutputs(signals_vec_names);
  }

  /**
   * @brief get names of inputs of python model
   */
  std::vector<char*> getInputNames() override
  {
    std::vector<char*> base_input_name = base_model.getInputNames();
    std::vector<char*> error_input_name = error_model.getInputNames();
    base_input_name.insert(base_input_name.end(), error_input_name.begin(), error_input_name.end());
    return base_input_name;
  }
  
  /**
   * @brief get names of states of python model
   */
  std::vector<char*> getStateNames() override
  {
    std::vector<char*> base_state_name = base_model.getStateNames();
    std::vector<char*> error_state_name = error_model.getStateNames();
    base_state_name.insert(base_state_name.end(), error_state_name.begin(), error_state_name.end());
    return base_state_name;
  }

  /**
   * @brief calculate the next state of this submodule
   * @param [in] model_signals_vec values of signals in model signal vector
   * @param [in] model_signals_vec_next values of signals in model signal vector to update
   */
  std::vector<double> getNextState(std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next) override
  {
    std::vector<double> base_next_state = base_model.getNextState(model_signals_vec, model_signals_vec_next);
    std::vector<double> base_next_state_delayed = base_model.signals_w_delayed_input;
    std::vector<double> next_state_error(model_signals_vec.size());
    std::fill(next_state_error.begin(), next_state_error.end(), 0.0);
    std::vector<double> error_correction = error_model.getNextState(base_next_state_delayed, next_state_error);  // need to change input in all_variable input to delayed one from base model
    for (size_t i = 0; i < base_next_state.size(); i++) base_next_state[i] += error_correction[i];
    return base_next_state;
  }
};


#endif  // LEARNED_MODEL__SIM_PYMODEL_BASE_ERROR_HPP_