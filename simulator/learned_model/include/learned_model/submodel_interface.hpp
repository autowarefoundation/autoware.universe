#ifndef LEARNED_MODEL__SUBMODEL_INTERFACE_HPP_
#define LEARNED_MODEL__SUBMODEL_INTERFACE_HPP_

#include <vector>

class SubModelInterface
{
public:
  // virtual ~PymodelInterface() = 0;

  /**
   * @brief set time step of the model
   * @param [in] dt time step
   */
  virtual void dtSet(double dt) = 0;

  /**
   * @brief get names of inputs of python model
   */
  virtual std::vector<char *> getInputNames() = 0;

  /**
   * @brief get names of states of python model
   */
  virtual std::vector<char *> getStateNames() = 0;

  /**
   * @brief create a map from model signal vector to python model inputs
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  virtual void mapInputs(std::vector<char *> signals_vec_names) = 0;

  /**
   * @brief create a map from python outputs to model signal vector
   * @param [in] signal_vec_names names of signals in model signal vector
   */
  virtual void mapOutputs(std::vector<char *> signals_vec_names) = 0;

  /**
   * @brief calculate the next state of this submodule
   * @param [in] model_signals_vec values of signals in model signal vector
   * @param [in] model_signals_vec_next values of signals in model signal vector to update
   */
  virtual std::vector<double> getNextState(
    std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next) = 0;
};

#endif  // LEARNED_MODEL__SUBMODEL_INTERFACE_HPP_