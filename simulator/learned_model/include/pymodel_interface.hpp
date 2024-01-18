#ifndef LEARNED_MODEL__PYMODEL_INTERFACE_HPP_
#define LEARNED_MODEL__PYMODEL_INTERFACE_HPP_

class PymodelInterface
 {  
 public:
    // virtual ~PymodelInterface() = 0;  //  <-- build fails after uncommenting this

    /**
     * @brief get names of inputs of python model
     */
    virtual std::vector<char*> getInputNames() = 0;

    /**
     * @brief get names of states of python model
     */
    virtual std::vector<char*> getStateNames() = 0;

    /**
     * @brief create a map from model signal vector to python model inputs
     * @param [in] signal_vec_names names of signals in model signal vector
     */
    virtual void mapInputs(std::vector<char*> signals_vec_names) = 0;
    
    /**
     * @brief create a map from python outputs to model signal vector
     * @param [in] signal_vec_names names of signals in model signal vector
     */
    virtual void mapOutputs(std::vector<char*> signals_vec_names) = 0;

    /**
     * @brief calculate the next state of this submodule
     * @param [in] model_signals_vec values of signals in model signal vector
     * @param [in] model_signals_vec_next values of signals in model signal vector to update
     */
    virtual std::vector<double> getNextState(std::vector<double> model_signals_vec, std::vector<double> model_signals_vec_next) = 0;
};

#endif  // LEARNED_MODEL__PYMODEL_INTERFACE_HPP_