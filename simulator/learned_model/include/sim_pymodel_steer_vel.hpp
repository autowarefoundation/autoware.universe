#ifndef LEARNED_MODEL__SIM_PYMODEL_STEER_VEL_HPP_
#define LEARNED_MODEL__SIM_PYMODEL_STEER_VEL_HPP_

#include <pybind11/embed.h>
#include <pybind11/stl.h>
namespace py = pybind11;

class SimPymodelSteerVel {
  public:             
    int test_variable = 123;  
    py::scoped_interpreter guard{};
    py::print("Testing py::print LIB"); // use the Python API
};

#endif  // LEARNED_MODEL__SIM_PYMODEL_STEER_VEL_HPP_