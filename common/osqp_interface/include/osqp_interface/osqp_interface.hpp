// Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
//
// Author: Robin Karlsson
//
#ifndef OSQP_INTERFACE__OSQP_INTERFACE_HPP_
#define OSQP_INTERFACE__OSQP_INTERFACE_HPP_

#include <string>
#include <vector>
#include <tuple>
#include <memory>

#include "osqp/osqp.h"

#include "eigen3/Eigen/Core"
namespace osqp
{
struct CSC_Matrix;
const c_float INF = OSQP_INFTY;
}  // namespace osqp

namespace osqp
{
/**
 * Implementation of a native C++ interface for the OSQP solver.
 *
 * The interface takes in the problem formulation as Eigen matrices and vectors, converts these objects into C-style
 * CSC matrices and dynamic arrays, loads the data into the OSQP workspace dataholder, and runs the optimizer.
 *
 * The optimization results are return as a vector tuple by the optimization function.
 *   std::tuple<std::vector<double>, std::vector<double>> result = osqp_interface.optimize();
 *   std::vector<double> param = std::get<0>(result);
 *   double x_0 = param[0];
 *   double x_1 = param[1];
 *
 * The interface can be used in several ways:
 *
 *   1. Initialize the interface WITHOUT data. Load the problem formulation at the optimization call.
 *        osqp_interface = OSQPInterface();
 *        osqp_interface.optimize(P, A, q, l, u);
 *
 *   2. Initialize the interface WITH data.
 *        osqp_interface = OSQPInterface(P, A, q, l, u);
 *        osqp_interface.optimize();
 *
 *   3. WARM START OPTIMIZATION by modifying the problem formulation between optimization runs.
 *        osqp_interface = OSQPInterface(P, A, q, l, u);
 *        osqp_interface.optimize();
 *        while()
 *        {
 *          osqp_interface.updateP(P_new);
 *          osqp_interface.updateA(A_new);
 *          osqp_interface.updateQ(q_new);
 *          osqp_interface.updateL(l_new);
 *          osqp_interface.updateU(u_new);
 *          osqp_interface.optimize();
 *        }
 *
 * Ref: https://osqp.org/docs/solver/index.html
 */
class OSQPInterface
{
private:
  /*****************************
   * OSQP WORKSPACE STRUCTURES
   *****************************/
  std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>> work;
  std::unique_ptr<OSQPSettings> settings;
  std::unique_ptr<OSQPData> data;

  // store last work info since work is cleaned up at every execution to prevent memory leak.
  OSQPInfo latest_work_info;

  // Number of parameters to optimize
  c_int param_n;

  // For destructor to know if matrices P, A are in
  bool problem_in_memory = false;

  // Flag to check if the current work exists
  bool work_initialized = false;

  // Runs the solver on the stored problem.
  std::tuple<std::vector<double>, std::vector<double>, int, int> solve();

  /*****************************
   * DATA CONVERSION FUNCTIONS
   *****************************/
  // Converts problem input matrices to CSC matrix structs.
  CSC_Matrix transformP(const Eigen::MatrixXd & P, int * nonzeros);
  CSC_Matrix transformA(const Eigen::MatrixXd & A);
  // Converts problem input vectors to dynamic arrays.
  double * transformQ(const std::vector<double> & q);
  double * transformL(const std::vector<double> & l);
  double * transformU(const std::vector<double> & u);
  // Converts an Eigen matrix into a CSC matrix struct.
  CSC_Matrix convEigenMatrixToCSCMatrix(const Eigen::MatrixXd A);
  // Converts an Eigen vector matrix into a dynamic array.
  double * convEigenVecToDynFloatArray(const Eigen::MatrixXd x);

  // Exitflag
  c_int exitflag;

  inline bool isEqual(double x, double y);

  static void OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept;

public:
  // Returns a flag for asserting interface condition (Healthy condition: 0).
  c_int getExitFlag(void);

  /****************************
   * INITIALIZATION FUNCTIONS
   ****************************/

  // Initializes the OSQP interface without setting up the problem.
  //
  // Steps:
  //   1. Initializes the OSQP object (incl. settings, data objects).
  //   2. Solver settings (accuracy etc.).
  explicit OSQPInterface(const c_float eps_abs = 1.0e-4, const bool polish = true);

  // Initializes the OSQP solver interface and sets up the problem.
  //
  // Steps:
  //   1. Runs the base constructor (without setting up the problem).
  //   2. Sets up the problem.
  //      2.1. Converts the Eigen matrices to CSC matrices.
  //      2.2. Converts the vectors to dynamic arrays.
  //      2.3. Loads the problem formulation into the OSQP data object and sets up the workspace.
  //
  // Args:
  //   P: (n,n) matrix defining relations between parameters.
  //   A: (m,n) matrix defining parameter constraints relative to the lower and upper bound.
  //   q: (n) vector defining the linear cost of the problem.
  //   l: (m) vector defining the lower bound problem constraint.
  //   u: (m) vector defining the upper bound problem constraint.
  //   eps_abs: Absolute convergence tolerance.
  OSQPInterface(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u, const c_float eps_abs);

  // For freeing dynamic memory used by OSQP's data object.
  ~OSQPInterface();

  /****************
   * OPTIMIZATION
   ****************/
  // Solves the stored convex quadratic program (QP) problem using the OSQP solver.
  //
  // The function returns a tuple containing the solution as two float vectors.
  // The first element of the tuple contains the 'primal' solution.
  // The second element contains the 'lagrange multiplier' solution.
  // The third element contains an integer with solver polish status information.
  //
  // How to use:
  //   1. Generate the Eigen matrices P, A and vectors q, l, u according to the problem.
  //   2. Initialize the interface and set up the problem.
  //        osqp_interface = OSQPInterface(P, A, q, l, u, 1e-6);
  //   3. Call the optimization function.
  //        std::tuple<std::vector<double>, std::vector<double>> result;
  //        result = osqp_interface.optimize();
  //   4. Access the optimized parameters.
  //        std::vector<float> param = std::get<0>(result);
  //        double x_0 = param[0];
  //        double x_1 = param[1];
  std::tuple<std::vector<double>, std::vector<double>, int, int> optimize();

  // Solves convex quadratic programs (QPs) using the OSQP solver.
  //
  // The function returns a tuple containing the solution as two float vectors.
  // The first element of the tuple contains the 'primal' solution.
  // The second element contains the 'lagrange multiplier' solution.
  // The third element contains an integer with solver polish status information.
  //
  // How to use:
  //   1. Generate the Eigen matrices P, A and vectors q, l, u according to the problem.
  //   2. Initialize the interface.
  //        osqp_interface = OSQPInterface(1e-6);
  //   3. Call the optimization function with the problem formulation.
  //        std::tuple<std::vector<double>, std::vector<double>> result;
  //        result = osqp_interface.optimize(P, A, q, l, u, 1e-6);
  //   4. Access the optimized parameters.
  //        std::vector<float> param = std::get<0>(result);
  //        double x_0 = param[0];
  //        double x_1 = param[1];
  std::tuple<std::vector<double>, std::vector<double>, int, int> optimize(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u);

  /**************************
   * DATA-RELATED FUNCTIONS
   **************************/

  // Converts the input data and sets up the workspace object.
  //
  // Args:
  //   P: (n,n) matrix defining relations between parameters.
  //   A: (m,n) matrix defining parameter constraints relative to the lower and upper bound.
  //   q: (n) vector defining the linear cost of the problem.
  //   l: (m) vector defining the lower bound problem constraint.
  //   u: (m) vector defining the upper bound problem constraint.
  c_int initializeProblem(
    const Eigen::MatrixXd & P, const Eigen::MatrixXd & A, const std::vector<double> & q,
    const std::vector<double> & l, const std::vector<double> & u);

  // Updates problem parameters while keeping solution in memory.
  //
  // Args:
  //   P_new: (n,n) matrix defining relations between parameters.
  //   A_new: (m,n) matrix defining parameter constraints relative to the lower and upper bound.
  //   q_new: (n) vector defining the linear cost of the problem.
  //   l_new: (m) vector defining the lower bound problem constraint.
  //   u_new: (m) vector defining the upper bound problem constraint.
  void updateP(const Eigen::MatrixXd & P_new);
  void updateA(const Eigen::MatrixXd & A_new);
  void updateQ(const std::vector<double> & q_new);
  void updateL(const std::vector<double> & l_new);
  void updateU(const std::vector<double> & u_new);
  void updateBounds(const std::vector<double> & l_new, const std::vector<double> & u_new);
  void updateEpsAbs(const double eps_abs);
  void updateEpsRel(const double eps_rel);
  void updateMaxIter(const int iter);
  void updateVerbose(const bool verbose);
  void updateRhoInterval(const int rho_interval);
  void updateRho(const double rho);
  void updateAlpha(const double alpha);

  int getTakenIter() {return static_cast<int>(latest_work_info.iter);}
  std::string getStatusMessage() {return static_cast<std::string>(latest_work_info.status);}
  int getStatus() {return static_cast<int>(latest_work_info.status_val);}
  int getStatusPolish() {return static_cast<int>(latest_work_info.status_polish);}
  double getRunTime() {return static_cast<double>(latest_work_info.run_time);}
  double getObjVal() {return static_cast<double>(latest_work_info.obj_val);}
};

}  // namespace osqp

#endif  // OSQP_INTERFACE__OSQP_INTERFACE_HPP_
