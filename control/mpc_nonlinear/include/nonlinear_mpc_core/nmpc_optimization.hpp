/*
 * Copyright 2021 - 2022 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef NONLINEAR_MPC_CORE__NMPC_OPTIMIZATION_HPP_
#define NONLINEAR_MPC_CORE__NMPC_OPTIMIZATION_HPP_

#include <Eigen/StdVector>
#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>
#include "Eigen/Core"
#include "Eigen/SparseCore"
#include "active_model.hpp"
#include "data_and_parameter_container.hpp"
#include "osqp_google/osqp++.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"
/**
 * @brief The decision variables of OSQP is the concatenated vectors [x;u].
 *
 * */
namespace ns_opt
{
    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    struct OsqpMatDims
    {
        OsqpMatDims() : nxK{STATE_DIM * K}, nuK{INPUT_DIM * K}, njK{INPUT_DIM * (K - 1)}
        {

          /*nxK = STATE_DIM * K;
          // We have a dummy last control action if we use FOH the last control is used.
          nuK = INPUT_DIM * K;        // !<@brief B = [0, 0, 0 ; B, 0, 0; 0, B, 0;... ]
          njK = INPUT_DIM * (K - 1);  // !<@brief Jerk input uj = du.
          */

          dcs_xu = nxK + nuK + njK;  // !<@brief size of P and q.
          Pxdim = nxK;
          Pudim = nuK;
          Pjerkdim = njK;

          Pdim = dcs_xu;
          qdim = dcs_xu;

          Aeq_xdim_row = nxK;  // !<@brief Only Xs are constrained with the equality

          // constraints. X = AX + BU + Z
          Aeq_xdim_col = (nxK + nuK + njK);  // !<@brief one extra zero column for the inputs.

          Aeq_ujdim_row = njK;
          Aeq_ujdim_col = (nxK + nuK + njK);

          Aineq_start_row = nxK + njK;  // Aeq. ends at this point.

          /**
           * @brief Total dimension of the constraint matrix.
           * {Aeq_dim + Aineq_dim}
           * */

          Arow_dim = (nxK + njK) + (nxK + nuK + njK);
          Acol_dim = (nxK + nuK + njK);
        }

        // ~OsqpMatDims() = default;

        // Where starts where ends.
        size_t nxK;  // !<@brief K is the number of prediction horizon, nx * K.
        size_t nuK;  // !<@brief nu * K.
        size_t njK;  // !<@brief state, control, jerk. nu * (K-2).

        // Parameters
        size_t dcs_xu{};    // !<@brief decision variables.
        size_t Pxdim{};     // !<@brief segment corresponds to x-variable
        size_t Pudim{};     // !<@brief segment corresponds to u-variable
        size_t Pjerkdim{};  // !<@brief segment corresponds to jerk variable.
        size_t Pdim{};      // !<@brief Px + Pu + Pj.

        size_t qdim{};          // !<@brief optimization vector in OSQP
        size_t Aeq_xdim_row{};  // !<@brief number of rows in equality constraints corresponds to x.
        size_t Aeq_xdim_col{};  // !<@brief number of cols in

        size_t Aeq_ujdim_row{};  // !<@brief u[k+1] - u[k] = uj
        size_t Aeq_ujdim_col{};

        // total A = [Aeq:Aineq] // equality and inequality constraint matrix vertically stacked.
        size_t Arow_dim{};
        size_t Acol_dim{};
        size_t Aineq_start_row{};
        size_t Aineq_start_col{};
    };

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    class OptimizationProblemOSQP
    {
    public:
        // Type defs.
        using triplet_type = Eigen::Triplet<double>;

        OptimizationProblemOSQP();

        OptimizationProblemOSQP(OptimizationProblemOSQP const &other);

        OptimizationProblemOSQP &operator=(OptimizationProblemOSQP const &other);

        //        OptimizationProblemOSQP(OptimizationProblemOSQP &&other) noexcept;
        //        OptimizationProblemOSQP &operator=(OptimizationProblemOSQP &&other) noexcept;
        ~OptimizationProblemOSQP() = default;

        // Getters.
        [[nodiscard]] bool isInitialized() const
        { return is_initialized_; }

        [[nodiscard]] bool isUpdated() const
        { return is_updated_; }

        osqp::OsqpExitCode testSolver();

        void getSolution(ns_data::ParamsOptimization const &params_optimization, Model::trajectory_data_t &td);

        // Setters.
        bool setUPOSQP_useTriplets(Model::state_vector_t const &x0,
                                   ns_data::data_nmpc_core_type_t const &data_nmpc,
                                   ns_data::ParamsOptimization const &param_opt);

        bool updateOSQP(ns_data::data_nmpc_core_type_t const &data_nmpc,
                        ns_data::ParamsOptimization const &param_opt);

        // Solve
        osqp::OsqpExitCode solve();

        [[nodiscard]] double getObjectiveValue() const
        { return osqp_solver_.objective_value(); }

    private:
        // Cost and Constraint matrix dimensions.
        OsqpMatDims<STATE_DIM, INPUT_DIM, K> osqp_dims_{};

        // OSQP members.
        osqp::OsqpSettings osqp_settings_{};
        osqp::OsqpInstance osqp_instance_{};
        osqp::OsqpSolver osqp_solver_{};

        // Objective matrix. [Px, Pu, Pjerk] symmetrical.
        Eigen::SparseMatrix<double> Pcost_{};   // Cost matrix.
        Eigen::SparseMatrix<double> Aconst_{};  // Constraint matrix.

        // Keep the following triplets which are constants and to be reused regularly.
        // There are the constant parts of the OSQP constraint matrix.
        std::vector<Eigen::Triplet<double>> triplets_Sx_;              // !<@brief Scaling matrix.
        std::vector<Eigen::Triplet<double>> triplets_Aequality_jerk_;  // !<@brief Equality constraint.
        std::vector<Eigen::Triplet<double>> triplets_Aineq_;           // !<@brief Inequality constraint.

        bool is_initialized_{false};  // !<-brief Initialization of optimization structure.
        bool is_updated_{false};      // !<-@brief whether the matrices are updated or not.
    };

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::OptimizationProblemOSQP()
      : osqp_dims_{OsqpMatDims<STATE_DIM, INPUT_DIM, K>()}
    {
      // Prepare P and A sparse matrix members.
      // Penalty matrix.
      Pcost_ = Eigen::SparseMatrix<double>(osqp_dims_.Pdim, osqp_dims_.Pdim);
      Aconst_ = Eigen::SparseMatrix<double>(osqp_dims_.Arow_dim, osqp_dims_.Acol_dim);  // Constraint matrix.
    }

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::OptimizationProblemOSQP(const OptimizationProblemOSQP &other)
      : osqp_dims_(other.osqp_dims_),
        osqp_settings_{other.osqp_settings_},
        osqp_instance_{osqp::OsqpInstance()},
        osqp_solver_{osqp::OsqpSolver()},
        Pcost_(other.Pcost_),
        Aconst_(other.Aconst_),
        triplets_Sx_(other.triplets_Sx_),
        triplets_Aequality_jerk_(other.triplets_Aequality_jerk_),
        triplets_Aineq_(other.triplets_Aineq_)
    {
    }

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K> &
    OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::operator=(const OptimizationProblemOSQP &other)
    {
      if (&other != this)
      {
        osqp_dims_ = other.osqp_dims_;

        // Eigen matrices.
        Pcost_ = other.Pcost_;
        Aconst_ = other.Aconst_;

        triplets_Sx_ = other.triplets_Sx_;
        triplets_Aequality_jerk_ = other.triplets_Aequality_jerk_;
        triplets_Aineq_ = other.triplets_Aineq_;

        // OSQP classes.
        osqp_settings_ = other.osqp_settings_;
        osqp_instance_ = osqp::OsqpInstance();
        osqp_solver_ = osqp::OsqpSolver();
      }
      return *this;
    }

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    bool OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::setUPOSQP_useTriplets(Model::state_vector_t const &x0,
                                                                                 ns_data::data_nmpc_core_type_t const &data_nmpc,
                                                                                 ns_data::ParamsOptimization const &param_opt)
    {
      auto &&params_optimization = param_opt;
      auto &&discretization_data = data_nmpc.discretization_data;

      // Prepare P and A sparse matrix members.
      Pcost_ = Eigen::SparseMatrix<double>(osqp_dims_.Pdim, osqp_dims_.Pdim);  // !<@brief Penalty matrix.
      Aconst_ = Eigen::SparseMatrix<double>(osqp_dims_.Arow_dim, osqp_dims_.Acol_dim);  // !<@briefConstraint matrix.

      /**
       *  To prevent KKT conditioning error we add values in the diagonals of P matrix at the end of this method.
       * */

      auto Psmall_diags = Eigen::SparseMatrix<double>(osqp_dims_.Pdim, osqp_dims_.Pdim);
      Psmall_diags.setIdentity();

      std::vector<triplet_type> triplets_P;
      std::vector<triplet_type> triplets_A;

      triplets_P.reserve(osqp_dims_.Pdim);  // We know the dimension.

      // Get triplets from Q, QN, R, Rjerk.
      // if item abs is greater than eps, the item  is assumed to be nonzero.
      double const &eps_triplet_zero = std::numeric_limits<double>::epsilon() / 2.;

      auto const &triplets_Q = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(params_optimization.Q), eps_triplet_zero);

      auto const &triplets_QN = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(params_optimization.QN), eps_triplet_zero);

      auto const &triplets_R = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(params_optimization.R), eps_triplet_zero);

      auto const &triplets_Rj = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(params_optimization.Rj), eps_triplet_zero);

      // Q, QN, R, Rj starts at the different cols.
      auto col_startR = K * STATE_DIM;  // !<@brief R columns start from in the cost matrix.
      auto col_startRj = K * STATE_DIM + K * INPUT_DIM;  // !<@brief int the cost matrix.

      // Prepare an Identity matrix to be used in Jerk matrices.
      Eigen::MatrixXd uId(INPUT_DIM, INPUT_DIM);
      uId.setIdentity();
      auto triplets_uId = ns_eigen_utils::ToTriplets(uId, eps_triplet_zero);

      for (size_t k = 0; k < K - 1; ++k)
      {
        // Set P triplets. P start index is [k * STATE_DIM, k * STATE_DIM] for

        auto const &&rowQ = k * STATE_DIM;
        auto const &&colQ = k * STATE_DIM;

        std::transform(triplets_Q.cbegin(), triplets_Q.cend(), std::back_inserter(triplets_P),
                       [&rowQ, &colQ](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowQ;
                           auto &&tcol = titem[1] + colQ;
                           auto &&val = titem[2];

                           return triplet_type(trow, tcol, val);
                       });

        // Insert Triplets of R
        auto rowR = k * INPUT_DIM + col_startR;
        auto colR = k * INPUT_DIM + col_startR;

        std::transform(
          triplets_R.cbegin(), triplets_R.cend(), std::back_inserter(triplets_P),
          [&rowR, &colR](auto const &titem)
          {
              auto &&trow = titem[0] + rowR;
              auto &&tcol = titem[1] + colR;
              auto &&val = titem[2];

              return triplet_type(trow, tcol, val);
          });

        // Insert Triplets of Rj ------------------------------
        auto rowRj = k * INPUT_DIM + col_startRj;
        auto colRj = k * INPUT_DIM + col_startRj;

        std::transform(triplets_Rj.cbegin(), triplets_Rj.cend(), std::back_inserter(triplets_P),
                       [&rowRj, &colRj](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowRj;
                           auto &&tcol = titem[1] + colRj;
                           auto &&val = titem[2];

                           return triplet_type(trow, tcol, val);
                       });

        // A_equality on the main diagonal set -Sx.

        auto rowSx = k * STATE_DIM;
        auto colSx = k * STATE_DIM;

        auto triplet_list_S = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(params_optimization.Sx), 0.0);

        std::transform(triplet_list_S.cbegin(), triplet_list_S.cend(), std::back_inserter(triplets_A),
                       [&rowSx, &colSx, this](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowSx;
                           auto &&tcol = titem[1] + colSx;
                           auto &&val = -1 * titem[2];  // -Sx

                           // Set the class member for the triplet_Sx.
                           auto &&triplet_temp = triplet_type(trow, tcol, val);
                           this->triplets_Sx_.template emplace_back(triplet_temp);

                           return triplet_temp;
                       });

        // Aequality on the diagonal k=-1 set A*Sx.

        auto rowA = (k + 1) * STATE_DIM;
        auto colA = k * STATE_DIM;

        auto &&ASx = Eigen::MatrixXd(discretization_data.A[k] * params_optimization.Sx);
        auto triplet_list_A = ns_eigen_utils::ToTriplets(ASx, eps_triplet_zero);

        std::transform(triplet_list_A.cbegin(), triplet_list_A.cend(), std::back_inserter(triplets_A),
                       [&rowA, &colA](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowA;
                           auto &&tcol = titem[1] + colA;
                           auto &&val = titem[2];

                           return triplet_type(trow, tcol, val);
                       });

        // Insert B into Aequality.
        auto rowB = (k + 1) * STATE_DIM;
        auto colB = K * STATE_DIM + k * INPUT_DIM;

        auto &&BSu = discretization_data.B[k] * params_optimization.Su;  // Apply scaling to B.
        auto triplet_list_B = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(BSu), eps_triplet_zero);

        std::transform(triplet_list_B.cbegin(), triplet_list_B.cend(), std::back_inserter(triplets_A),
                       [&rowB, &colB](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowB;
                           auto &&tcol = titem[1] + colB;
                           auto &&val = titem[2];

                           return triplet_type(trow, tcol, val);
                       });

        // Insert Bj triplets [-1, 1; -1, 1; ...]. Takes difference of u, d(u) =  u1-u0.
        // This jerk equalities is just below the state equalities X=AX + BU + z.
        // [AX + BU + z.; 0 d(u) du]

        // Set -Id on the main diagonals.  // Just after X = AX + BU + z
        auto rowBjmain = K * STATE_DIM + k * INPUT_DIM;
        auto colBjmain = K * STATE_DIM + k * INPUT_DIM;

        std::transform(triplets_uId.cbegin(), triplets_uId.cend(), std::back_inserter(triplets_A),
                       [&rowBjmain, &colBjmain, this](auto &titem)
                       {
                           auto &&trow = titem[0] + rowBjmain;
                           auto &&tcol = titem[1] + colBjmain;
                           auto &&val = -1 * titem[2];  // -Iu at the main diagonal.

                           // Keep these also in the class member.
                           auto &&triplet_temp = triplet_type(trow, tcol, val);
                           this->triplets_Aequality_jerk_.template emplace_back(triplet_temp);

                           return triplet_temp;
                       });

        // Set Id on the diagonal k=1.
        // Id is just one input colon block ahead of -Id.
        auto rowBj_k1 = K * STATE_DIM + k * INPUT_DIM;
        auto colBj_k1 = K * STATE_DIM + (k + 1) * INPUT_DIM;

        std::transform(triplets_uId.cbegin(), triplets_uId.cend(), std::back_inserter(triplets_A),
                       [&rowBj_k1, &colBj_k1, this](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowBj_k1;
                           auto &&tcol = titem[1] + colBj_k1;
                           auto &&val = titem[2];  // -Iu at the main diagonal.

                           // Keep these also in the class member.
                           auto &&triplet_temp = triplet_type(trow, tcol, val);
                           this->triplets_Aequality_jerk_.template emplace_back(triplet_temp);

                           return triplet_temp;
                       });

        // Set Identity for picking Bj = -I *du variable in Aequality.
        // This is below Aeq [AX BU 0;0 d(u) du]
        auto rowdu = K * STATE_DIM + k * INPUT_DIM;
        auto coldu = K * STATE_DIM + K * INPUT_DIM + k * INPUT_DIM;

        std::transform(triplets_uId.cbegin(), triplets_uId.cend(), std::back_inserter(triplets_A),
                       [&rowdu, &coldu, this](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowdu;
                           auto &&tcol = titem[1] + coldu;
                           auto &&val = -titem[2];  // d(u) - du = 0, here we set du.

                           // Keep these also in the class member.
                           auto &&triplet_temp = triplet_type(trow, tcol, val);
                           this->triplets_Aequality_jerk_.template emplace_back(triplet_temp);

                           return triplet_temp;
                       });
      }

      // Insert QN at (K-1). -----------------------------------------------

      auto row = (K - 1) * STATE_DIM;  // Last Q start positions
      auto col = (K - 1) * STATE_DIM;

      std::transform(triplets_QN.cbegin(), triplets_QN.cend(), std::back_inserter(triplets_P),
                     [&row, &col](auto const &titem)
                     {
                         auto &&trow = titem[0] + row;
                         auto &&tcol = titem[1] + col;
                         auto &&val = titem[2];

                         return triplet_type(trow, tcol, val);
                     });

      // Insert R at (K-1).---------------------------------------------

      row = col_startR + (K - 1) * INPUT_DIM;  // last R start position
      col = col_startR + (K - 1) * INPUT_DIM;

      std::transform(triplets_R.cbegin(), triplets_R.cend(), std::back_inserter(triplets_P),
                     [&row, &col](auto const &titem)
                     {
                         auto &&trow = titem[0] + row;
                         auto &&tcol = titem[1] + col;
                         auto &&val = titem[2];

                         return triplet_type(trow, tcol, val);
                     });

      // -------------------- Insert -Sx at K-1 ----------------

      auto rowSx = (K - 1) * STATE_DIM;
      auto colSx = (K - 1) * STATE_DIM;

      auto triplet_list_S = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(params_optimization.Sx), 0.0);

      std::transform(triplet_list_S.cbegin(), triplet_list_S.cend(), std::back_inserter(triplets_A),
                     [&rowSx, &colSx, this](auto const &titem)
                     {
                         auto &&trow = titem[0] + rowSx;
                         auto &&tcol = titem[1] + colSx;
                         auto &&val = -1.0 * titem[2];  // -Sx

                         // Set the class member for the triplet_Sx.
                         auto &&triplet_temp = triplet_type(trow, tcol, val);
                         this->triplets_Sx_.template emplace_back(triplet_temp);

                         return triplet_temp;
                     });

      // ----------- Set Ainequality at one go -----------------------
      auto row_aineq = osqp_dims_.Aineq_start_row;  // Inequality matrix start at this row.
      auto col_aineq = osqp_dims_.Aineq_start_col;

      // We need to set lower and upper bound for inequality here.
      Eigen::MatrixXd Aineq(osqp_dims_.Pdim, osqp_dims_.Pdim);
      Aineq.setIdentity();

      auto triplet_Aineq = ns_eigen_utils::ToTriplets(Aineq, eps_triplet_zero, row_aineq, col_aineq);

      std::transform(triplet_Aineq.cbegin(), triplet_Aineq.cend(), std::back_inserter(triplets_A),
                     [this](auto const &titem)
                     {
                         auto &&trow = titem[0];
                         auto &&tcol = titem[1];
                         auto &&val = titem[2];

                         // Set the class member for the triplet_Sx.
                         auto &&triplet_temp = triplet_type(trow, tcol, val);
                         this->triplets_Aineq_.template emplace_back(triplet_temp);

                         return triplet_temp;
                     });

      // ---------------------- Set Equality Constraints RHS----------
      // Optimization operates on the scaled variables.

      osqp_instance_.lower_bounds.resize(osqp_dims_.Arow_dim);  // low < A [x;u;du] < up
      osqp_instance_.upper_bounds.resize(osqp_dims_.Arow_dim);

      osqp_instance_.lower_bounds.setZero();
      osqp_instance_.upper_bounds.setZero();

      // auto xhat0 = trajectory_data_t::ScaleInv_x(mpc_to_osqp_params_.Sx_inv,
      // mpc_to_osqp_params_.Cx, x0);

      auto &&Cx = params_optimization.Cx;
      auto &&Cu = params_optimization.Cu;

      osqp_instance_.lower_bounds.template segment<STATE_DIM>(0) = -(x0 - Cx);
      osqp_instance_.upper_bounds.template segment<STATE_DIM>(0) = -(x0 - Cx);

      for (size_t k = 1; k < K; ++k)
      {
        auto &&zk =
          discretization_data.z.at(k - 1) + discretization_data.A.at(k - 1) * Cx
          + discretization_data.B.at(k - 1) * Cu - Cx;

        osqp_instance_.lower_bounds.template segment<STATE_DIM>(k * STATE_DIM) = -zk;
        osqp_instance_.upper_bounds.template segment<STATE_DIM>(k * STATE_DIM) = -zk;
      }

      // ----------- Upper and Lower Bounds in Aineq constraint ---------------

      // Controls start position in Aineq.
      // after we set state bounds along K*nx
      auto row_inq_u = row_aineq + K * STATE_DIM;  // Inequalities start at for controls at this row.
      auto row_inq_j = row_inq_u + K * INPUT_DIM;  // Inequalities start at for jerks at this row.

      for (size_t k = 0; k < K - 1; ++k)
      {
        // Set state bounds.
        osqp_instance_.upper_bounds.segment<STATE_DIM>(row_aineq + k * STATE_DIM) = params_optimization.xupper_scaled;

        osqp_instance_.lower_bounds.segment<STATE_DIM>(row_aineq + k * STATE_DIM) = params_optimization.xlower_scaled;

        // Set control bounds.
        osqp_instance_.upper_bounds.segment<INPUT_DIM>(row_inq_u + k * INPUT_DIM) = params_optimization.uupper_scaled;

        osqp_instance_.lower_bounds.segment<INPUT_DIM>(row_inq_u + k * INPUT_DIM) = params_optimization.ulower_scaled;

        // Set jerk bounds.
        osqp_instance_.upper_bounds.segment<INPUT_DIM>(row_inq_j + k * INPUT_DIM) << kInfinity, kInfinity;

        osqp_instance_.lower_bounds.segment<INPUT_DIM>(row_inq_j + k * INPUT_DIM) << -kInfinity, -kInfinity;
      }

      // Set state bounds at (K-1)
      osqp_instance_.upper_bounds.segment<STATE_DIM>(
        row_aineq + (K - 1) * STATE_DIM) = params_optimization.xupper_scaled;

      osqp_instance_.lower_bounds.segment<STATE_DIM>(
        row_aineq + (K - 1) * STATE_DIM) = params_optimization.xlower_scaled;

      // Set control bounds at (K-1)
      osqp_instance_.upper_bounds.segment<INPUT_DIM>(
        row_inq_u + (K - 1) * INPUT_DIM) = params_optimization.uupper_scaled;

      osqp_instance_.lower_bounds.segment<INPUT_DIM>(
        row_inq_u + (K - 1) * INPUT_DIM) = params_optimization.ulower_scaled;

      // DEBUG
      //  ns_utils::print("State Inequality Bounds in SetupOSQP:");
      //
      //  auto xbounds = ns_eigen_utils::hstack<double>(
      //    osqp_instance_.lower_bounds.middleRows(row_aineq, K * STATE_DIM),
      //    osqp_instance_.upper_bounds.middleRows(row_aineq, K * STATE_DIM));
      //
      //  ns_utils::print("Control Inequality Bounds in SetupOSQP:");
      //  ns_utils::print("Row start of input inequalities u : ", row_inq_u);
      //  auto ubounds = ns_eigen_utils::hstack<double>(
      //    osqp_instance_.lower_bounds.middleRows(row_inq_u, K * INPUT_DIM),
      //    osqp_instance_.upper_bounds.middleRows(row_inq_u, K * INPUT_DIM));
      //  ns_utils::print("\nBounds of controls in OSQP update:");
      //  ns_eigen_utils::printEigenMat(ubounds);

      // end of DEBUG

      // Set the Objective vector.

      osqp_instance_.objective_vector.resize(osqp_dims_.Pdim);
      osqp_instance_.objective_vector.setZero();
      osqp_instance_.objective_matrix.setZero();

      // Fill P and A from the triplets.
      // ------------------------------------------------------------------------
      Pcost_.template setFromTriplets(triplets_P.cbegin(), triplets_P.cend());
      Aconst_.template setFromTriplets(triplets_A.cbegin(), triplets_A.cend());

      // Initialize the solver.

      osqp_settings_.warm_start = params_optimization.osqp_warm_start;
      osqp_settings_.polish = params_optimization.osqp_polishing;
      osqp_settings_.scaling = params_optimization.osqp_scaling;
      osqp_settings_.max_iter = params_optimization.osqp_max_iters;
      osqp_settings_.polish_refine_iter = params_optimization.osqp_polish_iters;
      // osqp_settings_.time_limit = params_optimization.osqp_time_limit;
      osqp_settings_.scaled_termination = params_optimization.osqp_scaled_termination;

      // Set accuracy.
      osqp_settings_.eps_abs = params_optimization.osqp_eps_abs;
      osqp_settings_.eps_rel = params_optimization.osqp_eps_rel;

      // Set verbosity.
      osqp_settings_.verbose = params_optimization.osqp_verbose;

      // Set OSQP instance - Pmatrix must be upper diagonal.
      // On the diagonals of P we add a small value to force it not to behave ill-conditioned.
      // osqp_instance_.objective_matrix = Pcost_;
      auto &&EPS = std::numeric_limits<double>::epsilon();
      osqp_instance_.objective_matrix = (Pcost_ + EPS * Psmall_diags).triangularView<Eigen::Upper>();
      osqp_instance_.constraint_matrix = Aconst_;

      auto status = osqp_solver_.Init(osqp_instance_, osqp_settings_);
      ns_utils::print("Is the solver initialized : ", status());

      is_initialized_ = osqp_solver_.IsInitialized();

      // DEBUG
      ns_utils::print("\nOsqp Pdim cost matrix : ", osqp_dims_.Pdim);
      ns_utils::print("Osqp Adim constraint matrix : ", osqp_dims_.Acol_dim);

      //  ns_utils::print("\nP - cost matrix : ");
      //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Pcost_.toDense()));
      //  ns_utils::print("\nA - constraint matrix : ");
      //  ns_eigen_utils::printEigenMat(Eigen::MatrixXd(Aconst_.toDense()));
      // end of debug.

      return is_initialized_;

    }

/**
 *  @brief dimensions that defines where the the specific decision variable is located.
 *  Aeq = [A, B_u, B_jerk=0] on the diagonals.
 *  Aeqjerk = [0, Iu1_u0, I] which states that du = ujerk.
 *  Aineq   = diag ([x, u, uj]) ones.
 *
 *  @tparam STATE_DIM number of states in the active model equations,
 *  @tparam INPUT_DIM number of inputs in the active model equations,
 *  @tparam K number of NMPC prediction steps.
 * */

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    osqp::OsqpExitCode OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::testSolver()
    {
      osqp::OsqpExitCode exit_code = osqp_solver_.Solve();

      ns_utils::print("\nEXIT CODE example \n");
      ns_utils::print(ToString(exit_code));

      ns_utils::print("\nOptimization Results : \n");
      Eigen::VectorXd optimal_solution = osqp_solver_.primal_solution();
      ns_eigen_utils::printEigenMat(optimal_solution.topRows(10));

      ns_utils::print(" ... ");
      ns_eigen_utils::printEigenMat(optimal_solution.bottomRows(10));

      return exit_code;
    }

/**
 *  @brief At each step of the optimization, the reference states in the OSQP optimization vector q, initial states
 *  constraint and motion equations constraints (A, B, z) must be updated. In the osqp interface, since sparsity
 *  structure in the system matrices could change, we need to re-create the optimization problem. We can't use
 *  update functionalities of OSQP class for the constraint matrices due to the sparsity structure.
 *
 *  Arguments;
 *  @param params_ptr pointer to the common parameters
 *  @param trajectory_data  reference trajectories at which the system is linearized.
 *  @params discretization_data  discretisized matrices are stored in.
 *  @params target_references   target states (currently target Vx -- scaled).
 *
 * */
    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    bool OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::updateOSQP(ns_data::data_nmpc_core_type_t const &data_nmpc,
                                                                      ns_data::ParamsOptimization const &param_opt)
    {
      auto &&params_optimization = param_opt;
      auto &&discretization_data = data_nmpc.discretization_data;
      auto &&trajectory_data = data_nmpc.trajectory_data;
      auto &&target_references = data_nmpc.target_reference_states_and_controls;

      // Re-instantiate the Constraint matrix.
      Aconst_ = Eigen::SparseMatrix<double>(osqp_dims_.Arow_dim, osqp_dims_.Acol_dim);

      // Prepare a new objective vector. q in OSQP user_guide.
      Eigen::VectorXd new_objective_vector(osqp_dims_.Pdim, 1);
      new_objective_vector.setZero();

      std::vector<triplet_type> triplets_A;

      // if item abs is greater than eps, the item is assumed to be nonzero.
      double EPS = std::numeric_limits<double>::epsilon();

      // Extract Q, QN to use in the new objective vector q.
      auto &&Q = params_optimization.Q;
      auto &&QN = params_optimization.QN;

      // Set new As and Bs in the constraint matrix.
      for (size_t k = 0; k < K - 1; ++k)
      {
        // Aequality on the diagonal k=-1 set A*Sx.

        auto rowA = (k + 1) * STATE_DIM;
        auto colA = k * STATE_DIM;

        auto &&ASx = Eigen::MatrixXd(discretization_data.A[k] * params_optimization.Sx);
        auto triplet_list_A = ns_eigen_utils::ToTriplets(ASx, EPS);

        // ns_utils::print("\nASx\n");
        // ns_eigen_utils::printEigenMat(ASx);
        //
        // ns_utils::print("\nA\n");
        // ns_eigen_utils::printEigenMat(discretization_data.A[k]);

        std::transform(triplet_list_A.cbegin(), triplet_list_A.cend(), std::back_inserter(triplets_A),
                       [&rowA, &colA](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowA;
                           auto &&tcol = titem[1] + colA;
                           auto &&val = titem[2];

                           return triplet_type(trow, tcol, val);
                       });

        // Set the objective vector.
        // Insert B into Aequality.
        auto rowB = (k + 1) * STATE_DIM;
        auto colB = K * STATE_DIM + k * INPUT_DIM;

        auto &&BSu = discretization_data.B[k] * params_optimization.Su;  // Apply scaling to B.
        auto triplet_list_B = ns_eigen_utils::ToTriplets(Eigen::MatrixXd(BSu), EPS);

        std::transform(triplet_list_B.cbegin(), triplet_list_B.cend(), std::back_inserter(triplets_A),
                       [&rowB, &colB](auto const &titem)
                       {
                           auto &&trow = titem[0] + rowB;
                           auto &&tcol = titem[1] + colB;
                           auto &&val = titem[2];

                           return triplet_type(trow, tcol, val);
                       });

        // Set the optimization vector.
        // ns_eigen_utils::printEigenMat(-1 * Q * target_references.X[k]);
        // If we scale outside, we can use this line.
        //  new_objective_vector.template segment<STATE_DIM>(k * STATE_DIM) =
        //  -1 * Q *  target_references.X[k];

        // scale the targets in [-1, 1]
        auto
          &xref_hat =
          params_optimization.Sx_inv * (target_references.X[k] * data_nmpc.feedforward_speed_set_point_scale -
                                        params_optimization.Cx);

        new_objective_vector.template segment<STATE_DIM>(k * STATE_DIM) = -1 * Q * xref_hat;

        // ns_eigen_utils::printEigenMat(Eigen::MatrixXd(params_optimization.Sx_inv));
        // ns_utils::print("Target speed set in optimization ---------- : ", xref_hat(6));

        // ns_utils::print("\nTarget State in OSQP Class : \n");
        // ns_eigen_utils::printEigenMat(-1 * Q * target_references.X[k]);
      }

      // Set the final reference value in the objective vector.
      // Set the optimization vector. If scaled outside of this method.
      // new_objective_vector.template segment<STATE_DIM>((K - 1) * STATE_DIM)
      // = -1 * QN * target_references.X[K - 1];

      auto const
        &xref_hat =
        params_optimization.Sx_inv * (target_references.X[K - 1] * data_nmpc.feedforward_speed_set_point_scale -
                                      params_optimization.Cx);  // scale the targets in [-1, 1]

      new_objective_vector.template segment<STATE_DIM>((K - 1) * STATE_DIM) = -1 * QN * xref_hat;

      // ns_utils::print("\nOptimization Vector q : \n");
      // ns_eigen_utils::printEigenMat(new_objective_vector);

      // Update triplet_A constant items by copying the constant triplets into the
      // new constraint matrix.
      std::copy(triplets_Sx_.cbegin(), triplets_Sx_.cend(), std::back_inserter(triplets_A));
      std::copy(triplets_Aequality_jerk_.cbegin(), triplets_Aequality_jerk_.cend(),
                std::back_inserter(triplets_A));

      std::copy(triplets_Aineq_.cbegin(), triplets_Aineq_.cend(), std::back_inserter(triplets_A));

      // ---------------------- Set Equality Constraints RHS---------------------------------
      // Optimization operates on the scaled variables. The motion system matrices are scaled.

      auto &&Cx = params_optimization.Cx;  // State centering vector.
      auto &&Cu = params_optimization.Cu;  // Control centering vector.

      auto &&x0 = trajectory_data.X[0];  // take out the initial states.

      osqp_instance_.lower_bounds.template segment<STATE_DIM>(0) = -(x0 - Cx);
      osqp_instance_.upper_bounds.template segment<STATE_DIM>(0) = -(x0 - Cx);

      for (size_t k = 1; k < K; ++k)
      {
        auto &&zk = discretization_data.z[k - 1] + discretization_data.A[k - 1] * Cx +
                    discretization_data.B[k - 1] * Cu - Cx;

        osqp_instance_.lower_bounds.template segment<STATE_DIM>(k * STATE_DIM) = -zk;
        osqp_instance_.upper_bounds.template segment<STATE_DIM>(k * STATE_DIM) = -zk;
      }

      //  ns_utils::print("\nLower Bounds of controls in OSQP update:");
      //  ns_eigen_utils::printEigenMat(osqp_instance_.lower_bounds.middleRows(K * STATE_DIM,
      //  (K - 1) * INPUT_DIM));
      //
      //  ns_utils::print("\nUpper Bounds of controls in OSQP update:");
      //  ns_eigen_utils::printEigenMat(osqp_instance_.upper_bounds.middleRows(K * STATE_DIM,
      //                   (K - 1) * INPUT_DIM));

      //  ns_utils::print("\nLower Bounds of state in OSQP update:");
      //  ns_eigen_utils::printEigenMat(osqp_instance_.lower_bounds.topRows(K * STATE_DIM));
      //  ns_utils::print("\nUpper Bounds of state in OSQP update:");
      //  ns_eigen_utils::printEigenMat(osqp_instance_.upper_bounds.topRows(K * STATE_DIM));

      // Set the equality constraint vector.-------------------------------------------

      // Fill A from the triplets. -------------------------------------------
      Aconst_.template setFromTriplets(triplets_A.cbegin(), triplets_A.cend());

      // Re-instantiation.
      osqp_instance_.constraint_matrix = Aconst_;
      osqp_instance_.objective_vector = new_objective_vector;

      auto status = osqp_solver_.Init(osqp_instance_, osqp_settings_);
      is_updated_ = osqp_solver_.IsInitialized();

      //  ns_utils::print("\nSolver Status in updateOSQP", status());
      //  ns_utils::print("\nIs Solver Initialized in updateOSQP?",
      //  osqp_solver_.IsInitialized() ? "Ok" : "Not OK");

      // DEBUG
      // ns_utils::print("In the updateOSQP method : ");
      // end of Debug

      return is_updated_;
    }

    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    osqp::OsqpExitCode OptimizationProblemOSQP<STATE_DIM, INPUT_DIM, K>::solve()
    {
      osqp::OsqpExitCode exit_code = osqp_solver_.Solve();
      return exit_code;
    }

/**
 * @brief OSQP is a quadratic problem solver and it treats solution variables as a single optimizaiton vector;
 * [x, y, du].
 * In this problem settings, we have solution variables states-x, controls-u and jerks-du are unknowns. The index
 * location of these variables are known. We extract the solutions for each of the variable using the their
 * corresponding indices.
 *
 * */
    template<size_t STATE_DIM, size_t INPUT_DIM, size_t K>
    void OptimizationProblemOSQP<STATE_DIM,
      INPUT_DIM, K>::getSolution(ns_data::ParamsOptimization const &params_optimization,
                                 trajectory_data_t &td)
    {
      /**
       * states = x = [xw, yw, psi, s, ey, epxi, v, delta].
       * controls = u = [ax; steering_rate].
       */

      Eigen::VectorXd &&optimal_solution = osqp_solver_.primal_solution();
      auto row_cont_start = K * STATE_DIM;

      // We push the first state at the end of the vectors so that we can skip
      // shifting the matrices. Solutions are all scaled. We need to scale back.

      auto const &Sx = params_optimization.Sx;
      auto const &Su = params_optimization.Su;
      auto const &Cx = params_optimization.Cx;
      auto const &Cu = params_optimization.Cu;

      for (size_t k = 0; k < K; ++k)
      {
        // counts until k = K-1
        auto const &&xsolk = Model::state_vector_t(optimal_solution.template segment<STATE_DIM>(k * STATE_DIM));

        td.X[k] = Sx * xsolk + Cx;  // fills until k=K-2
        auto const &&usolk = Model::input_vector_t(optimal_solution.segment<INPUT_DIM>(k * INPUT_DIM + row_cont_start));

        td.U[k] = Su * usolk + Cu;
      }

      // // Put xsol0, usol0 to end. To avoid shifting operation again.
      // auto &&xsolk = Model::state_vector_t(optimal_solution.template
      // segment<STATE_DIM>(0 * STATE_DIM));
      // trajectory_data.X[K - 1] = Sx * xsolk + Cx;
      // completes with the last element in K-dim vector.
      //
      // auto &&usolk = Model::input_vector_t(optimal_solution.segment<INPUT_DIM>(0 * INPUT_DIM +
      // row_cont_start));
      // trajectory_data.U[K - 1] = Su * usolk + Cu;

      // DEBUG
      // ns_utils::print("In get solution: OSQP computed errors : ");
      // Get trajectories as a matrix and print for debugging purpose.
      // auto &&Xtemp = ns_eigen_utils::getTrajectory(trajectory_data.X);
      // auto &&Utemp = ns_eigen_utils::getTrajectory(trajectory_data.U);
      //
      // ns_utils::print("\nSolved OSQP trajectories ey epsi : ");
      // //ns_eigen_utils::printEigenMat(Xtemp.transpose().middleCols(4, 2));
      // [x, y, psi, s, ey, epsi, v, delta]
      // ns_eigen_utils::printEigenMat(Xtemp.transpose());
      //
      // ns_utils::print("\nSolved OSQP trajectories U : ");
      // ns_eigen_utils::printEigenMat(Utemp.transpose());

      // end of debug
    }
}  // namespace ns_opt

#endif  // NONLINEAR_MPC_CORE__NMPC_OPTIMIZATION_HPP_
