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

#ifndef NONLINEAR_MPC_CORE__NMPC_DATA_TRAJECTORY_HPP_
#define NONLINEAR_MPC_CORE__NMPC_DATA_TRAJECTORY_HPP_

#include <tf2/utils.h>
#include <Eigen/Core>
#include <Eigen/StdVector>  // must be before the vector but clang re-orders.
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "utils/nmpc_utils.hpp"
#include "utils_act/act_utils.hpp"
#include "utils_act/act_utils_eigen.hpp"

namespace ns_data
{
    enum class controlSampling_order : size_t
    {
        ZOH = 0,  // zero order hold.
        FOH = 1   // first order hold.
    };

    /**
     * @brief stores raw and smoothed trajectories in the std::vectors received from Autoware planning modules.
     * */
    class MPCdataTrajectoryVectors
    {
    public:
        MPCdataTrajectoryVectors() = default;

        explicit MPCdataTrajectoryVectors(size_t const &traj_size);

        // ~MPCdataTrajectoryVectors() = default;
        std::vector<double> s;          //!< @brief arc-length [m]
        std::vector<double> t;          //!< @brief relative time [s]
        std::vector<double> ax;        //!< @brief implied acceleration [m/s/s]
        std::vector<double> x;          //!< @brief x position x vector
        std::vector<double> y;          //!< @brief y position y vector
        std::vector<double> z;          //!< @brief z position z vector
        std::vector<double> yaw;        //!< @brief yaw pose yaw vector   [rad]
        std::vector<double> vx;         //!< @brief vx velocity vx vector [m/s]
        std::vector<double> curvature;  //!< @brief path curvature [1/m]

        /**
         * @brief push_back for all values.
         * @param msg trajectory message.
         */
        void emplace_back(autoware_auto_planning_msgs::msg::Trajectory const &msg);

        /**
         * @brief set the std::vectors of the corresponding data: i.e x, y, z, yaw.
         * @param coord_letter data variable name; s, x, y, z ...
         * */
        void setTrajectoryCoordinate(char const &coord_letter, std::vector<double> const &data);

        /**
         * @brief adds an additional point to the std:vectors to extend the coordinates at the end.
         * */
        void addExtraEndPoints(double const &avg_mpc_compute_time = 0.0);

        void clear();

        [[nodiscard]] size_t size() const;
    };

    /**
    * @brief contains the trajectory of states and controls as an Eigen vector for each time points in the planning
    * horizon.
    * @tparam Model vehicle model to get dimensions of the states and controls.
    * */
    template<class Model>
    struct TrajectoryData
    {
        typename Model::state_vector_v_t X;  // !<@brief state vector container of Eigen vector.
        typename Model::input_vector_v_t U;  // !<@brief control input vector container.
        typename Model::state_vector_t x_type;
        typename Model::input_vector_t u_type;

        /**
         * @brief initialize the state and control trajectories in the containers.
         * */
        void initializeTrajectory(size_t const &K, double const &dt_step);

        /**
         * @brief returns the size of X-vector container.
         * */
        [[nodiscard]] size_t nX() const;

        /**
         * @brief returns the size of U-vector container.
         * */
        [[nodiscard]] size_t nU() const;

        // Getters. For MPC solutions.
        /**
         * @brief getd the computed controls by NMPC.
         * @param [in] t time at which the control is interpolated.
         * @param [in] mpc_dt mpc time step duration.
         * @param [out] interpolated control signal at the requested time.
         * */
        void getControlMPCSolutionsAtTime(double const &t, double const &mpc_dt,
                                          typename Model::input_vector_t &u_solutions_mpc) const;

        // For LPV feedback solutions.
        /**
         * @brief gets the control signals computed by the LPV feedback. The controls are [vx, steering]_inputs.
         * @param [out] u_feedback_solution the feedback controls is passed into.
         * */
        void getFeedbackControls(typename Model::input_vector_t &u_feedback_solution) const;

        // SETTERS.
        /**
         * @brief sets the control signals computed by the LPV feedback.
         * @param [in] u_feedback_model_msg the feedback controls is passed into.
         * */
        void setFeedbackControls(typename Model::input_vector_t const &u_feedback_model_msg);

        // DATA MEMBERS
        double dt{};  // trajectory time step.

        // To be used in the later development stages (Zero order or
        // first order hold option: ZOH and FOH).
        controlSampling_order control_signal_order{controlSampling_order::ZOH};

        typename Model::input_vector_t u_model_solution_;  // [vx, steering]_inputs.
    };

    /**
     * We keep an extra place for the number of control vectors. Depending on the discretization methods; ZOH or FOH
     * the size of control vector container change. In FOH (First Order Hold) the number of control vectors must be
     * equal to the number of state vectors as the controls are interpolated between the integration interval.
     *
     * */
    template<class Model>
    void TrajectoryData<Model>::initializeTrajectory(size_t const &K, double const &dt_step)
    {
      // X.resize(K);
      // U.resize(K); // Alternative initialization of X and U is;

      x_type.setZero();
      u_type.setZero();

      X = typename Model::state_vector_v_t(K, x_type);
      U = typename Model::input_vector_v_t(K, u_type);

      // Initialize the control signal containers.
      u_model_solution_.setZero();

      dt = dt_step;
    }

    template<class Model>
    size_t TrajectoryData<Model>::nX() const
    {
      return X.size();
    }

    template<class Model>
    size_t TrajectoryData<Model>::nU() const
    {
      return U.size();
    }

    template<class Model>
    void TrajectoryData<Model>::getControlMPCSolutionsAtTime(const double &t,
                                                             const double &mpc_dt,
                                                             typename Model::input_vector_t &u_solutions_mpc) const
    {
      if (t > mpc_dt)
      {
        ns_utils::print("[nonlinear_mpc] The control at time t greater than the MPC time is not implemented  "
                        "...");

        ns_utils::print("[nonlinear_mpc] The requested time for the control interpolation must be less than "
                        "the MPC  time step ");
        return;
      }

      // Get the first and second controls.
      auto u0 = U.at(0);

      if (control_signal_order == controlSampling_order::ZOH)
      {
        u_solutions_mpc = u0;
      } else
      {
        auto const u1 = U.at(1);
        u_solutions_mpc = u0 + (t / mpc_dt) * (u1 - u0);
      }
    }

// u_model_solution_ = [ax, steering_rate]
    template<class Model>
    void TrajectoryData<Model>::setFeedbackControls(typename Model::input_vector_t const &u_feedback_model_msg)
    {
      u_model_solution_ = u_feedback_model_msg;
    }

    template<class Model>
    void TrajectoryData<Model>::getFeedbackControls(typename Model::input_vector_t &u_feedback_solution_msg) const
    {
      u_feedback_solution_msg = u_model_solution_;
    }

}  // namespace ns_data
#endif  // NONLINEAR_MPC_CORE__NMPC_DATA_TRAJECTORY_HPP_
