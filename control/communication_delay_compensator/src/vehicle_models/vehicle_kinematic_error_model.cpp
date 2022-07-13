// Copyright 2022 The Autoware Foundation
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

#include "vehicle_models/vehicle_kinematic_error_model.hpp"

#include "autoware_control_toolbox.hpp"


/**
 * @brief Nonlinear vehicle model
 * */
NonlinearVehicleKinematicModel::NonlinearVehicleKinematicModel(
        double const &wheelbase, double const &tau_vel, double const &tau_steer,
        double const &deadtime_vel, double const &deadtime_steer, double const &dt)
        : wheelbase_{wheelbase},
          tau_steer_{tau_steer},
          tau_vel_{tau_vel},
          dead_time_steer_{deadtime_steer},
          dead_time_vel_{deadtime_vel},
          dt_{dt} {
    size_t const order_pade = 2;
    auto tf_steer_input = ns_control_toolbox::pade(deadtime_steer, order_pade);
    auto tf_vel_input = ns_control_toolbox::pade(deadtime_vel, order_pade);

    if (!ns_utils::isEqual(deadtime_vel, 0.)) {
        use_delay_vel = true;
        deadtime_velocity_model_ = ns_control_toolbox::tf2ss(tf_vel_input, dt);

        ns_utils::print("deadtime vel model");
        deadtime_velocity_model_.print_discrete_system();
    }

    if (!ns_utils::isEqual(deadtime_steer, 0.)) {
        use_delay_steer = true;
        deadtime_steering_model_ = ns_control_toolbox::tf2ss(tf_steer_input, dt);

        ns_utils::print("deadtime steer model");
        deadtime_steering_model_.print_discrete_system();
    }

    xv0_ = Eigen::MatrixXd::Zero(order_pade, 1);
    xs0_ = Eigen::MatrixXd::Zero(order_pade, 1);

    //	ns_utils::print("delay internal state of vel");
    //	ns_eigen_utils::printEigenMat(xv0_);
    //	int a = 1;
}

void NonlinearVehicleKinematicModel::getInitialStates(std::array<double, 4> &x0) { x0 = x0_; }

/**
 *@brief Integrates the nonlinear dynamics one-step.
 * */
std::array<double, 4> NonlinearVehicleKinematicModel::simulateNonlinearOneStep(
        const double &desired_velocity, double const &desired_steering) {
    auto Vd = desired_velocity;
    auto delta_d = desired_steering;

    if (use_delay_vel) {
        // ns_utils::print("deadtime vel model");
        // deadtime_velocity_model_.print_discrete_system();

        auto &&Ad = deadtime_velocity_model_.Ad();
        auto &&Bd = deadtime_velocity_model_.Bd();
        auto &&Cd = deadtime_velocity_model_.Cd();
        auto &&Dd = deadtime_velocity_model_.Dd();

        ns_utils::print("in sim vel delay");
        ns_eigen_utils::printEigenMat(Cd * xv0_);
        ns_eigen_utils::printEigenMat(Bd * desired_velocity);
        //		ns_eigen_utils::printEigenMat((Cd * xv0_ + Dd * desired_velocity));

        // Yield y first
        Vd = (Cd * xv0_ + Dd * desired_velocity)(0);

        // Update x0v
        xv0_ = Ad * xv0_ + Bd * desired_velocity;

        // int a = 1;
    }

    if (use_delay_steer) {
        // ns_utils::print("deadtime steer model");
        // deadtime_steering_model_.print_discrete_system();

        auto Ad = deadtime_steering_model_.Ad();
        auto Bd = deadtime_steering_model_.Bd();
        auto Cd = deadtime_steering_model_.Cd();
        auto Dd = deadtime_steering_model_.Dd();

        // Yield y first
        delta_d = (Cd * xv0_ + Dd * desired_steering)(0);

        ns_utils::print("in sim steering delay");
        ns_eigen_utils::printEigenMat(Cd);
        //	ns_eigen_utils::printEigenMat((Cd * xv0_ + Bd * desired_steering));

        // Update x0v
        xv0_ = Ad * xv0_ + Bd * desired_steering;

        // int a = 1;
    }

    // Get the previous states.
    auto &&ey0 = x0_[0];
    auto &&eyaw0 = x0_[1];
    auto &&delta0 = x0_[2];
    auto &&V0 = x0_[3];

    x0_[0] = ey0 + dt_ * V0 * sin(eyaw0);                                     // ey
    x0_[1] = eyaw0 + dt_ * (V0 / wheelbase_) * (tan(delta0) - tan(delta_d));  // eyaw
    x0_[2] = delta0 - dt_ * (1 / tau_steer_) * (delta0 - delta_d);            // delta
    x0_[3] = V0 - dt_ * (1 / tau_vel_) * (V0 - Vd);                           // v

    return x0_;
}

/**
 *@brief Integrates the linear dynamics one-step.
 * */
std::array<double, 4> NonlinearVehicleKinematicModel::simulateLinearOneStep(
        const double &desired_velocity, double const &desired_steering) {
    auto Vd = desired_velocity;
    auto delta_d = desired_steering;

    if (use_delay_vel) {
        // ns_utils::print("deadtime vel model");
        // deadtime_velocity_model_.print_discrete_system();

        auto &&Ad = deadtime_velocity_model_.Ad();
        auto &&Bd = deadtime_velocity_model_.Bd();
        auto &&Cd = deadtime_velocity_model_.Cd();
        auto &&Dd = deadtime_velocity_model_.Dd();

        ns_utils::print("in sim vel delay");
        ns_eigen_utils::printEigenMat(Cd * xv0_);
        ns_eigen_utils::printEigenMat(Bd * desired_velocity);
        //		ns_eigen_utils::printEigenMat((Cd * xv0_ + Dd * desired_velocity));

        // Yield y first
        Vd = (Cd * xv0_ + Dd * desired_velocity)(0);

        // Update x0v
        xv0_ = Ad * xv0_ + Bd * desired_velocity;

        // int a = 1;
    }

    if (use_delay_steer) {
        // ns_utils::print("deadtime steer model");
        // deadtime_steering_model_.print_discrete_system();

        auto Ad = deadtime_steering_model_.Ad();
        auto Bd = deadtime_steering_model_.Bd();
        auto Cd = deadtime_steering_model_.Cd();
        auto Dd = deadtime_steering_model_.Dd();

        // Yield y first
        delta_d = (Cd * xv0_ + Dd * desired_steering)(0);

        ns_utils::print("in sim steering delay");
        ns_eigen_utils::printEigenMat(Cd);
        //	ns_eigen_utils::printEigenMat((Cd * xv0_ + Bd * desired_steering));

        // Update x0v
        xv0_ = Ad * xv0_ + Bd * desired_steering;

        // int a = 1;
    }

    // Get the previous states.
    auto &&ey0 = x0_[0];
    auto &&eyaw0 = x0_[1];
    auto &&delta0 = x0_[2];
    auto &&V0 = x0_[3];

    x0_[0] = ey0 + dt_ * V0 * eyaw0;                                                    // ey
    x0_[1] = eyaw0 + V0 * delta0 / (wheelbase_ * std::cos(delta0) * std::cos(delta0));  // eyaw
    x0_[2] = delta0 - dt_ * (1 / tau_steer_) * (delta0 - delta_d);                      // delta
    x0_[3] = V0 - dt_ * (1 / tau_vel_) * (V0 - Vd);                                     // v

    return x0_;
}
