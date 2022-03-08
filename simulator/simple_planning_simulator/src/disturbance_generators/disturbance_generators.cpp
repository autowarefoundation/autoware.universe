// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "disturbance_generators/disturbance_generators.hpp"

#include <cmath>
#include <random>
#include <utility>

InputDisturbance_TimeDelayPade::InputDisturbance_TimeDelayPade(DelayModelSISO delay_model,
                                                               double const &lambda_exp,
                                                               double const &td_fluctuation_amp,
                                                               double const &td_angular_velocity,
                                                               bool const &use_time_varying_delay)
        : delay_model_(std::move(delay_model)),
          time_delay_exp_dist_{std::exponential_distribution<>(lambda_exp)},
          lambda_exp_rate_{lambda_exp},
          time_delay_fluctuation_percentage_{td_fluctuation_amp}, /* percentage */
          w_of_sin_{td_angular_velocity},
          use_time_varying_delay_{use_time_varying_delay}
{

    std::random_device dev;
    generator_ = std::mt19937(dev());

    // Generate a time distance ahead to schedule the next Td change.
    next_time_interval_ = time_delay_exp_dist_(generator_);

//    auto now = std::chrono::system_clock::now();
//    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
//    auto epoch = now_ms.time_since_epoch();
//    seconds s = duration_cast<seconds>(p.time_since_epoch()); // エポックからの経過時間(秒)を取得
//    std::cout << s.count() << std::endl;

    // auto x = tp_instantiation_ + std::chrono::duration<double>(1.34343);
}

void InputDisturbance_TimeDelayPade::checkTimeAndUpdateModel()
{
    // Change the current time delay model, if the change event time arrived.
    auto tp_now{std::chrono::steady_clock::now()};
    auto &&diff = tp_now - tp_previous_;
    std::chrono::duration<double> time_since_previous_time_point{diff};

    // Change random seed
    std::random_device dev;
    generator_ = std::mt19937(dev());

    if (time_since_previous_time_point.count() > next_time_interval_)
    {
        // update the time-delay models.
        auto time_from_beginning = std::chrono::duration<double>(tp_now - tp_instantiation_);
        // ns_utils::print("Time since beginning : ", time_from_beginning.count());

        /**
         * Any method or randomization can be used to generate a new Td value.
         * */

        auto meanTd = delay_model_.getMeanTimeDelay();
        auto &&sinus_magnitude = time_delay_fluctuation_percentage_ * meanTd;

        double Td_new =
                delay_model_.getMeanTimeDelay() + sinus_magnitude * sin(w_of_sin_ * time_from_beginning.count());

        // Update
        delay_model_.updateModel(Td_new);

        // Mark the time_previous_
        next_time_interval_ = time_delay_exp_dist_(generator_);
        tp_previous_ = std::chrono::steady_clock::now();
    }

}

double InputDisturbance_TimeDelayPade::getDisturbedInput(const double &input)
{

    if (use_time_varying_delay_)
    {
        checkTimeAndUpdateModel(); // Check if it is time to change the time delay model.
    }


    delayed_input_ = delay_model_.getNextState(input);
    nondelayed_input_ = input;

    // Print delaySISO parameters
    // delay_model_.printModelTF();


    return delayed_input_;
}

// Get current time delay value.
[[nodiscard]] double InputDisturbance_TimeDelayPade::getCurrentTimeDelayValue() const
{
    return delay_model_.getCurrentTimeDelay();
}

DelayModelSISO::DelayModelSISO(const double &Td,
                               size_t const &pade_order,
                               const double &dt)
        : meanTd_{Td}, pade_order_{pade_order}, dt_{dt}, Td_{Td},
          x0_(Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(pade_order), 1))
{
    tf_ = ns_control_toolbox::pade(meanTd_, pade_order);
    ss_ = ns_control_toolbox::tf2ss(tf_);

}

/**
 * @brief given a time delay value, updates the internal transfer function and state space models of the time-delay.
 * @param newTd time-delay value.
 * */
void DelayModelSISO::updateModel(const double &newTd)
{
    Td_ = newTd;

    tf_ = ns_control_toolbox::pade(Td_, pade_order_);
    ss_ = ns_control_toolbox::tf2ss(tf_);

    // ns_utils::print("Time delay changed to ", Td_);
}

double DelayModelSISO::getNextState(const double &u)
{
    x0_.noalias() = x0_ + dt_ * (ss_.A_ * x0_ + ss_.B_ * u);

    auto &&u_delayed = (ss_.C_ * x0_ + ss_.D_ * u)(0);

    return u_delayed;
}

void DelayModelSISO::printModelTF()
{

    // Print current time delay and time duration at time delay changes.
    ns_utils::print("\nSteering mean time delay  vs current Td: ", meanTd_, Td_);

    // Print the transfer function.
    // ns_utils::print("\nCurrent Continuous Transfer Function ...");
    tf_.print();
//    ns_utils::print("Pade order : ", pade_order_);

    // Print the Continuous-time state space.
    ns_utils::print("\nCurrent Continuous Time State Space Models ...");
    ss_.print();

}

OutputDisturbance_SlopeVariation::OutputDisturbance_SlopeVariation(const double &rs_mean_slope_val,
                                                                   const double &rs_fluctuation_,
                                                                   const double &rs_angular_velocity,
                                                                   const bool &rs_use_time_varying_slope)
        : mean_road_slope_{rs_mean_slope_val},
          current_road_slope_{rs_mean_slope_val},
          road_slope_fluctuation_{rs_fluctuation_},
          w_of_sin_{rs_angular_velocity},
          use_time_varying_slope_{rs_use_time_varying_slope}
{

}

/**
 * @brief Computes g*sin(theta) for a varying slope. This is an output disturbance but we can compute by computing
 * the inputs that gives the disturbance added to the output.
 * */
double OutputDisturbance_SlopeVariation::getDisturbedOutput()
{

    // Compute time-varying slope.
    if (use_time_varying_slope_)
    {
        auto tp_now{std::chrono::steady_clock::now()};

        // update the slope current slope value.
        auto time_from_beginning = std::chrono::duration<double>(tp_now - tp_instantiation_);

        /**
         * Any method or randomization can be used to generate a new Td value.
         * */
        current_road_slope_ =
                mean_road_slope_ + road_slope_fluctuation_ * sin(w_of_sin_ * time_from_beginning.count());

    }

    current_slope_disturbance_ = g_ * sin(current_road_slope_);
    return current_slope_disturbance_;
}

InputDisturbance_DeadZone::InputDisturbance_DeadZone(const double &m_lr_variance,
                                                     const double &b_lr_mean,
                                                     const double &b_lr_variance_in_percent, /*in percentage*/
                                                     const double &sin_mag,
                                                     const bool &use_time_varying_deadzone) : b_mean_th_{b_lr_mean},
                                                                                              a_sin_mag_{sin_mag},
                                                                                              use_time_varying_deadzone_{
                                                                                                      use_time_varying_deadzone}
{

    if (m_lr_variance < 0. || m_lr_variance > 0.5)
    {
        throw std::invalid_argument("Slope variance should not exceed 0.5 and must be positive");
    }

    /**
     * @brief b_lr represents positive(right) and negative(left) thresholds. The left threshold must be multiplied by -1.
     * */
//    if (b_lr_variance < 0. || b_lr_variance > b_mean_th_ / 2.0)
//    {
//        throw std::invalid_argument(
//                "Deadzone threshold variance should be lower than half threshold and  must be positive");
//    }

    // Initialize the samplers.
    // Slope min-max interval = mean{1.} +- var/2.

    std::random_device dev;
    generator_ = std::mt19937(dev());

    // Initialize the slope sampler.
    auto slope_low = 1.0 - m_lr_variance / 2.;
    auto slope_high = 1.0 + m_lr_variance / 2.;
    sampler_m_ = std::uniform_real_distribution<double>(slope_low, slope_high);

    // Initialize the threshold sampler.
    auto b_var = b_lr_mean * b_lr_variance_in_percent / 100.;
    auto b_low = b_lr_mean - b_var / 2.;
    auto b_high = b_lr_mean + b_var / 2.;
    sampler_b_ = std::uniform_real_distribution<double>(b_low, b_high);

}


double InputDisturbance_DeadZone::getDisturbedInput(const double &delta_u)
{

    double delta_v{}; // deadzone output

    // Sample sinus phase.
    phi_phase_ = sampler_phi_(generator_);

    // if time-varying deadzone, sample
    if (use_time_varying_deadzone_)
    {
        // Change random seed
        std::random_device dev;
        generator_ = std::mt19937(dev());


        if (delta_u > current_br_threshold_)
        {
            // sample the slope. must be positive for all occasions.
            current_mr_slope_ = sampler_m_(generator_);
            current_br_threshold_ = sampler_b_(generator_);


            delta_v = current_mr_slope_ *
                      (delta_u + a_sin_mag_ * sin(M_2_PI * delta_u + phi_phase_) - current_br_threshold_);

        } else if (delta_u < current_bl_threshold_)
        {
            // sample the slope. must be positive for all occasions.
            current_ml_slope_ = sampler_m_(generator_);
            current_bl_threshold_ = -1. * sampler_b_(generator_);

            delta_v = current_ml_slope_ *
                      (delta_u + a_sin_mag_ * sin(M_2_PI * delta_u + phi_phase_) - current_bl_threshold_);

        } else
        {
            delta_v = 0.;
        }


    } else // omit sinusoidal part.
    {

        if (delta_u > current_br_threshold_)
        {
            delta_v = current_mr_slope_ * (delta_u - current_br_threshold_);

        } else if (delta_u < current_bl_threshold_)
        {
            delta_v = current_ml_slope_ * (delta_u - current_bl_threshold_);

        } else
        {
            delta_v = 0.;
        }

    }

    current_deadzoned_output_ = delta_v;

    return current_deadzoned_output_;
}


