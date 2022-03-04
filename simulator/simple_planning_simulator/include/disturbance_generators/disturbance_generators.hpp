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

#ifndef SIMPLE_PLANNING_SIMULATOR_DISTURBANCE_GENERATORS_HPP
#define SIMPLE_PLANNING_SIMULATOR_DISTURBANCE_GENERATORS_HPP

#include <chrono>
#include "autoware_control_toolbox/autoware_control_toolbox.hpp"
#include "autoware_control_toolbox/utils/act_utils.hpp"
#include <random>
#include <memory>

/**
 * @brief given a time delay Td value, stores time-varying time delay model.
 * */
class DelayModelSISO
{
public:

    DelayModelSISO() = default;

    explicit DelayModelSISO(double const &Td, size_t const &pade_order, double const &dt);

    // Methods.
    void updateModel(double const &newTd);

    double getNextState(double const &u);

    [[nodiscard]] double getMeanTimeDelay() const
    {
        return meanTd_;
    }

    [[nodiscard]] const double &getCurrentTimeDelay() const
    {
        return Td_;
    }

    void printModelTF();


private:
    double meanTd_{0.0}; //  !<-@brief current delay that is used to compute a transfer function.
    size_t pade_order_{1}; //!<-@brief order of Pade approximation.
    double dt_{0.05}; // !<-@brief time step
    double Td_{0.0}; // !<-@brief current randomized time delay

    ns_control_toolbox::tf tf_{}; //!<-@brief current transfer function.
    ns_control_toolbox::tf2ss ss_{}; //!<-@brief state space model.

    // Model initial state:
    Eigen::MatrixXd x0_{Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(pade_order_), 1)};

};

// -------------------- INTERFACE CLASSES ------------------------------------
/**
 * @brief Input Disturbance TimeDelay Interface.
 * */
class IDisturbanceInterface_TimeDelay
{
public:
    // virtual ~disturbanceInterface() = default;
    virtual double getDisturbedInput(double const &input) = 0;

    [[nodiscard]] virtual double getCurrentTimeDelayValue() const = 0;

    /**
     * @brief returns a pair of time-delayed and nondelayed input pairs.
     * */
    [[nodiscard]] std::pair<double, double> getOriginalAndDelayedInputs() const
    {
        return std::make_pair(nondelayed_input_, delayed_input_);
    }

protected:

    // Input disturbance time-delay related.
    double nondelayed_input_{};
    double delayed_input_{};

};

/**
 * @brief Input Disturbance Deadzone Interface
 * */

class IDisturbanceInterface_DeadZone
{
public:
    using pair_type = std::pair<std::pair<double, double>, std::pair<double, double>>;

    virtual double getDisturbedInput(double const &input) = 0;

    /**
     * @brief Deadzone interface.
     * @return pair_type pair ([ml, bl] , [mr, br])  where m is the slope of the deadzone, and b is x-intercept of
     * the deadzone boundary
     * */
    [[nodiscard]] virtual pair_type getCurrentTimeDelayValue() const = 0;

};


// -------------------- IDENTITY CLASSES ------------------------------------
/**
 * @brief Identity class that maps the input as is when the time-delay value is zero in the settings.
 * */
class InputDisturbance_IdentityTimeDelay : public IDisturbanceInterface_TimeDelay
{
public:
    double getDisturbedInput(double const &input) override
    {
        // ns_utils::print("Identity Input Disturbance is called ...");
        nondelayed_input_ = input;
        delayed_input_ = input;

        return input;
    }

    [[nodiscard]] double getCurrentTimeDelayValue() const override
    {
        return 0.0;
    }

};

/**
 * @brief Identity class that maps the input as is when the time-delay value is zero in the settings.
 * */
class InputDisturbance_DeadZone : public IDisturbanceInterface_DeadZone
{
public:
    double getDisturbedInput(double const &input) override
    {
        // ns_utils::print("Identity Input Disturbance is called ...");
        return input;
    }

    [[nodiscard]] pair_type getCurrentTimeDelayValue() const override
    {
        // m = 1, b =0 for both left and right parts of a dead-zone nonlinearity graph.
        auto inactive_deadzone_params = std::make_pair(1.0, 0.0);

        return std::make_pair(inactive_deadzone_params, inactive_deadzone_params);
    }

};


// -------------------- DISTURBANCE CLASSES ------------------------------------
/**
 * @brief Input disturbance classes.
 *
 * */
class InputDisturbance_TimeDelayPade : public IDisturbanceInterface_TimeDelay
{
public:

    InputDisturbance_TimeDelayPade() = default;

    InputDisturbance_TimeDelayPade(DelayModelSISO delay_model,
                                   double const &lambda_exp,
                                   double const &td_fluctuation_amp,
                                   double const &td_angular_velocity,
                                   bool const &use_time_varying_delay);

    // Produce the disturbed - time-delayed input.
    [[nodiscard]] double getDisturbedInput(double const &input) override;

    // Get current time delay value.
    [[nodiscard]] double getCurrentTimeDelayValue() const override;


private:
    DelayModelSISO delay_model_{};

    // To keep track of time in seconds in use in sin(wt) - since_epoch.
    std::chrono::steady_clock::time_point tp_instantiation_{std::chrono::steady_clock::now()};

    // to compare the tnext(now) - tprevious
    std::chrono::steady_clock::time_point tp_previous_{std::chrono::steady_clock::now()};

    //!<@brief Time-varying delay time delay change point distribution.
    /**
     *  @brief Time delay is changed after tau seconds ahead to a new value.
     * */
    std::mt19937 generator_;
    std::exponential_distribution<> time_delay_exp_dist_{0.0};

    double next_time_interval_{};

    double lambda_exp_rate_{0.}; // <-@brief exponential distribution rate. 1/time.
    double time_delay_fluctuation_percentage_{0.};  // <-@brief amplitude of a sinusoidal fluctuation.
    double w_of_sin_{0.}; // <-@brief time fluctuates sinusoidally with an angular velocity.
    bool use_time_varying_delay_{true};

    // Methods
    void checkTimeAndUpdateModel();

};



// -------------------- OUTPUT DISTURBANCE - INTERFACE CLASSES ------------------------------------
/**
 * @brief Input disturbance class.
 * */

class IOutputDisturbance_Interface
{
public:
    // virtual ~disturbanceInterface() = default;
    virtual double getDisturbedOutput() = 0;

    /**
     * @brief Single parameter output disturbance generator. For the acceleration, this parameter may be {mu}-road
     * surface friction, or time-varying slope variable.
     * */
    [[nodiscard]] virtual double getCurrentDisturbanceParameterValue() const = 0;

protected:
    double current_slope_disturbance_{};  //!<@brief g*sin(theta)

private:
};


// -------------------- OUTPUT DISTURBANCE - IDENTITY CLASSES ------------------------------------

class OutputDisturbance_AdditiveIdentity : public IOutputDisturbance_Interface
{
public:


    double getDisturbedOutput() override
    {
        return 0.0;
    }

    [[nodiscard]] double getCurrentDisturbanceParameterValue() const override
    {
        return current_slope_disturbance_;
    }

private:
};
// -------------------- OUTPUT DISTURBANCE  CLASSES ------------------------------------

class OutputDisturbance_SlopeVariation : public IOutputDisturbance_Interface
{
public:


    OutputDisturbance_SlopeVariation() = default;

    OutputDisturbance_SlopeVariation(double const &rs_mean_slope_val,
                                     double const &rs_fluctuation_,
                                     double const &rs_angular_velocity,
                                     bool const &rs_use_time_varying_slope);

    double getDisturbedOutput() override;

    [[nodiscard]] double getCurrentDisturbanceParameterValue() const override
    {
        return current_road_slope_;
    }

private:
    double mean_road_slope_{0.0};
    double current_road_slope_{0.0}; // <-@brief current value of road slope.
    double road_slope_fluctuation_{0.};  // <-@brief amplitude of a sinusoidal fluctuation.
    double w_of_sin_{0.}; // <-@brief time fluctuates sinusoidally with an angular velocity.
    bool use_time_varying_slope_{true};

    // g m/s/s
    const double g_{9.81};

    std::chrono::steady_clock::time_point tp_instantiation_{std::chrono::steady_clock::now()};

};


// --------------------  DISTURBANCE COLLECTION for the SIMULATOR NODE ----------------------------

// Collection of disturbances for a node.
struct IDisturbanceCollection
{
    IDisturbanceCollection()
            : steering_inputDisturbance_time_delay_ptr_{std::make_shared<InputDisturbance_IdentityTimeDelay>()},
              acc_inputDisturbance_time_delay_ptr_{std::make_shared<InputDisturbance_IdentityTimeDelay>()},
              road_slope_outputDisturbance_ptr_{std::make_shared<OutputDisturbance_AdditiveIdentity>()}
    {

    };


    // Destructor
    ~IDisturbanceCollection() = default;

    // Input disturbances time-varying time-delay.
    std::shared_ptr<IDisturbanceInterface_TimeDelay> steering_inputDisturbance_time_delay_ptr_;
    std::shared_ptr<IDisturbanceInterface_TimeDelay> acc_inputDisturbance_time_delay_ptr_;

    // Output disturbances
    std::shared_ptr<IOutputDisturbance_Interface> road_slope_outputDisturbance_ptr_;
//	disturbanceInterface yawMoment_sudden_yaw_outputDisturbance;
//	disturbanceInterface roadTireSlipVariation_outputDisturbance;

};

#endif // SIMPLE_PLANNING_SIMULATOR_DISTURBANCE_GENERATORS_HPP
