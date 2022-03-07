//
// Created by ali on 7/3/22.
//

#ifndef SIMPLE_PLANNING_SIMULATOR_SIM_MODEL_DELAY_STEER_ACC_GEARED_DIST_HPP
#define SIMPLE_PLANNING_SIMULATOR_SIM_MODEL_DELAY_STEER_ACC_GEARED_DIST_HPP


#include <deque>
#include <iostream>
#include <queue>

#include "eigen3/Eigen/LU"
#include "eigen3/Eigen/Core"

#include "simple_planning_simulator/vehicle_model/sim_model_interface.hpp"

class SimModelDelaySteerAccGeared_Dist : public SimModelInterface
{
public:
    /**
     * @brief constructor
     * @param [in] vx_lim velocity limit [m/s]
     * @param [in] steer_lim steering limit [rad]
     * @param [in] vx_rate_lim acceleration limit [m/ss]
     * @param [in] steer_rate_lim steering angular velocity limit [rad/ss]
     * @param [in] wheelbase vehicle wheelbase length [m]
     * @param [in] dt delta time information to set input buffer for delay
     * @param [in] acc_delay time delay for accel command [s]
     * @param [in] acc_time_constant time constant for 1D model of accel dynamics
     * @param [in] steer_delay time delay for steering command [s]
     * @param [in] steer_time_constant time constant for 1D model of steering dynamics
     */
    SimModelDelaySteerAccGeared_Dist(float64_t vx_lim,
                                     float64_t steer_lim,
                                     float64_t vx_rate_lim,
                                     float64_t steer_rate_lim,
                                     float64_t wheelbase,
                                     float64_t acc_delay,
                                     float64_t acc_time_constant,
                                     float64_t steer_delay,
                                     float64_t steer_time_constant,
                                     IDisturbanceCollection const &disturbance_collection);

    /**
     * @brief default destructor
     */
    ~SimModelDelaySteerAccGeared_Dist() = default;

private:
    const float64_t MIN_TIME_CONSTANT;  //!< @brief minimum time constant

    enum IDX
    {
        X = 0,
        Y,
        YAW,
        VX,
        STEER,
        ACCX,
    };
    enum IDX_U
    {
        ACCX_DES = 0,
        STEER_DES,
        DRIVE_SHIFT,
    };

    const float64_t vx_lim_;          //!< @brief velocity limit [m/s]
    const float64_t vx_rate_lim_;     //!< @brief acceleration limit [m/ss]
    const float64_t steer_lim_;       //!< @brief steering limit [rad]
    const float64_t steer_rate_lim_;  //!< @brief steering angular velocity limit [rad/s]
    const float64_t wheelbase_;       //!< @brief vehicle wheelbase length [m]

    std::deque<float64_t> acc_input_queue_;    //!< @brief buffer for accel command
    std::deque<float64_t> steer_input_queue_;  //!< @brief buffer for steering command
    const float64_t acc_delay_;                //!< @brief time delay for accel command [s]
    const float64_t acc_time_constant_;        //!< @brief time constant for accel dynamics
    const float64_t steer_delay_;              //!< @brief time delay for steering command [s]
    const float64_t steer_time_constant_;      //!< @brief time constant for steering dynamics

    /**
     * @brief set queue buffer for input command
     * @param [in] dt delta time
     */
    void initializeInputQueue(const float64_t &dt);

    /**
     * @brief get vehicle position x
     */
    float64_t getX() override;

    /**
     * @brief get vehicle position y
     */
    float64_t getY() override;

    /**
     * @brief get vehicle angle yaw
     */
    float64_t getYaw() override;

    /**
     * @brief get vehicle velocity vx
     */
    float64_t getVx() override;

    /**
     * @brief get vehicle lateral velocity
     */
    float64_t getVy() override;

    /**
     * @brief get vehicle longiudinal acceleration
     */
    float64_t getAx() override;

    /**
     * @brief get vehicle angular-velocity wz
     */
    float64_t getWz() override;

    /**
     * @brief get vehicle steering angle
     */
    float64_t getSteer() override;

    /**
     * @brief update vehicle states
     * @param [in] dt delta time [s]
     */
    void update(const float64_t &dt) override;

    /**
     * @brief calculate derivative of states with time delay steering model
     * @param [in] state current model state
     * @param [in] input input vector to model
     */
    Eigen::VectorXd calcModel(const Eigen::VectorXd &state, const Eigen::VectorXd &input) override;

    /**
     * @brief update state considering current gear
     * @param [in] state current state
     * @param [in] prev_state previous state
     * @param [in] gear current gear (defined in autoware_auto_msgs/GearCommand)
     * @param [in] dt delta time to update state
     */
    void
    updateStateWithGear(Eigen::VectorXd &state, const Eigen::VectorXd &prev_state, const uint8_t gear, const double dt);
};

#endif //SIMPLE_PLANNING_SIMULATOR_SIM_MODEL_DELAY_STEER_ACC_GEARED_DIST_HPP
