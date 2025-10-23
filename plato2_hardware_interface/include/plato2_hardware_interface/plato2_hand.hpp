#ifndef PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_

#include "plato2_hardware_interface/actuator.hpp"
#include "plato2_hardware_interface/ft_sensor.hpp"

#include <geometry_msgs/msg/wrench.hpp>

namespace plato2_hand {

class Hand {
public:
    /// @brief Constructor - initializes PCAN interface and actuators
    Hand(pcan_interface::PCANInterface& pcan_interface);

    /// @brief Destructor - disables motors safely
    ~Hand();

    /// @brief Enable all motors
    void enable();

    /// @brief Disable all motors
    void disable();

    /// @brief Set current position as zero reference for all actuators
    void set_current_position_as_zero();

    /// @brief Send impedance command to all actuators
    /// @param joint_position_command Target joint positions
    /// @param joint_velocity_command Target joint velocities
    /// @param joint_stiffness_command Stiffness gains (Kp)
    /// @param joint_damping_command Damping gains (Kd)
    /// @param joint_torque_command Feedforward torques
    void set_impedance_command(const std::vector<double> &joint_position_command,
                               const std::vector<double> &joint_velocity_command,
                               const std::vector<double> &joint_stiffness_command,
                               const std::vector<double> &joint_damping_command,
                               const std::vector<double> &joint_torque_command);

    /// @brief Update joint states from actuator feedback
    void update_joint_states(std::vector<double> &joint_position_states, 
                             std::vector<double> &joint_velocity_states, 
                             std::vector<double> &joint_effort_states);

    /// @brief Update force-torque sensor states
    void update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> &ft_sensor_states);

    /// @brief Get number of actuators
    size_t get_num_actuators() const { return num_actuators_; }

    /// @brief Get number of force-torque sensors
    size_t get_num_ft_sensors() const { return num_ft_sensors_; }

private:
    /// @brief Initialize CAN hardware (actuators and sensors)
    void init_can_hardware();

    /// @brief Print actuator and sensor initialization info
    void print_actuator_info_();

    // Hardware interfaces
    pcan_interface::PCANInterface &pcan_interface_;

    // Constants
    const size_t num_actuators_;
    const size_t num_ft_sensors_;

    // Hardware components
    std::vector<actuator::Actuator> actuators_;
    std::vector<sensor::FTSensor> ft_sensors_;

    // CAN message routing maps
    std::unordered_map<uint32_t, actuator::Actuator*> actuator_rx_id_map_;
    std::unordered_map<uint32_t, sensor::FTSensor*> ft_sensor_rx_id_map_;

    // Hardware configurations
    /// @brief Actuator configurations (Motor 1-8)
    std::vector<actuator::Config> actuator_configs_ = {
        {MotorTxID::MOTOR1, MotorRxID::MOTOR1, MotorOffset::MOTOR1, MotorDirection::MOTOR1, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::THUMB_ROLL_MAX, MotorPositionLimit::THUMB_ROLL_MIN},
        {MotorTxID::MOTOR2, MotorRxID::MOTOR2, MotorOffset::MOTOR2, MotorDirection::MOTOR2, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::THUMB_YAW_MAX, MotorPositionLimit::THUMB_YAW_MIN},
        {MotorTxID::MOTOR3, MotorRxID::MOTOR3, MotorOffset::MOTOR3, MotorDirection::MOTOR3, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::MCP_MAX, MotorPositionLimit::MCP_MIN},
        {MotorTxID::MOTOR4, MotorRxID::MOTOR4, MotorOffset::MOTOR4, MotorDirection::MOTOR4, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::PIP_MAX, MotorPositionLimit::PIP_MIN},
        {MotorTxID::MOTOR5, MotorRxID::MOTOR5, MotorOffset::MOTOR5, MotorDirection::MOTOR5, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::MCP_MAX, MotorPositionLimit::MCP_MIN},
        {MotorTxID::MOTOR6, MotorRxID::MOTOR6, MotorOffset::MOTOR6, MotorDirection::MOTOR6, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::PIP_MAX, MotorPositionLimit::PIP_MIN},
        {MotorTxID::MOTOR7, MotorRxID::MOTOR7, MotorOffset::MOTOR7, MotorDirection::MOTOR7, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::MCP_MAX, MotorPositionLimit::MCP_MIN},
        {MotorTxID::MOTOR8, MotorRxID::MOTOR8, MotorOffset::MOTOR8, MotorDirection::MOTOR8, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorPositionLimit::PIP_MAX, MotorPositionLimit::PIP_MIN}
    };

    /// @brief Force-torque sensor configurations
    std::vector<sensor::Config> ft_sensor_configs_ = {
        // Uncomment when FT sensors are used:
        // {FTSensorID::THUMB_FORCE, FTSensorID::THUMB_TORQUE},
        // {FTSensorID::INDEX_FORCE, FTSensorID::INDEX_TORQUE},
        // {FTSensorID::MIDDLE_FORCE, FTSensorID::MIDDLE_TORQUE}
    };
};

} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_