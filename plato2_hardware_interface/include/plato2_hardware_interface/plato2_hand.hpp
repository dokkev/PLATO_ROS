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


    void set_default_can_limits();

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
        actuator::ActuatorConfigFactory::create_thumb_roll_config(),  // Motor 1: Thumb CMC Roll
        actuator::ActuatorConfigFactory::create_thumb_yaw_config(),   // Motor 2: Thumb MCP Yaw
        actuator::ActuatorConfigFactory::create_mcp_config(3),        // Motor 3: Thumb MCP Pitch
        actuator::ActuatorConfigFactory::create_pip_config(4),        // Motor 4: Thumb PIP Pitch
        actuator::ActuatorConfigFactory::create_mcp_config(5),        // Motor 5: Index MCP Pitch
        actuator::ActuatorConfigFactory::create_pip_config(6),        // Motor 6: Index PIP Pitch
        actuator::ActuatorConfigFactory::create_mcp_config(7),        // Motor 7: Middle MCP Pitch
        actuator::ActuatorConfigFactory::create_pip_config(8)         // Motor 8: Middle PIP Pitch
    };

    /// @brief Force-torque sensor configurations
    std::vector<sensor::Config> ft_sensor_configs_ = {
        // Uncomment when FT sensors are used:
        {FTSensorID::THUMB_FORCE, FTSensorID::THUMB_TORQUE},
        {FTSensorID::INDEX_FORCE, FTSensorID::INDEX_TORQUE},
        {FTSensorID::MIDDLE_FORCE, FTSensorID::MIDDLE_TORQUE}
    };
};

} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_