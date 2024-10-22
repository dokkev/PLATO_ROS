#ifndef PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_


#include "plato2_hardware_interface/hardware_config/linkage_config.hpp"

#include "plato2_hardware_interface/actuator.hpp"
#include "plato2_hardware_interface/five_bar_linkage.hpp"


namespace plato2_hand{


struct States{
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> torque;
};

struct ActuatorGains{
    std::vector<uint32_t> kp_velocity;
    std::vector<uint32_t> ki_velocity;
    std::vector<uint32_t> kp_position;
    std::vector<uint32_t> ki_position;
    std::vector<uint32_t> kd_position;
};

enum class ControlMode {
    OFF,      // Sends Stop Control Cmd to the motors. Joint states are not updated
    IDLE,     // Send Zero Torque Command to the motors. Joint states are updated
    POSITION, // Send Position Command to the motors. Joint states are updated
    VELOCITY, // Send Velocity Command to the motors. Joint states are updated
    TORQUE    // Send Torque Command to the motors. Joint states are updated
};

class Hand{
public:

    /// @brief Constructor for the Hand. It initializes the PMOTOR Interface and the Actuators 
    Hand();

    /// @brief  Destructor for the Hand. It stops the motion control
    ~Hand();

    /// @brief Initialize the actuators. Called in the constructor
    void init_actuators();

    /// @brief Enable the motors
    void enable();

    /// @brief Disable the motors and set the motors to stop position when the motor is in motion
    void disable();

    /// @brief Stop the motion control
    void stop();

    /// @brief Send the command to the motors depending on the control mode
    /// @param joint_command float joint command value (position, velocity, torque)
    void set_commands(const double &joint_command, const uint32_t &duration);

    /// @brief Update the states of the motors
    void update_states(double &joint_position_states, double &joint_velocity_states, double &joint_effort_states);

    /// @brief  
    void get_states();   

    /// @brief Modify the control mode of the hand
    /// @param control_mode 
    void set_control_mode(const ControlMode &control_mode);

private:
    /// @brief PCAN Interface to communicate with motors
    pcan_interface::PCANInterface pcan_interface_;

    /// @brief Actuator Vector
    std::vector<actuator::Actuator> actuators_;

    /// @brief Control Mode of the Hand
    ControlMode control_mode_;

    /// @brief Internal Counter
    uint32_t counter_ = 0;

    /// @brief Print the Actuator Information
    void print_actuator_info_();

    /// @brief read RX CAN messages from the Bus using the PCAN Interface and sort the messages to the corresponding actuators
    void sort_can_rx_id_(const TPCANMsg &msg);


    /// @brief Predefined Actuator Configurations
    std::vector<actuator::Config> actuator_configs_ = {
        actuator::Config{MotorTxID::MOTOR1, MotorRxID::MOTOR1, MotorOffset::MOTOR1, MotorDirection::MOTOR1, XM430::TORQUE_CONSTANT, XM430::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR2, MotorRxID::MOTOR2, MotorOffset::MOTOR2, MotorDirection::MOTOR2, XM430::TORQUE_CONSTANT, XM430::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR3, MotorRxID::MOTOR3, MotorOffset::MOTOR3, MotorDirection::MOTOR3, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR4, MotorRxID::MOTOR4, MotorOffset::MOTOR4, MotorDirection::MOTOR4, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR5, MotorRxID::MOTOR5, MotorOffset::MOTOR5, MotorDirection::MOTOR5, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR6, MotorRxID::MOTOR6, MotorOffset::MOTOR6, MotorDirection::MOTOR6, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR7, MotorRxID::MOTOR7, MotorOffset::MOTOR7, MotorDirection::MOTOR7, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR8, MotorRxID::MOTOR8, MotorOffset::MOTOR8, MotorDirection::MOTOR8, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO}
    };

    ///@brief Five Bar Linkage Configuration
    // FiveBarLinkage::FiveBarLinkageConfig five_bar_linkage_config_ = {
    //     PlatoLinkage::L1,
    //     PlatoLinkage::L2,
    //     PlatoLinkage::L3,
    //     PlatoLinkage::L4,
    //     PlatoLinkage::L5,
    //     Plato::deg2rad(PlatoLinkage::PIP_MOTOR_ZERO_ANGLE_OFFSET)

    // };


    /// @brief Predefined Actuator Vector Map to corresponding RX CAN ID
    std::unordered_map<uint32_t, actuator::Actuator*> actuator_rx_id_map_ = {
        {MotorRxID::MOTOR1, &actuators_[0]},
        {MotorRxID::MOTOR2, &actuators_[1]},
        {MotorRxID::MOTOR3, &actuators_[2]},
        {MotorRxID::MOTOR4, &actuators_[3]},
        {MotorRxID::MOTOR5, &actuators_[4]},
        {MotorRxID::MOTOR6, &actuators_[5]},
        {MotorRxID::MOTOR7, &actuators_[6]},
        {MotorRxID::MOTOR8, &actuators_[7]}
    };


};


} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PMOTOR_INTERFACE_HPP_