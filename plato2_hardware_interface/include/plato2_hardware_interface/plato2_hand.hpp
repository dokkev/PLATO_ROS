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
    TORQUE,   // Send Torque Command to the motors. Joint states are updated
    GRASP     // Send Grasp Command to the motors. Joint states are updated
};

class Hand{
public:

    /// @brief Constructor for the Hand. It initializes the PMOTOR Interface and the Actuators 
    Hand(pcan_interface::PCANInterface& pcan_interface);

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

    /// 
    void set_control_mode(const ControlMode &control_mode);

    /// @brief update linkage kinematics to calculate the reduction ratios
    void update_linkage_kinematics();

    /// @brief Send the command to the motors depending on the control mode
    /// @param joint_command float joint command value (position, velocity, torque)
    void set_commands(const std::vector<double> &joint_command, const uint32_t &duration);

    /// @brief Update the states of the motors
    void update_states(std::vector<double> &joint_position_states, std::vector<double> &joint_velocity_states, std::vector<double> &joint_effort_states);

    /// @brief  
    void print_motor_positions();

    void set_current_position_as_zero();



private:
    /// @brief PCAN Interface to communicate with motors
    pcan_interface::PCANInterface &pcan_interface_;

    /// @brief Actuator Vector
    std::vector<actuator::Actuator> actuators_;


    /// @brief Five bar linkage object
    FiveBarLinkage::FiveBarLinkage linkage_;

    /// @brief const actuator size for loop iteration
    const size_t num_actuators_;

    /// @brief Predefined Actuator Vector Map to corresponding RX CAN ID
    std::unordered_map<uint32_t, actuator::Actuator*> actuator_rx_id_map_;

    /// @brief Control Mode of the Hand
    ControlMode control_mode_;

    /// @brief Off Command fuction
    void set_off_command_();

    /// @brief IDLE Command fuction
    void set_idle_command_();

    /// @brief POSITION Command fuction
    void set_position_command_(const std::vector<double> &joint_position_command, const uint32_t &duration);

    /// @brief VELOCITY Command fuction
    void set_velocity_command_(const std::vector<double> &joint_velocity_command, const uint32_t &duration);

    /// @brief TORQUE Command fuction
    void set_torque_command_(const std::vector<double> &joint_torque_command, const uint32_t &duration);

    /// @brief Grasp Command fuction
    void set_grasp_command_();

    /// Control Mode to Command Function Map
    std::unordered_map<ControlMode, std::function<void(const std::vector<double>&, const uint32_t&)>> command_function_map_;

    /// @brief Lambda function for motion control for different control mode
    std::function<void(const std::vector<double> &joint_command, const uint32_t &duration)> command_mode_function_;

    /// @brief Function to initialize the function map
    void initialize_command_functions_();

    /// @brief Internal Counter
    uint32_t counter_ = 0;



    ///////////////////////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////////////////////

    /// @brief Print the Actuator Information
    void print_actuator_info_();

    /// @brief read RX CAN messages from the Bus using the PCAN Interface and sort the messages to the corresponding actuators
    void sort_can_rx_id_(const TPCANMsg &msg);

    
    ///////////////////////////////////////////////// CONFIGURATIONS //////////////////////////////////////////////

    /// @brief Predefined Actuator Configurations
    std::vector<actuator::Config> actuator_configs_ = {
        actuator::Config{MotorTxID::MOTOR1, MotorRxID::MOTOR1, MotorOffset::MOTOR1, MotorDirection::MOTOR1, XM430::TORQUE_CONSTANT, XM430::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR2, MotorRxID::MOTOR2, MotorOffset::MOTOR2, MotorDirection::MOTOR2, XM430::TORQUE_CONSTANT, XM430::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR4, MotorRxID::MOTOR4, MotorOffset::MOTOR4, MotorDirection::MOTOR4, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR3, MotorRxID::MOTOR3, MotorOffset::MOTOR3, MotorDirection::MOTOR3, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR6, MotorRxID::MOTOR6, MotorOffset::MOTOR6, MotorDirection::MOTOR6, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR5, MotorRxID::MOTOR5, MotorOffset::MOTOR5, MotorDirection::MOTOR5, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR8, MotorRxID::MOTOR8, MotorOffset::MOTOR8, MotorDirection::MOTOR8, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR7, MotorRxID::MOTOR7, MotorOffset::MOTOR7, MotorDirection::MOTOR7, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO}
    };

    ///@brief Five Bar Linkage Configuration
    FiveBarLinkage::FiveBarLinkageConfig five_bar_linkage_config_ = {
        PlatoLinkage::L1,
        PlatoLinkage::L2,
        PlatoLinkage::L3,
        PlatoLinkage::L4,
        PlatoLinkage::L5,
        Plato::deg2rad(PlatoLinkage::PIP_MOTOR_ZERO_ANGLE_OFFSET)
    };


    
    std::vector<float> linkage_reduction_ratios_ = {
        1.0f, // MOTOR1 // JOINT1: Thumb CMC Roll (const)
        1.0f, // MOTOR2 // JOINT2: Thumb CMC Yaw (const)
        1.0f, // MOTOR4 // JOINT3: Thumb MCP (const)
        1.0f, // MOTOR3 // JOINT4: Thumb IP  
        1.0f, // MOTOR6 // JOINT5: Index MCP (const)
        1.0f, // MOTOR5 // JOINT6: Index PIP 
        1.0f, // MOTOR8 // JOINT7: Middle MCP (const)  
        1.0f, // MOTOR7 // JOINT8: Middle PIP 
        
    };
};


} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PMOTOR_INTERFACE_HPP_