#ifndef PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_


#include "plato2_hardware_interface/actuator.hpp"
#include "plato2_hardware_interface/ft_sensor.hpp"
#include "plato2_hardware_interface/five_bar_linkage.hpp"
#include "plato2_hardware_interface/karnopp_compensator.hpp"

#include <geometry_msgs/msg/wrench.hpp>


namespace plato2_hand{


struct States{
    std::vector<float> position;
    std::vector<float> velocity;
    std::vector<float> torque;

    std::vector<float> force_raw;
    std::vector<float> torque_raw;
    std::vector<float> force_filtered;
    std::vector<float> torque_filtered;
};



enum class ControlMode {
    OFF,      // Sends Stop Control Cmd to the motors. Joint states are not updated
    IDLE,     // Send Zero Torque Command to the motors. Joint states are updated
    POSITION, // Send Position Command to the motors. Joint states are updated
    VELOCITY, // Send Velocity Command to the motors. Joint states are updated
    TORQUE,   // Send Torque Command to the motors. Joint states are updated
};

class Hand{
public:

    /// @brief Constructor for the Hand. It initializes the PMOTOR Interface and the Actuators 
    Hand(pcan_interface::PCANInterface& pcan_interface);

    /// @brief  Destructor for the Hand. It stops the motion control
    ~Hand();

    /// @brief Initialize the actuator and sensors communicating via CAN bus. Called in the constructor
    void init_can_hardware();

    /// @brief Enable the motors
    void enable();

    /// @brief Disable the motors and set the motors to stop position when the motor is in motion
    void disable();

    /// @brief Stop the motion control
    void stop();

    /// @brief Off Command fuction
    void set_off_command_();

    /// @brief IDLE Command fuction
    void set_idle_command();

    /// @brief POSITION Command fuction
    void set_position_command(const std::vector<double> &joint_position_command, const uint32_t &duration);

    /// @brief VELOCITY Command fuction
    void set_velocity_command(const std::vector<double> &joint_velocity_command, const uint32_t &duration);

    /// @brief TORQUE Command fuction
    void set_torque_command(const std::vector<double> &joint_torque_command, const uint32_t &duration);

    /// @brief  Send TORQUE command using PD controller
    /// @param joint_impedance position mann
    void set_impedance_command(const std::vector<double> &joint_impedance_command, const uint32_t& servo_current, const std::vector<double> &joint_position_states, const std::vector<double> &joint_velocity_states);

    void set_zero_motor_position();


    /// @brief update linkage kinematics to calculate the reduction ratios
    void update_linkage_kinematics();


    /// @brief Update the joint states of from the actuators
    void update_joint_states(std::vector<double> &joint_position_states, std::vector<double> &joint_velocity_states, std::vector<double> &joint_effort_states);

    /// @brief Update the force-torque sensor states
    void update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> &ft_sensor_states);

    /// @brief  
    void print_motor_positions();

    void get_actuators_temperature();

    void set_current_position_as_zero();

    /// @brief Return function for num of actuators and sensors
    size_t get_num_actuators() const { return num_actuators_; }
    size_t get_num_ft_sensors() const { return num_ft_sensors_; }


private:
    /// @brief PCAN Interface to communicate with motors
    pcan_interface::PCANInterface &pcan_interface_;

    /// @brief const actuator size for loop iteration
    const size_t num_actuators_;

    const size_t num_ft_sensors_;


    /// @brief Actuator Vector
    std::vector<actuator::Actuator> actuators_;
    std::vector<sensor::FTSensor> ft_sensors_;

    /// @brief Linkage Configuration
    FiveBarLinkage::FiveBarLinkageConfig five_bar_linkage_config_; 

    /// @brief Five bar linkage objects
    FiveBarLinkage::FiveBarLinkage linkage1_;
    FiveBarLinkage::FiveBarLinkage linkage2_;
    FiveBarLinkage::FiveBarLinkage linkage3_;

    // MOTOR1 // JOINT1: Thumb CMC Roll (const)
    // MOTOR2 // JOINT2: Thumb CMC Yaw (const)
    // MOTOR4 // JOINT3: Thumb MCP (const)
    // MOTOR3 // JOINT4: Thumb IP  
    // MOTOR6 // JOINT5: Index MCP (const)
    // MOTOR5 // JOINT6: Index PIP 
    // MOTOR8 // JOINT7: Middle MCP (const)  
    // MOTOR7 // JOINT8: Middle PIP
    /// @brief Reduction Ratios
    std::vector<float> pos_ratios_;
    std::vector<float> vel_ratios_;
    std::vector<float> trq_ratios_;

    std::vector<double> actuators_temperature_;


    /// @brief Predefined Actuator Vector Map to corresponding RX CAN ID
    std::unordered_map<uint32_t, actuator::Actuator*> actuator_rx_id_map_;
    std::unordered_map<uint32_t, sensor::FTSensor*> ft_sensor_rx_id_map_;

    /// @brief Internal Counter
    uint32_t counter_ = 0;


    /// @brief Karnopp Friction Compensator
    std::vector<KarnoppCompensator> friction_compensators_;


    ///////////////////////////////////////////////// PRIVATE FUNCTIONS //////////////////////////////////////////////

    /// @brief Print the Actuator Information
    void print_actuator_info_();

    /// @brief read RX CAN messages from the Bus using the PCAN Interface and sort the messages to the corresponding actuators
    void sort_can_rx_id_(const TPCANMsg &msg);

    

    
    ///////////////////////////////////////////////// CONFIGURATIONS //////////////////////////////////////////////

    /// @brief Predefined Actuator Configurations
    std::vector<actuator::Config> actuator_configs_ = {
        actuator::Config{MotorTxID::MOTOR1, MotorRxID::MOTOR1, MotorOffset::MOTOR1,  MotorDirection::MOTOR1, XM430::TORQUE_CONSTANT,   XM430::GEAR_RATIO,   MotorJointLimit::XM430_MAX, MotorJointLimit::XM430_MIN},
        actuator::Config{MotorTxID::MOTOR2, MotorRxID::MOTOR2, MotorOffset::MOTOR2,  MotorDirection::MOTOR2, XM430::TORQUE_CONSTANT,   XM430::GEAR_RATIO,   MotorJointLimit::XM430_MAX, MotorJointLimit::XM430_MIN},
        actuator::Config{MotorTxID::MOTOR4, MotorRxID::MOTOR4, MotorOffset::MOTOR4,  MotorDirection::MOTOR4, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorJointLimit::MCP_MAX, MotorJointLimit::MCP_MIN},
        actuator::Config{MotorTxID::MOTOR3, MotorRxID::MOTOR3, MotorOffset::MOTOR3,  MotorDirection::MOTOR3, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorJointLimit::PIP_MAX, MotorJointLimit::PIP_MIN},
        actuator::Config{MotorTxID::MOTOR6, MotorRxID::MOTOR6, MotorOffset::MOTOR6,  MotorDirection::MOTOR6, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorJointLimit::MCP_MAX, MotorJointLimit::MCP_MIN},
        actuator::Config{MotorTxID::MOTOR5, MotorRxID::MOTOR5, MotorOffset::MOTOR5,  MotorDirection::MOTOR5, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorJointLimit::PIP_MAX, MotorJointLimit::PIP_MIN},
        actuator::Config{MotorTxID::MOTOR8, MotorRxID::MOTOR8, MotorOffset::MOTOR8,  MotorDirection::MOTOR8, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorJointLimit::MCP_MAX, MotorJointLimit::MCP_MIN},
        actuator::Config{MotorTxID::MOTOR7, MotorRxID::MOTOR7, MotorOffset::MOTOR7,  MotorDirection::MOTOR7, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO, MotorJointLimit::PIP_MAX, MotorJointLimit::PIP_MIN}
    };

    std::vector<sensor::Config> ft_sensor_configs_ = {
        sensor::Config{FTSensorID::THUMB_FORCE, FTSensorID::THUMB_TORQUE},
        sensor::Config{FTSensorID::INDEX_FORCE, FTSensorID::INDEX_TORQUE},
        sensor::Config{FTSensorID::MIDDLE_FORCE, FTSensorID::MIDDLE_TORQUE}
    };

};


} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PMOTOR_INTERFACE_HPP_