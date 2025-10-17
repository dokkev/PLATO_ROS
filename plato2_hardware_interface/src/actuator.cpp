#include "plato2_hardware_interface/actuator.hpp"
#include <iostream>

namespace actuator{

////////////////////////////////////////////////////////////////////////////
////////////////////////////// ACTUATOR CLASS //////////////////////////////
////////////////////////////////////////////////////////////////////////////

Actuator::Actuator(pcan_interface::PCANInterface &pcan_interface, const Config& config) 
    :   pcan_interface_(pcan_interface), 
        config_(config),
        encoder_(config.gear_ratio, config.torque_constant, config.can_tx_id),
        decoder_(config.gear_ratio, config.torque_constant),
        b_motor_enabled_(false) {   

    // Initialize the command message
    onoff_msg_ = init_message_(8);
    cmd_msg_ = init_message_(8);
    config_msg_ = init_message_(7);

    // Precalculate limit thresholds using config struct limits
    min_limit_threshold_ = config_.joint_limit_min + JOINT_LIMIT_SAFETY_MARGIN;
    max_limit_threshold_ = config_.joint_limit_max - JOINT_LIMIT_SAFETY_MARGIN;
}

Actuator::~Actuator(){
    stop_control();
}

////////////////////////////////////////////////////////////////////////////

void Actuator::enable_motor(){

    encoder_.set_limits(config_msg_,
                        mit_can_protocol::POS_MAX,
                        mit_can_protocol::VEL_MAX,
                        mit_can_protocol::T_MAX,
                        true, true, true);
    pcan_interface_.send_message(config_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();

    



}

////////////////////////////////////////////////////////////////////////////

void Actuator::disable_motor(){

    encoder_.stop_motor(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();
    
}

////////////////////////////////////////////////////////////////////////////

void Actuator::stop_control(){
    encoder_.stop_control(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);


}





////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_torque(const float &joint_torque, const uint32_t &duration ) {



    float motor_torque;
    joint_to_motor_(joint_torque, motor_torque);

    // Encode and send the torque command over CAN
    encoder_.set_impedance(cmd_msg_, 0.0f, 0.0f, 0.0f, 0.0f, motor_torque);
    pcan_interface_.send_message(cmd_msg_);

    // Cache command values
    commands_.torque = joint_torque;
    
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_impedance(const float &joint_position, 
                                   const float &joint_velocity, 
                                   const float &joint_stiffness, 
                                   const float &joint_damping, 
                                   const float &joint_torque) {

    float motor_position, motor_velocity, motor_torque;
    joint_to_motor_(joint_position, motor_position);
    joint_to_motor_(joint_velocity, motor_velocity);
    joint_to_motor_(joint_torque, motor_torque);

    // Encode and send the impedance command over CAN
    encoder_.set_impedance(cmd_msg_, motor_position, motor_velocity, joint_stiffness, joint_damping, motor_torque);
    pcan_interface_.send_message(cmd_msg_);

    // Cache command values
    commands_.position = joint_position;
    commands_.velocity = joint_velocity;
    commands_.stiffness = joint_stiffness;
    commands_.damping = joint_damping;
    commands_.torque = joint_torque;
}


////////////////////////////////////////////////////////////////////////////

void Actuator::process_message(const TPCANMsg &msg) {
    
    // Early return for invalid messages
    if (msg.LEN < 7) return;
    
    // Handle state messages (8-byte MIT control responses or 7-byte 0xF1 responses)
    if (msg.LEN == 8 || msg.DATA[0] == 0xF1) {
        process_state_message(msg);
    }
    else if (msg.DATA[0] == 0xF0) {
        process_limits_message(msg);
    }
}

void Actuator::process_state_message(const TPCANMsg &msg) {
    float motor_pos, motor_vel, motor_torque;
    float kp, kd;  // Unused for 0xF1 but required by decoder interface

    // Decode MIT CAN state response - decoder returns joint-space values
    decoder_.get_states(msg, motor_pos, motor_vel, kp, kd, motor_torque, 
                       states_.in_oc_mode, states_.has_fault);

    // Use motor_to_joint_ for consistent transformations (direction + offset if needed)
    motor_to_joint_(motor_pos, states_.position);    // Apply direction transformation
    motor_to_joint_(motor_vel, states_.velocity);    // Apply direction transformation  
    motor_to_joint_(motor_torque, states_.torque);   // Apply direction transformation

    // Store raw motor position for internal tracking
    motor_position_ = motor_pos;

    // Update motor enabled status and handle faults
    b_motor_enabled_ = states_.in_oc_mode && !states_.has_fault;
    
    if (states_.has_fault) {
        std::cerr << "Motor fault on CAN ID 0x" << std::hex << msg.ID << std::dec << std::endl;
    }
}

void Actuator::process_limits_message(const TPCANMsg &msg) {
    float pos_max, vel_max, tq_max;
    decoder_.get_limits(msg, pos_max, vel_max, tq_max);
    std::cout << "Motor limits CAN ID 0x" << std::hex << msg.ID << std::dec 
              << ": pos=" << pos_max << "rad, vel=" << vel_max << "rad/s, torque=" << tq_max << "Nm" << std::endl;
}



} // namespace actuator