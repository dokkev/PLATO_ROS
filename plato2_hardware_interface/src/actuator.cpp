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

    // Apply limit protection
    const float safe_joint_torque = limit_torque_near_bounds_(joint_torque, states_.position);
    
    // Convert joint torque to motor torque 
    // Since decoder applies gear_ratio², we need gear_ratio² compensation for commands
    float gear_ratio_sq = config_.gear_ratio * config_.gear_ratio;
    float motor_torque = (joint_torque * config_.direction) / gear_ratio_sq;

    // Encode and send the torque command over CAN
    encoder_.set_impedance(cmd_msg_, 0.0f, 0.0f, 0.0f, 0.0f, motor_torque);
    pcan_interface_.send_message(cmd_msg_);

    // Cache command values
    commands_.torque = joint_torque;
    
}


////////////////////////////////////////////////////////////////////////////

void Actuator::calibrate_encoder(){}

void Actuator::calibrate_phase_order(){}

////////////////////////////////////////////////////////////////////////////

void Actuator::process_message(const TPCANMsg &msg){
    
    // Handle state messages (8-byte MIT control responses or 7-byte 0xF1 responses)
    if (msg.LEN == 8 || (msg.LEN >= 7 && msg.DATA[0] == 0xF1)) {
        
        float motor_pos, motor_vel, motor_torque;
        float kp, kd;  // Not used by 0xF1 but required by decoder interface

        // Decode MIT CAN state response directly into states_ where possible
        decoder_.get_states(msg, motor_pos, motor_vel, kp, kd, motor_torque, 
                          states_.in_oc_mode, states_.has_fault);

        // Values from decoder already have gear_ratio² applied, use directly
        motor_position_ = motor_pos;  // Raw motor position for internal tracking
        states_.position = motor_pos;   // Joint position (with gear_ratio² applied)
        states_.velocity = motor_vel;   // Joint velocity (with gear_ratio² applied)
        states_.torque = motor_torque;  // Joint torque (with gear_ratio² applied)

        // Update motor enabled status
        b_motor_enabled_ = states_.in_oc_mode && !states_.has_fault;

        // Log faults
        if (states_.has_fault) {
            std::cerr << "Fault on CAN ID " << std::hex << msg.ID << std::dec << std::endl;
        }
    }
    else if (msg.LEN >= 7 && msg.DATA[0] == 0xF0) {
        // Handle limits response
        float pos_max, vel_max, tq_max;
        decoder_.get_limits(msg, pos_max, vel_max, tq_max);
        std::cout << "Limits ID " << std::hex << msg.ID << std::dec 
                  << ": " << pos_max << "rad, " << vel_max << "rad/s, " << tq_max << "Nm" << std::endl;
    }
}



} // namespace actuator