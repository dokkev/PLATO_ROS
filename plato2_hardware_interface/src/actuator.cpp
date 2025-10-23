#include "plato2_hardware_interface/actuator.hpp"
#include <iostream>

namespace actuator {

Actuator::Actuator(pcan_interface::PCANInterface &pcan_interface, const Config& config) 
    : pcan_interface_(pcan_interface), 
      config_(config),
      encoder_(config.gear_ratio, config.torque_constant, config.can_tx_id),
      decoder_(config.gear_ratio, config.torque_constant),
      motor_position_(0.0f),
      b_motor_enabled_(false) {

    // Initialize CAN messages
    onoff_msg_ = init_message_(8);
    cmd_msg_ = init_message_(8);
    config_msg_ = init_message_(7);
    calibrate_msg_ = init_message_(1);
}

Actuator::~Actuator() {
    // Stop control on destruction
    encoder_.stop_control(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
}

////////////////////////////////////////////////////////////////////////

void Actuator::enable_motor() {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();
}

////////////////////////////////////////////////////////////////////////

void Actuator::disable_motor() {
    encoder_.stop_motor(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();
}

////////////////////////////////////////////////////////////////////////

void Actuator::set_current_position_as_zero() {
    encoder_.set_zero_position(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
}

////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_torque(float joint_torque, uint32_t duration) {
    float motor_torque;
    joint_to_motor_(joint_torque, motor_torque);

    encoder_.set_impedance(cmd_msg_, 0.0f, 0.0f, 0.0f, 0.0f, motor_torque);
    pcan_interface_.send_message(cmd_msg_);

    commands_.torque = joint_torque;
}

////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_impedance(float joint_position, 
                                   float joint_velocity, 
                                   float joint_stiffness, 
                                   float joint_damping, 
                                   float joint_torque) {
    float motor_position, motor_velocity, motor_torque;
    joint_to_motor_(joint_position, motor_position);
    joint_to_motor_(joint_velocity, motor_velocity);
    joint_to_motor_(joint_torque, motor_torque);

    encoder_.set_impedance(cmd_msg_, motor_position, motor_velocity, 
                          joint_stiffness, joint_damping, motor_torque);
    pcan_interface_.send_message(cmd_msg_);

    // Cache commands
    commands_.position = joint_position;
    commands_.velocity = joint_velocity;
    commands_.stiffness = joint_stiffness;
    commands_.damping = joint_damping;
    commands_.torque = joint_torque;
}

////////////////////////////////////////////////////////////////////////

void Actuator::process_message(const TPCANMsg &msg) {
    if (msg.LEN < 7) return;
    
    if (msg.LEN == 8 || msg.DATA[0] == 0xF1) {
        process_state_message(msg);
    } else if (msg.DATA[0] == 0xF0) {
        process_limits_message(msg);
    }
}

////////////////////////////////////////////////////////////////////////

void Actuator::process_state_message(const TPCANMsg &msg) {
    float motor_pos, motor_vel, motor_torque;
    float kp, kd;  // Required by decoder interface but unused

    decoder_.get_states(msg, motor_pos, motor_vel, kp, kd, motor_torque, 
                       states_.in_oc_mode, states_.has_fault);

    motor_to_joint_(motor_pos, states_.position);
    motor_to_joint_(motor_vel, states_.velocity);
    motor_to_joint_(motor_torque, states_.torque);

    motor_position_ = motor_pos;
    b_motor_enabled_ = states_.in_oc_mode && !states_.has_fault;
    
    if (states_.has_fault) {
        std::cerr << "Motor fault on CAN ID 0x" << std::hex << msg.ID << std::dec << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////

void Actuator::process_limits_message(const TPCANMsg &msg) {
    float pos_max, vel_max, tq_max;
    decoder_.get_limits(msg, pos_max, vel_max, tq_max);
    std::cout << "Motor limits CAN ID 0x" << std::hex << msg.ID << std::dec 
              << ": pos=" << pos_max << " rad, vel=" << vel_max 
              << " rad/s, torque=" << tq_max << " Nm" << std::endl;
}

////////////////////////////////////////////////////////////////////////

} // namespace actuator