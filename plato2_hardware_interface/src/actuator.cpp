#include "plato2_hardware_interface/actuator.hpp"
#include <iostream>
#include <cmath>

namespace actuator {

Actuator::Actuator(pcan_interface::PCANInterface &pcan_interface, const Config& config) 
    : pcan_interface_(pcan_interface), 
      config_(config),
      control_state_(SoftLimitState::kOperational),
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

void Actuator::determine_current_state_() {
    // Update state machine with hysteresis
    if (states_.position <= config_.position_limit_min || 
        states_.position >= config_.position_limit_max) {
        // Hard limit exceeded
        control_state_ = SoftLimitState::kOverLimit;
        
    } else if (states_.position < config_.position_limit_min + safety_margin_ && 
               states_.velocity < 0.0f) {
        // Approaching lower limit (moving downward)
        control_state_ = SoftLimitState::kLowerLimit;
        
    } else if (states_.position > config_.position_limit_max - safety_margin_ && 
               states_.velocity > 0.0f) {
        // Approaching upper limit (moving upward)
        control_state_ = SoftLimitState::kUpperLimit;
        
    } else if (control_state_ == SoftLimitState::kLowerLimit) {
        // Exit lower limit state with hysteresis
        if (states_.position > config_.position_limit_min + safety_margin_ + hysteresis_margin_) {
            control_state_ = SoftLimitState::kOperational;
        }
        
    } else if (control_state_ == SoftLimitState::kUpperLimit) {
        // Exit upper limit state with hysteresis
        if (states_.position < config_.position_limit_max - safety_margin_ - hysteresis_margin_) {
            control_state_ = SoftLimitState::kOperational;
        }
        
    } else {
        control_state_ = SoftLimitState::kOperational;
    }
}

////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_impedance(float joint_position_cmd, 
                                   float joint_velocity_cmd, 
                                   float joint_stiffness_cmd, 
                                   float joint_damping_cmd, 
                                   float joint_torque_cmd) {

    // Clamp commands to limits first
    clamp_commands_(joint_position_cmd, joint_velocity_cmd, joint_stiffness_cmd, joint_damping_cmd, joint_torque_cmd);
    
    // Soft limit gradient torque parameters
    const float MAX_LIMIT_TORQUE = 2.0f; // Maximum resistive torque at hard limit [Nm]
    
    // Determine current state based on position and velocity
    determine_current_state_();
    
    // Calculate gradient torque based on state
    float limit_torque = 0.0f;
    
    switch (control_state_) {
        case SoftLimitState::kOperational:
            // No additional torque needed
            limit_torque = 0.0f;
            break;
            
        case SoftLimitState::kLowerLimit: {
            // Quadratic gradient increases as we approach lower limit
            float violation = (config_.position_limit_min + safety_margin_) - states_.position;
            float gradient = std::clamp(violation / safety_margin_, 0.0f, 1.0f);
            // Apply quadratic scaling: gradient^2
            // This makes the torque increase smoothly and more aggressively near the limit
            float quad_gradient = gradient * gradient;
            // limit_torque = quad_gradient * MAX_LIMIT_TORQUE;  // Push back up (positive torque)
            break;
        }
        
        case SoftLimitState::kUpperLimit: {
            // Quadratic gradient increases as we approach upper limit
            float violation = states_.position - (config_.position_limit_max - safety_margin_);
            float gradient = std::clamp(violation / safety_margin_, 0.0f, 1.0f);
            // Apply quadratic scaling: gradient^2
            float quad_gradient = gradient * gradient;
            // limit_torque = -quad_gradient * MAX_LIMIT_TORQUE;  // Push back down (negative torque)
            break;
        }
        
        case SoftLimitState::kOverLimit:
            // Emergency: zero all commands for safety
            joint_position_cmd = states_.position;
            joint_velocity_cmd = 0.0f;
            joint_stiffness_cmd = 0.0f;
            joint_damping_cmd = 0.0f;
            joint_torque_cmd = 0.0f;
            limit_torque = 0.0f;
            break;
    }
    
    // Add gradient torque to commanded torque (except in kOverLimit)
    if (control_state_ != SoftLimitState::kOverLimit) {
        joint_torque_cmd += 0.0f; // limit_torque;
    }

    // Convert to motor frame
    float motor_position_cmd = joint_to_motor_(joint_position_cmd);
    float motor_velocity_cmd = joint_to_motor_(joint_velocity_cmd);
    float motor_torque_cmd = joint_to_motor_(joint_torque_cmd);



    // NOTE: we disable internal (embedded) kp gain but keep it external (plato2 joint impedance controller)
    //       However, kd gain is retained internally for better damping behavior
    // Send command to motor
    encoder_.set_impedance(cmd_msg_, motor_position_cmd, motor_velocity_cmd, 
                          joint_stiffness_cmd, joint_damping_cmd, motor_torque_cmd);
    pcan_interface_.send_message(cmd_msg_);

    // Cache commands
    commands_.position = joint_position_cmd;
    commands_.velocity = joint_velocity_cmd;
    commands_.stiffness = joint_stiffness_cmd;
    commands_.damping = joint_damping_cmd;
    commands_.torque = joint_torque_cmd;
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

    states_.position = motor_to_joint_(motor_pos);
    states_.velocity = motor_to_joint_(motor_vel);
    states_.torque = motor_to_joint_(motor_torque);

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

bool Actuator::check_joint_limits_(float joint_position) const {
    if (joint_position < config_.position_limit_min) {
        std::cerr << "Hard limit exceeded! Joint position " << joint_position 
                  << " < min limit " << config_.position_limit_min << " [CAN ID: 0x" 
                  << std::hex << config_.can_tx_id << std::dec << "]" << std::endl;
        return false;
    }
    
    if (joint_position > config_.position_limit_max) {
        std::cerr << "Hard limit exceeded! Joint position " << joint_position 
                  << " > max limit " << config_.position_limit_max << " [CAN ID: 0x" 
                  << std::hex << config_.can_tx_id << std::dec << "]" << std::endl;
        return false;
    }
    
    return true;
}

////////////////////////////////////////////////////////////////////////

void Actuator::clamp_commands_(float& joint_position, float& joint_velocity, 
                               float& joint_stiffness, float& joint_damping, 
                               float& joint_torque) const {
    // Clamp position to joint limits
    joint_position = std::clamp(joint_position, 
                                config_.position_limit_min, 
                                config_.position_limit_max);
    
    // Clamp velocity to limit (symmetric)
    joint_velocity = std::clamp(joint_velocity, 
                                -config_.velocity_limit, 
                                config_.velocity_limit);
    
    // Clamp torque to effort limit (symmetric)
    joint_torque = std::clamp(joint_torque, 
                              -config_.effort_limit, 
                              config_.effort_limit);
    
    // Clamp stiffness (always positive, 0 to max)
    joint_stiffness = std::clamp(joint_stiffness, 
                                 0.0f, 
                                 config_.stiffness_limit);
    
    // Clamp damping (always positive, 0 to max)
    joint_damping = std::clamp(joint_damping, 
                               0.0f, 
                               config_.damping_limit);
}

////////////////////////////////////////////////////////////////////////

} // namespace actuator