#include "plato2_hardware_interface/actuator.hpp"


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
    onoff_msg_ = init_message_();
    pos_msg_ = init_message_();
    vel_msg_ = init_message_();
    trq_msg_ = init_message_();
    gain_msg_ = init_message_();
    ind_msg_ = init_message_();
    config_msg_ = init_message_();

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
                                12.566f,// config_.joint_limit_max,
                                42.0f,
                                1.56f,
                                true, true, true);
    pcan_interface_.send_message(config_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();

    // encoder_.set_zero_position(onoff_msg_);
    // pcan_interface_.send_message(onoff_msg_);
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // pcan_interface_.receive_message();
    encoder_.start_motor(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
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

void Actuator::set_joint_position(const float &joint_position, const uint32_t &duration){


    float position_gain = 1.0f; // Nm/rad
    // Convert the joint position to motor position
    float motor_position;
    joint_to_motor_(joint_position, motor_position, true);

    // Encode and send the position command over CAN
    encoder_.set_impedance(pos_msg_, motor_position, 0.0f, position_gain, 0.0f, 0.0f);
    pcan_interface_.send_message(pos_msg_);
    
    // Cache command values
    commands_.position = joint_position;
 
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_velocity(const float &joint_velocity, const uint32_t &duration) {

    float velocity_gain = .10f; // Nm/rad/s
    // Convert the joint velocity to motor velocity
    float motor_velocity;
    joint_to_motor_(joint_velocity, motor_velocity);

    // Encode and send the velocity command over CAN
    encoder_.set_impedance(vel_msg_, 0.0f, motor_velocity, 0.0f, velocity_gain, 0.0f);
    pcan_interface_.send_message(vel_msg_);
    
    // Cache command values
    commands_.velocity = joint_velocity;

}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_torque(const float &joint_torque, const uint32_t &duration ) {

    // Apply limit protection
    const float safe_joint_torque = limit_torque_near_bounds_(joint_torque, states_.position);
    
    // Convert the joint torque to motor torque
    float motor_torque;
    joint_to_motor_(joint_torque, motor_torque);

    // Encode and send the torque command over CAN
    encoder_.set_impedance(trq_msg_, 0.0f, 0.0f, 0.0f, 0.0f, motor_torque);
    pcan_interface_.send_message(trq_msg_);
    
    // Cache command values
    commands_.torque = joint_torque;
    
}


////////////////////////////////////////////////////////////////////////////

void Actuator::calibrate_encoder(){}

void Actuator::calibrate_phase_order(){}

////////////////////////////////////////////////////////////////////////////

void Actuator::process_message(const TPCANMsg &msg){

    // Decode the received message
    float position_rad = 0.0f;
    float velocity_rps = 0.0f;
    float kp = 0.0f;
    float kd = 0.0f;
    float torque_nm = 0.0f;
    bool in_oc_mode = false;
    bool has_fault = false;

    decoder_.get_states(msg, position_rad, velocity_rps, kp, kd, torque_nm, in_oc_mode, has_fault);

    // Convert motor states to joint states
    motor_to_joint_(position_rad, states_.position, true);
    motor_to_joint_(position_rad, motor_position_, true);
    motor_to_joint_(velocity_rps, states_.velocity);
    motor_to_joint_(torque_nm, states_.torque);

    states_.in_oc_mode = in_oc_mode;
    states_.has_fault = has_fault;

    // Update motor enabled status
    b_motor_enabled_ = in_oc_mode && !has_fault;
}



} // namespace actuator