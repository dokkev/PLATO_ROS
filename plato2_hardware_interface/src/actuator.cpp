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

    // encoder_.set_limits(config_msg_,
    //                     12.566f,// config_.joint_limit_max,
    //                     52.36f,
    //                     2.08f,
    //                     true, true, true);
    // pcan_interface_.send_message(config_msg_);
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
    
    // Convert the joint torque to motor torque
    float motor_torque;
    joint_to_motor_(joint_torque, motor_torque);

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