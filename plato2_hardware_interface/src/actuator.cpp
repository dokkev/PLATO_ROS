#include "plato2_hardware_interface/actuator.hpp"


namespace actuator{

////////////////////////////////////////////////////////////////////////////
////////////////////////////// ACTUATOR CLASS //////////////////////////////
////////////////////////////////////////////////////////////////////////////

Actuator::Actuator(pcan_interface::PCANInterface &pcan_interface, Config& config) 
    :   pcan_interface_(pcan_interface), 
        config_(config),
        encoder_(config.gear_ratio, config.torque_constant),
        decoder_(config.gear_ratio, config.torque_constant){

    // Initialize the command message
    onoff_msg_ = init_message_();
    pos_msg_ = init_message_();
    vel_msg_ = init_message_();
    trq_msg_ = init_message_();
    gain_msg_ = init_message_();
    ind_msg_ = init_message_();
    config_msg_ = init_message_();
}

Actuator::~Actuator(){
    stop_control();
}

////////////////////////////////////////////////////////////////////////////

void Actuator::enable_motor(){
    encoder_.start_motor(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
}

////////////////////////////////////////////////////////////////////////////

void Actuator::disable_motor(){
    encoder_.stop_motor(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);
}

////////////////////////////////////////////////////////////////////////////

void Actuator::stop_control(){
    encoder_.stop_control(onoff_msg_);
    pcan_interface_.send_message(onoff_msg_);


}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_position(const float &joint_position, const uint32_t &duration){


    // Convert the joint position to motor position
    float motor_position;
    joint_to_motor_(joint_position, motor_position, true);

    // Encode and send the position command over CAN
    encoder_.set_position(pos_msg_, motor_position, duration);
    pcan_interface_.send_message(pos_msg_);
    
    // Cache command values
    commands_.position = joint_position;
 
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_velocity(const float &joint_velocity, const uint32_t &duration) {

    // Convert the joint velocity to motor velocity
    float motor_velocity;
    joint_to_motor_(joint_velocity, motor_velocity);

    // Encode and send the velocity command over CAN
    encoder_.set_velocity(vel_msg_, motor_velocity, duration);
    pcan_interface_.send_message(vel_msg_);
    
    // Cache command values
    commands_.velocity = joint_velocity;

}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_torque(const float &joint_torque, const uint32_t &duration ) {

    // Convert the joint torque to motor torque
    float motor_torque;
    joint_to_motor_(joint_torque, motor_torque);

    // Encode and send the torque command over CAN
    encoder_.set_torque(trq_msg_, motor_torque, duration);
    pcan_interface_.send_message(trq_msg_);
    
    // Cache command values
    commands_.torque = joint_torque;
    
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_gains(const Gains &new_gains){
    // Set the gains to the motor driver
        // Compare each gain component and send the gain if it has changed
    if (new_gains.kp_velocity != gains_.kp_velocity) {
        encoder_.set_gain(gain_msg_, new_gains.kp_velocity, ParamID::KP_SPEED);
        pcan_interface_.send_message(trq_msg_);
        gains_.kp_velocity = new_gains.kp_velocity;  // Update cached gain
    }

    if (new_gains.ki_velocity != gains_.ki_velocity) {
        encoder_.set_gain(gain_msg_, new_gains.ki_velocity, ParamID::KI_SPEED);
        pcan_interface_.send_message(trq_msg_);
        gains_.ki_velocity = new_gains.ki_velocity;  // Update cached gain
    }

    if (new_gains.kp_position != gains_.kp_position) {
        encoder_.set_gain(gain_msg_, new_gains.kp_position, ParamID::KP_POSITION);
        pcan_interface_.send_message(trq_msg_);
        gains_.kp_position = new_gains.kp_position;  // Update cached gain
    }

    if (new_gains.ki_position != gains_.ki_position) {
        encoder_.set_gain(gain_msg_, new_gains.ki_position, ParamID::KI_POSITION);
        pcan_interface_.send_message(trq_msg_);
        gains_.ki_position = new_gains.ki_position;  // Update cached gain
    }

    if (new_gains.kd_position != gains_.kd_position) {
        encoder_.set_gain(gain_msg_, new_gains.kd_position, ParamID::KD_POSITION);
        pcan_interface_.send_message(gain_msg_);
        gains_.kd_position = new_gains.kd_position;  // Update cached gain
    }
}

////////////////////////////////////////////////////////////////////////////

void Actuator::retrieve_position(){
    encoder_.retrieve_position(ind_msg_);
    pcan_interface_.send_message(ind_msg_);
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_zero_position(const float &zero_position){
    encoder_.set_zero_position(config_msg_, zero_position);
    pcan_interface_.send_message(config_msg_);
}

////////////////////////////////////////////////////////////////////////////

void Actuator::process_message(const TPCANMsg &msg){
        
    // Check the Command Byte of the Received Message and call the corresponding function
    // In switch statement, check the Message with higher priority first (msg such as motion control msgs which are updated every loop)
    // uint8_t command_byte = msg.DATA[0];
    switch (msg.DATA[0]){

        // Response Message from the Motion Control
        case CommandByte::POSITION_CONTROL:
        case CommandByte::SPEED_CONTROL:
        case CommandByte::TORQUE_CONTROL:
            // get the states if there is a valid response without any error
            if (decoder_.get_result(msg.DATA[1])) {
                // decode the states
                float motor_position, motor_velocity, motor_torque;
                decoder_.get_states(msg, status_.temperature, motor_position, motor_velocity, motor_torque);
                // store motor_position without offset
                motor_position_ = motor_position;
                // apply the motor to joint conversion and store the states
                motor_to_joint_(motor_position, states_.position, true);
                motor_to_joint_(motor_velocity, states_.velocity);
                motor_to_joint_(motor_torque, states_.torque);
            }
            
            break;

        // Gain message reponse upon request to get the gains from the motor
        case CommandByte::RETRIVE_PARAMETER:
            // If the parameter exists in the map, retrieve the pointer to the corresponding gain variable, dereference it, and pass it to the get_gain function.
            // uint8_t gain_byte = msg.DATA[1];
            // check if the parameter exists in the map

            try {
                decoder_.get_gain(msg, *gain_map_.at(msg.DATA[1]));
            } catch (const std::out_of_range& e) {
                std::cerr << "Gain Parameter Byte not found in gain_map_: " << +msg.DATA[1] << std::endl;
            }

            break;

        case CommandByte::RETRIVE_INDICATOR:
            // get the position if there is a valid response without any error
            float motor_position;
            decoder_.retrieve_position(msg, motor_position);

            // apply the motor to joint conversion and store the states
            motor_to_joint_(motor_position, states_.position, true);

            break;

        /////////////////////////// RESPONSE MESSAGES WITHOUT SIGNIFICANT DATA ///////////////////////////

        // Response Message without encoder data; noting to read besides the result
        case CommandByte::START_MOTOR:  
        case CommandByte::STOP_MOTOR: 
        case CommandByte::STOP_CONTROL:
            decoder_.get_result(msg.DATA[1]);
            break;

        // Gain message response upon setting the gains; noting to read besides the result
        case CommandByte::MODIFY_PARAMETER:
            decoder_.get_result(msg.DATA[2]);
            break;

        case CommandByte::MODIFY_CONFIGURATION:
            decoder_.get_result(msg.DATA[1]);
            break;
    }
}



} // namespace actuator