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

    std::cerr << "Actuator is being Stopped!" << std::endl;
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_position(const float &joint_position, const uint32_t &duration){
    // Check if the joint position is different from the previous command
    if (!almost_equal(joint_position, commands_.position)) {

        // Convert the joint position to motor position
        float motor_position;
        joint_to_motor_(joint_position, motor_position);

        // Encode and send the position command over CAN
        encoder_.set_position(pos_msg_, motor_position, duration);
        pcan_interface_.send_message(pos_msg_);
        
        // Cache command values
        commands_.position = joint_position;
    };
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_velocity(const float &joint_velocity, const uint32_t &duration) {
    // Check if the joint velocity is different from the previous command
    if (!almost_equal(joint_velocity, commands_.velocity)) {

        // Convert the joint velocity to motor velocity
        float motor_velocity;
        joint_to_motor_(joint_velocity, motor_velocity);

        // Encode and send the velocity command over CAN
        encoder_.set_velocity(vel_msg_, motor_velocity, duration);
        pcan_interface_.send_message(vel_msg_);
        
        // Cache command values
        commands_.velocity = joint_velocity;
    }
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_joint_torque(const float &joint_torque, const uint32_t &duration) {
    // Check if the joint torque is different from the previous command
    if (!almost_equal(joint_torque, commands_.torque)) {

        // Convert the joint torque to motor torque
        float motor_torque;
        joint_to_motor_(joint_torque, motor_torque);

        // Encode and send the torque command over CAN
        encoder_.set_torque(trq_msg_, motor_torque, duration);
        pcan_interface_.send_message(trq_msg_);
        
        // Cache command values
        commands_.torque = joint_torque;
    }
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
            decoder_.get_states(msg, status_.temperature, states_.position, states_.velocity, states_.torque);
            
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
    }
}    

} // namespace actuator