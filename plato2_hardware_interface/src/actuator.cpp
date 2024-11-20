#include "plato2_hardware_interface/actuator.hpp"


namespace actuator{

////////////////////////////////////////////////////////////////////////////
////////////////////////////// ACTUATOR CLASS //////////////////////////////
////////////////////////////////////////////////////////////////////////////

Actuator::Actuator(pcan_interface::PCANInterface &pcan_interface, Config& config) 
    :   pcan_interface_(pcan_interface), 
        config_(config),
        encoder_(config.gear_ratio, config.torque_constant),
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
}

Actuator::~Actuator(){
    stop_control();
}

////////////////////////////////////////////////////////////////////////////

void Actuator::enable_motor(){

    // while (!b_motor_enabled_){
        encoder_.start_motor(onoff_msg_);
        pcan_interface_.send_message(onoff_msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pcan_interface_.receive_message();
    // }

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
        pcan_interface_.send_message(gain_msg_);
        gains_.kp_velocity = new_gains.kp_velocity;  // Update cached gain
    }

    if (new_gains.ki_velocity != gains_.ki_velocity) {
        encoder_.set_gain(gain_msg_, new_gains.ki_velocity, ParamID::KI_SPEED);
        pcan_interface_.send_message(gain_msg_);
        gains_.ki_velocity = new_gains.ki_velocity;  // Update cached gain
    }

    if (new_gains.kp_position != gains_.kp_position) {
        encoder_.set_gain(gain_msg_, new_gains.kp_position, ParamID::KP_POSITION);
        pcan_interface_.send_message(gain_msg_);
        gains_.kp_position = new_gains.kp_position;  // Update cached gain
    }

    if (new_gains.ki_position != gains_.ki_position) {
        encoder_.set_gain(gain_msg_, new_gains.ki_position, ParamID::KI_POSITION);
        pcan_interface_.send_message(gain_msg_);
        gains_.ki_position = new_gains.ki_position;  // Update cached gain
    }

    if (new_gains.kd_position != gains_.kd_position) {
        encoder_.set_gain(gain_msg_, new_gains.kd_position, ParamID::KD_POSITION);
        pcan_interface_.send_message(gain_msg_);
        gains_.kd_position = new_gains.kd_position;  // Update cached gain
    }
}

////////////////////////////////////////////////////////////////////////////

void Actuator::set_default_gains(const Gains &new_default_gains){
    

        encoder_.set_default_gain(gain_msg_, new_default_gains.kp_velocity, IntConfigID::KP_SPEED);
        pcan_interface_.send_message(gain_msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pcan_interface_.receive_message();



        encoder_.set_default_gain(gain_msg_, new_default_gains.ki_velocity, IntConfigID::KI_SPEED);
        pcan_interface_.send_message(gain_msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pcan_interface_.receive_message();
 


        encoder_.set_default_gain(gain_msg_, new_default_gains.kp_position, IntConfigID::KP_POSITION);
        pcan_interface_.send_message(gain_msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pcan_interface_.receive_message();
   


        encoder_.set_default_gain(gain_msg_, new_default_gains.ki_position, IntConfigID::KI_POSITION);
        pcan_interface_.send_message(gain_msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pcan_interface_.receive_message();
    

 
        encoder_.set_default_gain(gain_msg_, new_default_gains.kd_position, IntConfigID::KD_POSITION);
        pcan_interface_.send_message(gain_msg_);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        pcan_interface_.receive_message();
   
}

////////////////////////////////////////////////////////////////////////////

void Actuator::retrieve_position(){
    encoder_.retrieve_position(ind_msg_);
    pcan_interface_.send_message(ind_msg_);
}

////////////////////////////////////////////////////////////////////////////

void Actuator::retrieve_gains(){
    
    encoder_.retrieve_gain(gain_msg_, ParamID::KP_SPEED);
    pcan_interface_.send_message(gain_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();

    encoder_.retrieve_gain(gain_msg_, ParamID::KI_SPEED);
    pcan_interface_.send_message(gain_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();

    encoder_.retrieve_gain(gain_msg_, ParamID::KP_POSITION);
    pcan_interface_.send_message(gain_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();

    encoder_.retrieve_gain(gain_msg_, ParamID::KI_POSITION);
    pcan_interface_.send_message(gain_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();

    encoder_.retrieve_gain(gain_msg_, ParamID::KD_POSITION);
    pcan_interface_.send_message(gain_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();
}


////////////////////////////////////////////////////////////////////////////

void Actuator::set_zero_position(const float &zero_position){
    encoder_.set_zero_position(config_msg_, zero_position);
    pcan_interface_.send_message(config_msg_);
}

////////////////////////////////////////////////////////////////////////////

void Actuator::calibrate_encoder(){
    encoder_.calibrate_encoder(config_msg_);
    pcan_interface_.send_message(config_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();
}

void Actuator::calibrate_phase_order(){
    encoder_.calibrate_phase_order(config_msg_);
    pcan_interface_.send_message(config_msg_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    pcan_interface_.receive_message();
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
            // Get the states if there is a valid response without any error
            if (decoder_.get_result(msg.DATA[1])) {
                // Decode the states
                float motor_position, motor_velocity, motor_torque;
                decoder_.get_states(msg, status_.temperature, motor_position, motor_velocity, motor_torque);
                
                // Store motor_position without offset
                motor_position_ = motor_position;

                
                // Convert velocity units from RPM to rad/s
                motor_velocity = motor_velocity * 2.0f * M_PI / 60.0f;

                // Apply the motor-to-joint conversion and store the states
                motor_to_joint_(motor_position, states_.position, true);
                motor_to_joint_(motor_velocity, states_.velocity);
                motor_to_joint_(motor_torque, states_.torque);
            } else {
                std::cerr << "Actuator ID: 0x" <<  std::hex <<config_.can_tx_id << " Control Failed" << std::endl;
            }

            break;

        // Gain message reponse upon request to get the gains from the motor
        case CommandByte::RETRIVE_PARAMETER:
            // If the parameter exists in the map, retrieve the pointer to the corresponding gain variable, dereference it, and pass it to the get_gain function.
            // uint8_t gain_byte = msg.DATA[1];
            // check if the parameter exists in the map

            if (decoder_.get_result(msg.DATA[2])){

                if (msg.DATA[1] == ParamID::KP_SPEED){
                    decoder_.get_gain(msg, gains_.kp_velocity);
        
                } else if (msg.DATA[1] == ParamID::KI_SPEED){
                    decoder_.get_gain(msg, gains_.ki_velocity);
   
                } else if (msg.DATA[1] == ParamID::KP_POSITION){
                    decoder_.get_gain(msg, gains_.kp_position);
     
                } else if (msg.DATA[1] == ParamID::KI_POSITION){
                    decoder_.get_gain(msg, gains_.ki_position);
          
                } else if (msg.DATA[1] == ParamID::KD_POSITION){
                    decoder_.get_gain(msg, gains_.kd_position);

                } else {
                    std::cerr << "Unknown Gain Parameter Byte: " << +msg.DATA[1] << std::endl;
                }
            } else {
                std::cerr << "Actuator ID: 0x" << std::hex << msg.ID << " failed to retrieve gains" << std::endl;
            }

            // try {
            //     decoder_.get_gain(msg, *gain_map_.at(msg.DATA[1]));
            //     std::cout << "Gain Parameter Byte: " << +msg.DATA[1] << " Value: " << *gain_map_.at(msg.DATA[1]) << std::endl;
            // } catch (const std::out_of_range& e) {
            //     std::cerr << "Gain Parameter Byte not found in gain_map_: " << +msg.DATA[1] << std::endl;
            // }

            break;

        case CommandByte::RETRIVE_INDICATOR:
            // get the position if there is a valid response without any error
            std::cerr << "Unimplemented Feature [Glitch found]" << std::endl;
            break;


        // Response Message without encoder data; noting to read besides the result
        case CommandByte::START_MOTOR:  
            if(decoder_.get_result(msg.DATA[1])){
                b_motor_enabled_ = true;
                std::cout << "Actuator ID: 0x" << std::hex << msg.ID << " enabled!" << std::endl;
            } else {
                std::cerr << "Actuator ID: 0x" << std::hex << msg.ID << " failed to enable motor" << std::endl;
            }
            break;
        case CommandByte::STOP_MOTOR:
            if (decoder_.get_result(msg.DATA[1])){
                std::cout << "Actuator ID: 0x" << std::hex << msg.ID << " Disabled!" << std::endl;
            } else {
                std::cerr << "Actuator ID: 0x" << std::hex << msg.ID << " failed to stop motor" << std::endl;
            }
            break; 
        case CommandByte::STOP_CONTROL:
            if (decoder_.get_result(msg.DATA[1])){
                b_motor_enabled_ = false;
                std::cout << "Actuator ID: 0x" << std::hex << msg.ID << " Stop Control!" << std::endl;
            } else {
                std::cerr << "Actuator ID: 0x" << std::hex << msg.ID << " failed to disable motor" << std::endl;
            }
            break;

        case CommandByte::CALIBRATE:
            if (decoder_.get_result(msg.DATA[2])){
                std::cout << "Actuator ID: 0x" << std::hex << msg.ID << " Calibrated!" << std::endl;
            } else {
                std::cerr << "Actuator ID: 0x" << std::hex << msg.ID << " failed to calibrate" << std::endl;
            }
            break;

        // Gain message response upon setting the gains; noting to read besides the result
        case CommandByte::MODIFY_PARAMETER:
            decoder_.get_result(msg.DATA[2]);
            break;

        case CommandByte::MODIFY_CONFIGURATION:
     
            if (decoder_.get_result(msg.DATA[3]) && msg.DATA[1] == ConfigType::INT32){

                std::cout << "Actuator ID: 0x" << std::hex << msg.ID;

                if (msg.DATA[3] == IntConfigID::KP_SPEED){
                    gains_.b_kp_velocity_updated = true;
                    std::cout << " Kp Velocity ";

                } else if (msg.DATA[3] == IntConfigID::KI_SPEED){
                    gains_.b_ki_velocity_updated = true;
                    std::cout << " Ki Velocity ";

                } else if (msg.DATA[3] == IntConfigID::KP_POSITION){
                    gains_.b_kp_position_updated = true;
                    std::cout << " Kp Position ";

                } else if (msg.DATA[3] == IntConfigID::KI_POSITION){
                    gains_.b_ki_position_updated = true;
                    std::cout << " Ki Position ";

                } else if (msg.DATA[3] == IntConfigID::KD_POSITION){
                    gains_.b_kd_position_updated = true;
                    std::cout << " Kd Position ";
                }

                std:: cout <<" Initial Gain Modification Sucessful!" << std::endl;

            } else {
                std::cerr << "Actuator ID: 0x" << std::hex << msg.ID << " failed to modify configuration" << std::endl;
            }
            break;
    }
}



} // namespace actuator