#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    :          pcan_interface_(pcan_interface),
               control_mode_(ControlMode::OFF), 
               linkage_(five_bar_linkage_config_),
               num_actuators_(8){


    // set the callback function of the read_car of the PCANInterface to process the received CAN message
    pcan_interface_.set_read_callback(
    [this](const TPCANMsg &msg) {
        this->sort_can_rx_id_(msg);  // Set the callback to process CAN messages
    });

    // Initialize the actuators
    init_actuators();

    // Print the actuator info
    print_actuator_info_();


    
}

////////////////////////////////////////////////////////////////////////

Hand::~Hand(){
    // Stop the motion control

    stop();
    std::cout << "Stopping the motion control..." << std::endl;
    
    disable();
    std::cout << "Disabling the motors..." << std::endl;
    
}

////////////////////////////////////////////////////////////////////////

void Hand::init_actuators(){

    // Reserve space in the vector to avoid reallocation
    actuators_.reserve(actuator_configs_.size());

    
    // Initialize the actuators
    for (size_t i = 0; i < actuator_configs_.size(); ++i) {
        // 'push back' the actuator to the vector of actuators
        actuators_.emplace_back(pcan_interface_, actuator_configs_[i]);
    }

    actuator_rx_id_map_ = {
        {MotorRxID::MOTOR1, &actuators_[0]},
        {MotorRxID::MOTOR2, &actuators_[1]},
        {MotorRxID::MOTOR4, &actuators_[3]},
        {MotorRxID::MOTOR3, &actuators_[2]},
        {MotorRxID::MOTOR6, &actuators_[5]},
        {MotorRxID::MOTOR5, &actuators_[4]},
        {MotorRxID::MOTOR8, &actuators_[7]},
        {MotorRxID::MOTOR7, &actuators_[6]}
    };

  

}

////////////////////////////////////////////////////////////////////////

void Hand::enable(){
    for (auto &actuator : actuators_){
        actuator.enable_motor();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::disable(){
    for (auto &actuator : actuators_){
        actuator.disable_motor();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::stop(){
    for (auto &actuator : actuators_){
        actuator.stop_control();
    }
}

////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////

void Hand::update_linkage_kinematics(){
    // Update the linkage kinematics to calculate the reduction ratios for Motor 3, 5, 7
    linkage_reduction_ratios_[2] = linkage_.update_kinematics(actuators_[2].get_states().position, actuators_[3].get_states().position);
    linkage_reduction_ratios_[4] = linkage_.update_kinematics(actuators_[4].get_states().position, actuators_[5].get_states().position);
    linkage_reduction_ratios_[6] = linkage_.update_kinematics(actuators_[6].get_states().position, actuators_[7].get_states().position);
    // the rest of the motors are 1:1 reduction ratio
}

////////////////////////////////////////////////////////////////////////


void Hand::set_control_mode(const ControlMode &control_mode){
    control_mode_ = control_mode;
}

////////////////////////////////////////////////////////////////////////

void Hand::set_commands(const std::vector<double> &joint_command, const uint32_t &duration=200){

    float actuator_cmd;
    switch (control_mode_){
        case ControlMode::OFF:
            stop();

   
            break;

        // Send zero torque command to the motors and update the joint states
        case ControlMode::IDLE:
            for (size_t i = 0; i < num_actuators_; ++i){

                actuators_[i].set_joint_torque(0.00f, duration);

            }
            break;

        case ControlMode::POSITION:
            for (size_t i = 0; i < num_actuators_; ++i){
                // Apply the reduction ratio to the joint command
                actuator_cmd = joint_command[i] / linkage_reduction_ratios_[i];
                actuators_[i].set_joint_position(actuator_cmd, duration);
            }
            break;

        case ControlMode::VELOCITY:
            for (size_t i = 0; i < num_actuators_; ++i){
                // Apply the reduction ratio to the joint command
                actuator_cmd = joint_command[i] / linkage_reduction_ratios_[i];
                actuators_[i].set_joint_velocity(actuator_cmd, duration);
            }
            break;

        case ControlMode::TORQUE:
            for (size_t i = 0; i < num_actuators_; ++i){
                // Apply the reduction ratio to the joint command
                actuator_cmd = joint_command[i] / linkage_reduction_ratios_[i];
                actuators_[i].set_joint_torque(actuator_cmd, duration);
            }
            break;

        default:
            std::cerr << "ERROR: plato2_hand::Invalid Control Mode!" << std::endl;
            break;
    }
}

////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////

void Hand::update_states(std::vector<double>&joint_position_states, std::vector<double>&joint_velocity_states, std::vector<double> &joint_effort_states){

    // recevie the message from the CAN bus every loop
    pcan_interface_.receive_message();

    // update the linkage kinematics to calculate the reduction ratios
    update_linkage_kinematics();

    switch (control_mode_){
        case ControlMode::POSITION:
        case ControlMode::VELOCITY:
        case ControlMode::TORQUE:
        case ControlMode::IDLE:
            // update the joint states for each actuator
            for (size_t i = 0; i < num_actuators_; ++i){
                joint_position_states[i] = static_cast<double>(actuators_[i].get_states().position * static_cast<double>(linkage_reduction_ratios_[i]));
                // TODO: Implement the velocity and effort states reduction ratios
                joint_velocity_states[i] = static_cast<double>(actuators_[i].get_states().velocity);
                joint_effort_states[i] = static_cast<double>(actuators_[i].get_states().torque);
            }
            break;

        case ControlMode::OFF:
            for (size_t i = 0; i < num_actuators_; ++i){
            actuators_[i].stop_control();
            }

            for (size_t i = 0; i < num_actuators_; ++i){
                joint_position_states[i] = static_cast<double>(actuators_[i].get_states().position) * static_cast<double>(linkage_reduction_ratios_[i]);
                // TODO: Implement the velocity and effort states reduction ratios
                joint_velocity_states[i] = static_cast<double>(actuators_[i].get_states().velocity);
                joint_effort_states[i] = static_cast<double>(actuators_[i].get_states().torque);
            }
            break;

        default:
            std::cerr << "ERROR: plato2_hand::Invalid Control Mode!" << std::endl;
            break;
    }

}

////////////////////////////////////////////////////////////////////////

void Hand::set_zero_positions(){
    // disable the motors
    // disable();
    
    for (size_t i=0; i < num_actuators_; ++i){
        actuators_[i].retrieve_position();
        // actuators_[i].set_zero_position(actuators_[i].get_motor_position());
        std::cout << "current zero position of actuator: " << i+1 << " is: " << actuators_[i].get_motor_position() << std::endl;
    }
    
}

////////////////////////////////////////////////////////////////////////

void Hand::sort_can_rx_id_(const TPCANMsg &msg){
      // Find the actuator corresponding to the received CAN Rx ID
    auto it = actuator_rx_id_map_.find(msg.ID);

    // Check if the key (CAN Rx ID) was found in the map
    if (it != actuator_rx_id_map_.end()) {
        // `it->second` is the Actuator* corresponding to the found CAN Rx ID
        it->second->process_message(msg);
    }
}
 

void Hand::print_actuator_info_() {  
    std::cout << "================== Actuators Info ===================" << std::endl;

    // Total number of actuators
    std::cout << "[INFO] Total Number of Actuators: " << actuators_.size() << " are initialized!" << std::endl;

    // Actuator TX and RX IDs
    for (size_t i = 0; i < num_actuators_; ++i) {
        std::cout << "[INFO] Actuator " << i + 1 << " TX ID: 0x" << std::hex << (int)actuators_[i].get_tx_id()
                  << " RX ID: 0x" << std::hex << (int)actuators_[i].get_rx_id() << std::endl;
    }



    std::cout << "====================================================" << std::endl;
}

////////////////////////////////////////////////////////////////////////

} // namespace plato2_hand