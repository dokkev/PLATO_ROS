#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand() : control_mode_(ControlMode::OFF){

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

    // Retrieve the initial position of the actuators
    for (auto &actuator : actuators_){
        actuator.retrieve_position();
    }

    // print actuator position
    for (auto &actuator : actuators_){

        std::cout << actuator.get_states().position << std::endl;
    }

    // Print actuator Size

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

void Hand::set_commands(const double &joint_command, const uint32_t &duration=200){

    //convert double command to float
    float command = static_cast<float>(joint_command);

    switch (control_mode_){
        case ControlMode::OFF:
            stop();
            break;

        // Send zero torque command to the motors and update the joint states
        case ControlMode::IDLE:
            for (auto &actuator : actuators_){
                actuator.set_joint_torque(0.0, duration);
            }
            break;

        case ControlMode::POSITION:
            for (auto &actuator : actuators_){
                actuator.set_joint_position(command, duration);
            }
            break;

        case ControlMode::VELOCITY:
            for (auto &actuator : actuators_){
                actuator.set_joint_velocity(command, duration);
            }
            break;

        case ControlMode::TORQUE:
            for (auto &actuator : actuators_){
                actuator.set_joint_torque(command, duration);
            }
            break;

        default:
            std::cerr << "ERROR: plato2_hand::Invalid Control Mode!" << std::endl;
            break;
    }
}

void Hand::update_states(double &joint_position_states, double &joint_velocity_states, double &joint_effort_states){

    // recevie the message from the CAN bus every loop
    pcan_interface_.receive_message();

    switch (control_mode_){
        case ControlMode::POSITION:
        case ControlMode::VELOCITY:
        case ControlMode::TORQUE:
        case ControlMode::IDLE:
            // update the joint states for each actuator
            for (size_t i = 0; i < actuators_.size(); ++i){
            //     // joint_position_states[i] = actuators_[i].get_states().position;
            //     // joint_velocity_states[i] = actuators_[i].get_states().velocity;
            //     // joint_effort_states[i] = actuators_[i].get_states().effort;
            }
            
            break;

 

        case ControlMode::OFF:
            for (size_t i = 0; i < actuators_.size(); ++i){
                // joint_position_states[i] = actuators_[i].get_states().position;
            }
   
            break;

        default:
            std::cerr << "ERROR: plato2_hand::Invalid Control Mode!" << std::endl;
            break;
    }

}

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
    for (size_t i = 0; i < actuators_.size(); ++i) {
        std::cout << "[INFO] Actuator " << i + 1 << " TX ID: 0x" << std::hex << (int)actuators_[i].get_tx_id()
                  << " RX ID: 0x" << std::hex << (int)actuators_[i].get_rx_id() << std::endl;
    }

    std::cout << "====================================================" << std::endl;
}

////////////////////////////////////////////////////////////////////////

} // namespace plato2_hand