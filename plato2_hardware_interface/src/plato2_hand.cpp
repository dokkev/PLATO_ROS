#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    :          pcan_interface_(pcan_interface),
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

    // calibrate();

    // set default gains
    // set_default_gains();
    

    // get current gains
    // retrieve_runtime_gains();

    // Enable the motors
    enable();

    // Empty the CAN buffer
    pcan_interface_.receive_message();

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
    gains_.resize(num_actuators_);


    
    // Initialize the actuator instances and push them to the actuators_ vector
    for (size_t i = 0; i < actuator_configs_.size(); ++i) {
        // 'push back' the actuator to the vector of actuators
        actuators_.emplace_back(pcan_interface_, actuator_configs_[i]);
        
    }

    // Create a map of actuator Rx IDs to the corresponding Actuator object
    for (auto &actuator : actuators_) {
        actuator_rx_id_map_[actuator.get_rx_id()] = &actuator;
    }


}

////////////////////////////////////////////////////////////////////////

void Hand::enable(){
    for (auto &actuator : actuators_){
        actuator.enable_motor();
 
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::disable(){
    // Dynamixel
    actuators_[0].set_joint_position(0.0, 0);
    actuators_[1].set_joint_position(0.0, 0);
    // GIM3505
    for (size_t i=2; i < num_actuators_; ++i){
        actuators_[i].disable_motor();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::stop(){
    // Dynamixel
    actuators_[0].set_joint_position(0.0, 0);
    actuators_[1].set_joint_position(0.0, 0);
    // GIM3505
     for (size_t i=2; i < num_actuators_; ++i){
        actuators_[i].stop_control();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_idle_command() {
    actuators_[0].set_joint_torque(0.0, 1);
    actuators_[1].set_joint_torque(0.0, 1);
    actuators_[2].set_joint_torque(0.0, 0);
    actuators_[3].set_joint_torque(0.0, 0);
    actuators_[4].set_joint_torque(0.0, 0);
    actuators_[5].set_joint_torque(0.0, 0);
    actuators_[6].set_joint_torque(0.0, 0);
    actuators_[7].set_joint_torque(0.0, 0);
    
}


void Hand::set_position_command(const std::vector<double>& joint_position_command, const uint32_t& duration) {
    for (size_t i = 0; i < num_actuators_; ++i) {
        float actuator_cmd = joint_position_command[i] / linkage_reduction_ratios_[i];
        actuators_[i].set_joint_position(actuator_cmd, duration);
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_velocity_command(const std::vector<double>& joint_velocity_command, const uint32_t& duration) {
    for (size_t i = 0; i < num_actuators_; ++i) {
        float actuator_cmd = joint_velocity_command[i];
        actuators_[i].set_joint_velocity(actuator_cmd, duration);
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_torque_command(const std::vector<double>& joint_torque_command, const uint32_t& duration) {
    for (size_t i = 0; i < num_actuators_; ++i) {
        float actuator_cmd = joint_torque_command[i];
        actuators_[i].set_joint_torque(actuator_cmd, duration);
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_zero_motor_position(){
    for (size_t i = 0; i < num_actuators_; ++i){
        actuators_[i].set_joint_position(0.0, 10);
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::update_linkage_kinematics(){
    // Update the linkage kinematics to calculate the reduction ratios for Motor 3, 5, 7
    linkage_reduction_ratios_[3] = linkage_.update_kinematics(actuators_[2].get_states().position, actuators_[3].get_states().position);
    linkage_reduction_ratios_[5] = linkage_.update_kinematics(actuators_[4].get_states().position, actuators_[5].get_states().position);
    linkage_reduction_ratios_[7] = linkage_.update_kinematics(actuators_[6].get_states().position, actuators_[7].get_states().position);
    // the rest of the motors are 1:1 reduction ratio
}




////////////////////////////////////////////////////////////////////////

void Hand::update_states(std::vector<double>&joint_position_states, std::vector<double>&joint_velocity_states, std::vector<double> &joint_effort_states){

    // recevie the message from the CAN bus every loop
    pcan_interface_.receive_message();

    // update the linkage kinematics to calculate the reduction ratios
    update_linkage_kinematics();

    // update the joint states for each actuator
    for (size_t i = 0; i < num_actuators_; ++i){
        joint_position_states[i] = static_cast<double>(actuators_[i].get_states().position * static_cast<double>(linkage_reduction_ratios_[i]));
        // TODO: Implement the velocity and effort states reduction ratios
        joint_velocity_states[i] = static_cast<double>(actuators_[i].get_states().velocity);
        joint_effort_states[i] = static_cast<double>(actuators_[i].get_states().torque);
    }


}

////////////////////////////////////////////////////////////////////////

void Hand::print_motor_positions(){
    // disable the motors
    // disable();
    // print actuator 5 and 6 positions

    std::cout << "J 1 Position: " << actuators_[0].get_motor_position() << std::endl;
    std::cout << "J 2 Position: " << actuators_[1].get_motor_position() << std::endl;
    std::cout << "J 3 Position: " << actuators_[3].get_motor_position() << std::endl;
    std::cout << "J 4 Position: " << actuators_[2].get_motor_position() << std::endl;
    std::cout << "J 5 Position: " << actuators_[5].get_motor_position() << std::endl;
    std::cout << "J 6 Position: " << actuators_[4].get_motor_position() << std::endl;
    std::cout << "J 7 Position: " << actuators_[7].get_motor_position() << std::endl;
    std::cout << "J 8 Position: " << actuators_[6].get_motor_position() << std::endl;


}

////////////////////////////////////////////////////////////////////////

void Hand::set_default_gains(){
    for (size_t i=2; i < num_actuators_; ++i){

        actuators_[i].set_default_gains(default_gains[i]);
    }
    // receive the message from the CAN bus
    
}


void Hand::retrieve_runtime_gains(){
    // Send a command message to retrieve the gains of the motor
    for (size_t i=2; i < num_actuators_; ++i){
        actuators_[i].retrieve_gains();
    }

    // receive the message from the CAN bus
    

    // get gains from each actuator and store them in the gains_ vector
    for (size_t i=0; i < num_actuators_; ++i){
        gains_[i] = actuators_[i].get_gains();
    } 

    // Actuator Initial Gains
    // Change the output to decimal
    std::cout << std::dec << std::endl;
    for (size_t i = 0; i < num_actuators_; ++i) {
        std::cout << "[INFO] Actuator " << i + 1 << " Gains: " << std::endl;
        std::cout << "  Kp Velocity: " << gains_[i].kp_velocity << std::endl;
        std::cout << "  Ki Velocity: " << gains_[i].ki_velocity << std::endl;
        std::cout << "  Kp Position: " << gains_[i].kp_position << std::endl;
        std::cout << "  Ki Position: " << gains_[i].ki_position << std::endl;
        std::cout << "  Kd Position: " << gains_[i].kd_position << std::endl;

    }
}

////////////////////////////////////////////////////////////////////////

void Hand::calibrate(){
    for (size_t i=2; i < num_actuators_; ++i){
        actuators_[i].calibrate_encoder();

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
 
////////////////////////////////////////////////////////////////////////

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