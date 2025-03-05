#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    :          pcan_interface_(pcan_interface),
               num_actuators_(8),
               linkage1_(five_bar_linkage_config_),
               linkage2_(five_bar_linkage_config_),
               linkage3_(five_bar_linkage_config_){


    // initialize the all reduction ratios to 1
    for (size_t i = 0; i < num_actuators_; ++i){
        pos_ratios_.push_back(1.0);
        vel_ratios_.push_back(1.0);
        trq_ratios_.push_back(1.0);
    }

    // set actuator temperature vector size
    actuators_temperature_.resize(num_actuators_);

    // set the callback function of the read_car of the PCANInterface to process the received CAN message
    pcan_interface_.set_read_callback(
    [this](const TPCANMsg &msg) {
        this->sort_can_rx_id_(msg);  // Set the callback to process CAN messages
    });

    // Initialize the actuators
    init_actuators();

    // Print the actuator info
    print_actuator_info_();

    // Initialize friction compensators with sample values 
    for (size_t i = 0; i < num_actuators_; ++i) {
        friction_compensators_.emplace_back(KarnoppCompensator(0.04, 0.01, 0.05)); // Example values
    }

    // enable the motors
    enable();

    // Empty the CAN buffer
    pcan_interface_.receive_message();

}

////////////////////////////////////////////////////////////////////////

Hand::~Hand(){
    // Stop the motion control

    set_idle_command();
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
    // Don't use this function.
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
    if (counter_ % 50 == 0){
        actuators_[0].set_joint_torque(0.0, 0);
        actuators_[1].set_joint_torque(0.0, 0);
    }
    actuators_[2].set_joint_torque(0.0, 0);
    actuators_[3].set_joint_torque(0.0, 0);
    actuators_[4].set_joint_torque(0.0, 0);
    actuators_[5].set_joint_torque(0.0, 0);
    actuators_[6].set_joint_torque(0.0, 0);
    actuators_[7].set_joint_torque(0.0, 0);
    
}


////////////////////////////////////////////////////////////////////////

void Hand::set_torque_command(const std::vector<double>& joint_torque_command, const uint32_t& duration) {
    

    // update J1 and J2 once every 10 loops
    if (counter_ % 10 == 0){
        // Dynamixel
        actuators_[0].set_joint_torque(joint_torque_command[0], duration);
        actuators_[1].set_joint_torque(joint_torque_command[1], duration);
    }

    
    for (size_t i = 2; i < num_actuators_; ++i) {
        float actuator_cmd = joint_torque_command[i];
        actuators_[i].set_joint_torque(actuator_cmd, duration);
    }

    
}

////////////////////////////////////////////////////////////////////////

void Hand::set_impedance_command(const std::vector<double> &joint_impedance_command, 
                                 const uint32_t& servo_current, 
                                 const std::vector<double> &joint_position_states, 
                                 const std::vector<double> &joint_velocity_states){ 

    if (counter_ % 10 == 0){ // update J1 and J2 once every 10 loops 
        // Dynamixel only accepts position control
        actuators_[0].set_joint_position(joint_impedance_command[0], servo_current);
        actuators_[1].set_joint_position(joint_impedance_command[1], servo_current);
    }

    for (size_t i = 2; i < num_actuators_; ++i) {
        // PD controller

        float actuator_cmd = joint_impedance_command[i] / trq_ratios_[i] * 8.0; // apply the amplification ratio from the linkage kinematics
        // clamp the actuator_cmd to the maximum torque
        actuator_cmd = std::clamp(actuator_cmd, -9.8f, 9.8f);

        actuators_[i].set_joint_torque(actuator_cmd, 0);


     
    //  std::cout << "Actuator " << i << " Impedance Command: " << actuator_cmd << std::endl;
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
    linkage1_.update_kinematics(actuators_[2].get_states().position, actuators_[3].get_states().position);
    linkage2_.update_kinematics(-actuators_[4].get_states().position, -actuators_[5].get_states().position);
    linkage3_.update_kinematics(-actuators_[6].get_states().position, -actuators_[7].get_states().position);


    pos_ratios_[3] = linkage1_.get_position_amplification();
    pos_ratios_[5] = linkage2_.get_position_amplification();
    pos_ratios_[7] = linkage3_.get_position_amplification();

    vel_ratios_[3] = 1 / linkage1_.get_torque_amplification();
    vel_ratios_[5] = 1 / linkage2_.get_torque_amplification();
    vel_ratios_[7] = 1 / linkage3_.get_torque_amplification();

    trq_ratios_[3] = linkage1_.get_torque_amplification();
    trq_ratios_[5] = linkage2_.get_torque_amplification();
    trq_ratios_[7] = linkage3_.get_torque_amplification();

    // print the pos ratios
    // for (size_t i = 0; i < num_actuators_; ++i){
    //     std::cout << "Pos Ratios: " << pos_ratios_[i] << std::endl;
    // }





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
        joint_position_states[i] = static_cast<double>(actuators_[i].get_states().position * static_cast<double>(pos_ratios_[i]));
        joint_velocity_states[i] = static_cast<double>(actuators_[i].get_states().velocity * static_cast<double>(vel_ratios_[i]));

        // Update effort only for joints 3-7
        if (i >= 2) {
            // joint_effort_states[i] = static_cast<double>(actuators_[i].get_commands().torque * static_cast<double>(trq_ratios_[i]));
            joint_effort_states[i] = static_cast<double>(actuators_[i].get_states().torque / 8.0 * static_cast<double>(trq_ratios_[i]));
        }
    }

    if (counter_ % 500 == 0){
        for (size_t i = 0; i < num_actuators_; ++i){
        actuators_temperature_[i] = static_cast<double>(actuators_[i].get_status().temperature) / 10.0;
        }
        // among the 8 actuator, get the highest temperature
        // set effort of joint 0 is highest tempature of actuator
        joint_effort_states[0] = *std::max_element(actuators_temperature_.begin(), actuators_temperature_.end());
    }

    counter_++;
}

////////////////////////////////////////////////////////////////////////

void Hand::print_motor_positions(){
    // useful function for offset calibration
    std::cout << "J 1 Position: " << actuators_[0].get_motor_position() << std::endl;
    std::cout << "J 2 Position: " << actuators_[1].get_motor_position() << std::endl;
    std::cout << "J 3 Position: " << actuators_[2].get_motor_position() << std::endl;
    std::cout << "J 4 Position: " << actuators_[3].get_motor_position() << std::endl;
    std::cout << "J 5 Position: " << actuators_[4].get_motor_position() << std::endl;
    std::cout << "J 6 Position: " << actuators_[5].get_motor_position() << std::endl;
    std::cout << "J 7 Position: " << actuators_[6].get_motor_position() << std::endl;
    std::cout << "J 8 Position: " << actuators_[7].get_motor_position() << std::endl;

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

void Hand::get_actuators_temperature(){
    for (size_t i = 0; i < num_actuators_; ++i){
        actuators_temperature_[i] = static_cast<double>(actuators_[i].get_status().temperature) / 10.0;
    }

    // among the 8 actuator, get the highest temperature
    double max_temp = *std::max_element(actuators_temperature_.begin(), actuators_temperature_.end());
}

////////////////////////////////////////////////////////////////////////


} // namespace plato2_hand