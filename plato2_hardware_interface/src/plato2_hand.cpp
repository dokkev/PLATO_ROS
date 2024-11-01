#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    :          pcan_interface_(pcan_interface),
               linkage_(five_bar_linkage_config_),
               num_actuators_(8),
               control_mode_(ControlMode::OFF){


    // set the callback function of the read_car of the PCANInterface to process the received CAN message
    pcan_interface_.set_read_callback(
    [this](const TPCANMsg &msg) {
        this->sort_can_rx_id_(msg);  // Set the callback to process CAN messages
    });

    // Initialize the actuators
    init_actuators();

    // Print the actuator info
    print_actuator_info_();

    initialize_command_functions_();
    set_control_mode(control_mode_);


    
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
    linkage_reduction_ratios_[3] = linkage_.update_kinematics(actuators_[2].get_states().position, actuators_[3].get_states().position);
    linkage_reduction_ratios_[5] = linkage_.update_kinematics(actuators_[4].get_states().position, actuators_[5].get_states().position);
    linkage_reduction_ratios_[7] = linkage_.update_kinematics(actuators_[6].get_states().position, actuators_[7].get_states().position);
    // the rest of the motors are 1:1 reduction ratio
}

////////////////////////////////////////////////////////////////////////


void Hand::set_control_mode(const ControlMode& control_mode) {
    control_mode_ = control_mode;
    auto it = command_function_map_.find(control_mode);
    if (it != command_function_map_.end()) {
        command_mode_function_ = it->second;
    } else {
        std::cerr << "ERROR: Unsupported control mode." << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_commands(const std::vector<double>& joint_command, const uint32_t& duration) {
    if (command_mode_function_) {
        command_mode_function_(joint_command, duration);
    }
    else {
        std::cerr << "Hand::set_commands ERROR: Control mode function not set." << std::endl;
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
        case ControlMode::GRASP:
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
                joint_position_states[i] = static_cast<double>(actuators_[i].get_states().position);
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

void Hand::print_motor_positions(){
    // disable the motors
    // disable();
    // print actuator 5 and 6 positions

    std::cout << "J 3 Position: " << actuators_[3].get_motor_position() << std::endl;
    std::cout << "J 4 Position: " << actuators_[2].get_motor_position() << std::endl;
    std::cout << "J 5 Position: " << actuators_[5].get_motor_position() << std::endl;
    std::cout << "J 6 Position: " << actuators_[4].get_motor_position() << std::endl;
    std::cout << "J 7 Position: " << actuators_[7].get_motor_position() << std::endl;
    std::cout << "J 8 Position: " << actuators_[6].get_motor_position() << std::endl;


}

////////////////////////////////////////////////////////////////////////

void Hand::set_current_position_as_zero(){
    for (auto &actuator : actuators_){
        actuator.set_zero_position(actuator.get_states().position);
    }
}


void Hand::set_position_command_(const std::vector<double>& joint_position_command, const uint32_t& duration) {
    for (size_t i = 0; i < num_actuators_; ++i) {
        float actuator_cmd = joint_position_command[i] / linkage_reduction_ratios_[i];
        actuators_[i].set_joint_position(actuator_cmd, duration);
    }
}

void Hand::set_velocity_command_(const std::vector<double>& joint_velocity_command, const uint32_t& duration) {
    for (size_t i = 0; i < num_actuators_; ++i) {
        float actuator_cmd = joint_velocity_command[i];
        actuators_[i].set_joint_velocity(actuator_cmd, duration);
    }
}

void Hand::set_torque_command_(const std::vector<double>& joint_torque_command, const uint32_t& duration) {
    for (size_t i = 0; i < num_actuators_; ++i) {
        float actuator_cmd = joint_torque_command[i];
        actuators_[i].set_joint_torque(actuator_cmd, duration);
    }
}

void Hand::set_grasp_command_() {
    float grasping_torque = 0.06f;

    actuators_[2].set_joint_torque(-grasping_torque, 200);
    actuators_[3].set_joint_torque(-grasping_torque, 200);
    actuators_[4].set_joint_torque(grasping_torque, 200);
    actuators_[5].set_joint_torque(grasping_torque, 200);
    actuators_[6].set_joint_torque(0.0, 200);
    actuators_[7].set_joint_torque(0.0, 200);
}

void Hand::set_idle_command_() {
    for (auto& actuator : actuators_) {
        actuator.set_joint_torque(0.0f, 200);
    }
}

void Hand::set_off_command_() {
    stop();
}

////////////////////////////////////////////////////////////////////////

void Hand::initialize_command_functions_() {
    command_function_map_ = {
        {ControlMode::OFF, [this](const std::vector<double>&, const uint32_t&) { set_off_command_(); }},
        {ControlMode::IDLE, [this](const std::vector<double>&, const uint32_t& duration) { set_idle_command_(); }},
        {ControlMode::POSITION, [this](const std::vector<double>& command, const uint32_t& duration) { set_position_command_(command, duration); }},
        {ControlMode::VELOCITY, [this](const std::vector<double>& command, const uint32_t& duration) { set_velocity_command_(command, duration); }},
        {ControlMode::TORQUE, [this](const std::vector<double>& command, const uint32_t& duration) { set_torque_command_(command, duration); }},
        {ControlMode::GRASP, [this](const std::vector<double>& command, const uint32_t& duration) { set_grasp_command_(); }}

    };
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