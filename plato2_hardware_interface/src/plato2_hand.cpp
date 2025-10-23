#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    :          pcan_interface_(pcan_interface),
               num_actuators_(8),
               num_ft_sensors_(3){

    // set actuator temperature vector size
    actuators_temperature_.resize(num_actuators_);


    // Initialize the actuators
    init_can_hardware();

    // Print the actuator info
    print_actuator_info_();

    // enable the motors
    // enable();

    // Empty the CAN buffer
    pcan_interface_.receive_message();

}

////////////////////////////////////////////////////////////////////////

Hand::~Hand(){
    // Stop the motion control
    std::cout << "Stopping the motion control..." << std::endl;
    for (size_t i=0; i < num_actuators_; ++i){
        actuators_[i].disable_motor();
    }
    std::cout << "Disabling the motors..." << std::endl;
    
}

////////////////////////////////////////////////////////////////////////

void Hand::init_can_hardware() {
    // Reserve space to avoid reallocation
    actuators_.reserve(actuator_configs_.size());
    ft_sensors_.reserve(ft_sensor_configs_.size());

    // Initialize actuators
    for (const auto &config : actuator_configs_) {
        actuators_.emplace_back(pcan_interface_, config);
    }

    // Initialize force-torque sensors
    for (const auto &config : ft_sensor_configs_) {
        ft_sensors_.emplace_back(pcan_interface_, config);
    }

    // Create a unified map for sorting CAN messages
    for (auto &actuator : actuators_) {
        actuator_rx_id_map_[actuator.get_rx_id()] = &actuator;
    }

    for (auto &sensor : ft_sensors_) {
        ft_sensor_rx_id_map_[sensor.get_force_rx_id()] = &sensor;
        ft_sensor_rx_id_map_[sensor.get_torque_rx_id()] = &sensor;
    }

    // Register a single callback for processing all CAN messages
    pcan_interface_.set_read_callback(
        [this](const TPCANMsg &msg) {
            auto act_it = actuator_rx_id_map_.find(msg.ID);
            if (act_it != actuator_rx_id_map_.end()) {
                act_it->second->process_message(msg);
                return;
            }

            auto sensor_it = ft_sensor_rx_id_map_.find(msg.ID);
            if (sensor_it != ft_sensor_rx_id_map_.end()) {
                sensor_it->second->process_message(msg);
                return;
            }
        }
    );

    std::cout << "Hardware initialization completed: Actuators and FT sensors initialized." << std::endl;
}


////////////////////////////////////////////////////////////////////////

void Hand::enable(){
    for (auto &actuator : actuators_){
        actuator.enable_motor();
 
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::disable(){
    // GIM3505

}

////////////////////////////////////////////////////////////////////////

void Hand::stop(){
    // Don't use this function.
    // GIM3505
     for (size_t i=0; i < num_actuators_; ++i){
        actuators_[i].stop_control();
    }
}


////////////////////////////////////////////////////////////////////////

void Hand::set_impedance_command(const std::vector<double> &joint_position_command, 
                                 const std::vector<double> &joint_velocity_command,
                                 const std::vector<double> &joint_stiffness_command,
                                 const std::vector<double> &joint_damping_command,
                                 const std::vector<double> &joint_torque_command) {

    // The controller provides a RELATIVE angle for the PIP joint (relative to the MCP link).
    // The hardware needs an ABSOLUTE angle for the PIP motor (relative to the palm).
    // The conversion is: pip_motor_absolute = pip_joint_relative + mcp_joint_absolute_STATE.
    // This compensation is applied directly within the loop for the affected joints.

    for (size_t i = 0; i < num_actuators_; ++i) {
        float pos_cmd = joint_position_command[i];

        // Apply compensation for PIP joints by adding the measured state of the parent MCP joint.
        if (i == 3) { // Index finger PIP (parent MCP is joint 2)
            const auto& mcp_state = actuators_[2].get_states();
            pos_cmd += mcp_state.position;
        } else if (i == 5) { // Middle finger PIP (parent MCP is joint 4)
            const auto& mcp_state = actuators_[4].get_states();
            pos_cmd += mcp_state.position;
        } else if (i == 7) { // Ring/Pinky finger PIP (parent MCP is joint 6)
            const auto& mcp_state = actuators_[6].get_states();
            pos_cmd += mcp_state.position;
        }

        // Send the final command (compensated or direct) to the actuator.
        actuators_[i].set_joint_impedance(
            pos_cmd, 
            joint_velocity_command[i], 
            joint_stiffness_command[i], 
            joint_damping_command[i], 
            joint_torque_command[i]
        );
    }
}

////////////////////////////////////////////////////////////////////////


void Hand::set_current_position_as_zero(){
    for (size_t i = 0; i < num_actuators_; ++i){
        actuators_[i].set_current_position_as_zero();
    }
}


////////////////////////////////////////////////////////////////////////

void Hand::update_joint_states(std::vector<double>&joint_position_states, std::vector<double>&joint_velocity_states, std::vector<double> &joint_effort_states){
    // recevie the message from the CAN bus every loop
    pcan_interface_.receive_message();

    for (size_t i = 0; i < num_actuators_; ++i){
        joint_position_states[i] = static_cast<double>(actuators_[i].get_states().position);
        joint_velocity_states[i] = static_cast<double>(actuators_[i].get_states().velocity);
        joint_effort_states[i] = static_cast<double>(actuators_[i].get_states().torque);
    }

    // Compensation for decoupled PIP joints (actuators are grounded):
    // In the URDF, fingers are modeled as 2RR (MCP followed by PIP).
    // The physical linkage means pip_joint_angle = pip_motor_angle - mcp_motor_angle.
    // We apply this correction to the reported states for joints 3, 5, and 7.

    // Index finger (PIP joint 3, MCP joint 2)
    joint_position_states[3] -= joint_position_states[2];


    // Middle finger (PIP joint 5, MCP joint 4)
    joint_position_states[5] -= joint_position_states[4];


    // Ring/Pinky finger (PIP joint 7, MCP joint 6)
    joint_position_states[7] -= joint_position_states[6];

    

    counter_++;
}


////////////////////////////////////////////////////////////////////////

void Hand::update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> &ft_sensor_states){
    // receive the message from the CAN bus every loop

    pcan_interface_.receive_message();

    for (size_t i = 0; i < ft_sensors_.size(); ++i) {
        auto states = ft_sensors_[i].get_states();
        ft_sensor_states[i].force.x = states.force_filtered.x();
        ft_sensor_states[i].force.y = states.force_filtered.y();
        ft_sensor_states[i].force.z = states.force_filtered.z();
        ft_sensor_states[i].torque.x = states.torque_filtered.x();
        ft_sensor_states[i].torque.y = states.torque_filtered.y();
        ft_sensor_states[i].torque.z = states.torque_filtered.z();
    }

    // update the FT sensor states
    
    
}


////////////////////////////////////////////////////////////////////////

void Hand::print_motor_positions(){
    // useful function for offset calibration
    std::cout << "J1 Motor Position: " << actuators_[0].get_motor_position() << std::endl;
    std::cout << "J2 Motor Position: " << actuators_[1].get_motor_position() << std::endl;
    std::cout << "J3 Motor Position: " << actuators_[2].get_motor_position() << std::endl;
    std::cout << "J4 Motor Position: " << actuators_[3].get_motor_position() << std::endl;
    std::cout << "J5 Motor Position: " << actuators_[4].get_motor_position() << std::endl;
    std::cout << "J6 Motor Position: " << actuators_[5].get_motor_position() << std::endl;
    std::cout << "J7 Motor Position: " << actuators_[6].get_motor_position() << std::endl;
    std::cout << "J8 Motor Position: " << actuators_[7].get_motor_position() << std::endl;

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
        std::cout << "[INFO] Actuator " << i + 1 
                  << " TX ID: 0x" << std::hex << (int)actuators_[i].get_tx_id()
                  << " RX ID: 0x" << std::hex << (int)actuators_[i].get_rx_id()

                  << std::endl;
    }
    
    std::cout << "===================FT Sensors Info ==================" << std::endl;

    // Total number of FT sensors
    std::cout << "[INFO] Total Number of FT Sensors: " << ft_sensors_.size() << " are initialized!" << std::endl;

    // FT Sensor force and torque RX IDs
    for (size_t i = 0; i < ft_sensors_.size(); ++i) {
        std::cout << "[INFO] FT Sensor " << i + 1 << " Force RX ID: 0x" << std::hex << (int)ft_sensors_[i].get_force_rx_id()
                  << " Torque RX ID: 0x" << std::hex << (int)ft_sensors_[i].get_torque_rx_id() << std::endl;
    }
}


////////////////////////////////////////////////////////////////////////


} // namespace plato2_hand