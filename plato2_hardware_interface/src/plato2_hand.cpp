#include <plato2_hardware_interface/plato2_hand.hpp>


namespace plato2_hand{

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    :          pcan_interface_(pcan_interface),
               num_actuators_(8),
               num_ft_sensors_(3){


    // initialize the all reduction ratios to 1
    for (size_t i = 0; i < num_actuators_; ++i){
        pos_ratios_.push_back(1.0);
        vel_ratios_.push_back(1.0);
        trq_ratios_.push_back(1.0);
    }

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

    set_idle_command();
    std::cout << "Stopping the motion control..." << std::endl;
    
    disable();
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

            // std::cerr << "Unknown CAN message ID: 0x" << std::hex << msg.ID << std::dec << std::endl;
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
    for (size_t i=0; i < num_actuators_; ++i){
        actuators_[i].disable_motor();
    }
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

void Hand::set_idle_command() {

    // viscous damping coefficient (N·m·s/rad)
    const double b = 0.0;

    // apply damping to actuators 0 through 7
    for (size_t i = 0; i < actuators_.size(); ++i) {
        double vel = actuators_[i].get_states().velocity;
        double tau_damp = -b * vel;
        actuators_[i].set_joint_torque(tau_damp, 0);
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

void Hand::set_impedance_command(const std::vector<double> &joint_position_command, 
                                 const std::vector<double> &joint_velocity_command,
                                 const std::vector<double> &joint_stiffness_command,
                                 const std::vector<double> &joint_damping_command,
                                 const std::vector<double> &joint_torque_command) {

    // Create local copies of command vectors to apply compensation
    auto pos_cmd_compensated = joint_position_command;
    auto vel_cmd_compensated = joint_velocity_command;

    // Compensate for the PIP joints (indices 3, 5, 7) using the CURRENT MEASURED STATE of the MCP joints.
    // The controller provides a RELATIVE angle for the PIP joint (relative to the MCP link).
    // The hardware needs an ABSOLUTE angle for the PIP motor (relative to the palm).
    // The conversion is: pip_motor_absolute = pip_joint_relative + mcp_joint_absolute_STATE.
    
    // Index finger (PIP joint 3, MCP joint 2)
    pos_cmd_compensated[3] = joint_position_command[3] + actuators_[2].get_states().position;
    vel_cmd_compensated[3] = joint_velocity_command[3] + actuators_[2].get_states().velocity;

    // Middle finger (PIP joint 5, MCP joint 4)
    pos_cmd_compensated[5] = joint_position_command[5] + actuators_[4].get_states().position;
    vel_cmd_compensated[5] = joint_velocity_command[5] + actuators_[4].get_states().velocity;

    // Ring/Pinky finger (PIP joint 7, MCP joint 6)
    pos_cmd_compensated[7] = joint_position_command[7] + actuators_[6].get_states().position;
    vel_cmd_compensated[7] = joint_velocity_command[7] + actuators_[6].get_states().velocity;

    for (size_t i = 0; i < num_actuators_; ++i) {
        float pos_cmd = pos_cmd_compensated[i];
        float vel_cmd = vel_cmd_compensated[i];
        float kp = joint_stiffness_command[i];
        float kd = joint_damping_command[i];
        float torque_cmd = joint_torque_command[i];
        actuators_[i].set_joint_impedance(pos_cmd, vel_cmd, kp, kd, torque_cmd);
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