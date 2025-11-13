#include <plato2_hardware_interface/plato2_hand.hpp>

namespace plato2_hand {

Hand::Hand(pcan_interface::PCANInterface &pcan_interface) 
    : pcan_interface_(pcan_interface),
      num_actuators_(8),
      num_ft_sensors_(3) {

    init_can_hardware();
    print_actuator_info_();
    
    // Empty the CAN buffer
    pcan_interface_.receive_message();
}

////////////////////////////////////////////////////////////////////////

Hand::~Hand() {
    std::cout << "Disabling motors..." << std::endl;
    for (auto &actuator : actuators_) {
        actuator.disable_motor();
    }
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

void Hand::enable() {
    for (auto &actuator : actuators_) {
        actuator.enable_motor();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::disable() {
    for (auto &actuator : actuators_) {
        actuator.disable_motor();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_current_position_as_zero() {
    for (auto &actuator : actuators_) {
        actuator.set_current_position_as_zero();
    }
}

////////////////////////////////////////////////////////////////////////

void Hand::set_impedance_command(const std::vector<double> &joint_position_command, 
                                 const std::vector<double> &joint_velocity_command,
                                 const std::vector<double> &joint_stiffness_command,
                                 const std::vector<double> &joint_damping_command,
                                 const std::vector<double> &joint_torque_command) {

    // Apply kinematic compensation for PIP joints (3, 5, 7).
    // The URDF models PIP joints relative to their MCP link,
    // but the hardware motors are grounded to the palm.
    // Conversion: pip_motor_absolute = pip_joint_relative + mcp_joint_state

    for (size_t i = 0; i < num_actuators_; ++i) {
        float pos_cmd = joint_position_command[i];

        // Compensate PIP joints by adding the current MCP state
        if (i == 3) { // Index finger PIP (parent is MCP joint 2)
            pos_cmd += actuators_[2].get_states().position;
        } else if (i == 5) { // Middle finger PIP (parent is MCP joint 4)
            pos_cmd += actuators_[4].get_states().position;
        } else if (i == 7) { // Ring/Pinky finger PIP (parent is MCP joint 6)
            pos_cmd += actuators_[6].get_states().position;
        }

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

void Hand::update_joint_states(std::vector<double> &joint_position_states, 
                               std::vector<double> &joint_velocity_states, 
                               std::vector<double> &joint_effort_states) {
    // Receive CAN messages
    pcan_interface_.receive_message();

    // Read raw motor states
    for (size_t i = 0; i < num_actuators_; ++i) {
        const auto& states = actuators_[i].get_states();
        joint_position_states[i] = static_cast<double>(states.position);
        joint_velocity_states[i] = static_cast<double>(states.velocity);
        joint_effort_states[i] = static_cast<double>(states.torque);
    }

    // Apply kinematic compensation for PIP joints.
    // The URDF models PIP joints relative to their MCP link,
    // but the hardware reports absolute motor angles relative to the palm.
    // Conversion: pip_joint_relative = pip_motor_absolute - mcp_motor_absolute
    joint_position_states[3] -= joint_position_states[2];  // Index finger
    joint_position_states[5] -= joint_position_states[4];  // Middle finger
    joint_position_states[7] -= joint_position_states[6];  // Ring/Pinky finger
}


////////////////////////////////////////////////////////////////////////

void Hand::update_ft_sensor_states(std::vector<geometry_msgs::msg::Wrench> &ft_sensor_states) {
    pcan_interface_.receive_message();

    for (size_t i = 0; i < ft_sensors_.size(); ++i) {
        const auto& states = ft_sensors_[i].get_states();
        ft_sensor_states[i].force.x = states.force_filtered.x();
        ft_sensor_states[i].force.y = states.force_filtered.y();
        ft_sensor_states[i].force.z = states.force_filtered.z();
        ft_sensor_states[i].torque.x = states.torque_filtered.x();
        ft_sensor_states[i].torque.y = states.torque_filtered.y();
        ft_sensor_states[i].torque.z = states.torque_filtered.z();
    }
}


////////////////////////////////////////////////////////////////////////

void Hand::print_actuator_info_() {  
    std::cout << "================== Actuators Info ===================" << std::endl;
    std::cout << "[INFO] Total Actuators: " << actuators_.size() << std::endl;

    for (size_t i = 0; i < num_actuators_; ++i) {
        std::cout << "[INFO] Actuator " << (i + 1) 
                  << " - TX: 0x" << std::hex << actuators_[i].get_tx_id()
                  << ", RX: 0x" << actuators_[i].get_rx_id() << std::dec << std::endl;
    }
    
    std::cout << "=================== FT Sensors Info ==================" << std::endl;
    std::cout << "[INFO] Total FT Sensors: " << ft_sensors_.size() << std::endl;

    for (size_t i = 0; i < ft_sensors_.size(); ++i) {
        std::cout << "[INFO] FT Sensor " << (i + 1)
                  << " - Force RX: 0x" << std::hex << ft_sensors_[i].get_force_rx_id()
                  << ", Torque RX: 0x" << ft_sensors_[i].get_torque_rx_id() << std::dec << std::endl;
    }
}

////////////////////////////////////////////////////////////////////////

} // namespace plato2_hand