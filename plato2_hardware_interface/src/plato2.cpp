#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "plato2_hardware_interface/plato2.hpp"

namespace plato2_hardware_interface {

hardware_interface::CallbackReturn
PLATO2Hardware::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize hand with pcan_interface_
  hand_ = std::make_unique<plato2_hand::Hand>(pcan_interface_);

  // Initialize all Joint Vectors
  joint_position_commands_.resize(info_.joints.size(),
                                std::numeric_limits<double>::quiet_NaN());

  joint_effort_commands_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());

  joint_position_states_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());

  joint_velocity_states_.resize(info_.joints.size(),
                              std::numeric_limits<double>::quiet_NaN());

  joint_effort_states_.resize(info_.joints.size(),
                            std::numeric_limits<double>::quiet_NaN());
  
  actuator_temperature_states_.resize(info_.joints.size(),
                            std::numeric_limits<double>::quiet_NaN());

  // Initialize FT sensor states
  // ft_sensor_states_.resize(6, std::numeric_limits<double>::quiet_NaN());

  // ft_sensor_states_.resize(6, std::numeric_limits<double>::quiet_NaN());

  // ft_sensor_ = std::make_unique<FTSensorCAN>("can0");

  // ft_sensor_->set_callback([this](double fx, double fy, double fz, double tx, double ty, double tz) {
  //     ft_sensor_states_[0] = fx;
  //     ft_sensor_states_[1] = fy;
  //     ft_sensor_states_[2] = fz;
  //     ft_sensor_states_[3] = tx;
  //     ft_sensor_states_[4] = ty;
  //     ft_sensor_states_[5] = tz;
  //   });

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (!(joint.command_interfaces[0].name == "effort")) {
      RCLCPP_FATAL(rclcpp::get_logger("PLATO2Hardware"),
                  "[ERROR] PLATO Hand V2 hardware interface only supports effort interface");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }



  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PLATO2Hardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"),
              "Configuring ...setting all joint state to 0..");

  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PLATO2Hardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Reserve space for joint and FT sensor interfaces
  state_interfaces.reserve(info_.joints.size() * 4 + 6);

  // Joint state interfaces
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &joint_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &joint_velocity_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &joint_effort_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, "temperature",
        &actuator_temperature_states_[i])); 
  }

  // FT sensor state interfaces
    // state_interfaces.emplace_back(hardware_interface::StateInterface("ft_sensor", "force.x", &ft_sensor_states_[0]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface("ft_sensor", "force.y", &ft_sensor_states_[1]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface("ft_sensor", "force.z", &ft_sensor_states_[2]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface("ft_sensor", "torque.x", &ft_sensor_states_[3]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface("ft_sensor", "torque.y", &ft_sensor_states_[4]));
    // state_interfaces.emplace_back(hardware_interface::StateInterface("ft_sensor", "torque.z", &ft_sensor_states_[5]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PLATO2Hardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  effort_command_interface_names_.reserve(info_.joints.size());

  // Position Command Interface
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &joint_effort_commands_[i]));
    effort_command_interface_names_.push_back(
        command_interfaces.back().get_name());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
PLATO2Hardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"),
              "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully activated!");

  //set initial joint effort command to 0
  for (size_t i = 0; i < joint_effort_commands_.size(); ++i) {
    joint_effort_commands_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
PLATO2Hardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("PLATO2Hardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
PLATO2Hardware::read(const rclcpp::Time &time,
                    const rclcpp::Duration &period) {
  // Update hand state
  hand_->set_impedance_command(joint_effort_commands_, 100, 
                             joint_position_states_, joint_velocity_states_);


  // hand_->set_idle_command();
  // hand_->print_motor_positions();

  hand_->update_states(joint_position_states_, joint_velocity_states_, 
                      joint_effort_states_);

  // Read FT sensor data


  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
PLATO2Hardware::write(const rclcpp::Time &time,
                     const rclcpp::Duration & /*period*/) {
  return hardware_interface::return_type::OK;
}

} // namespace plato2_hardware_interface

PLUGINLIB_EXPORT_CLASS(plato2_hardware_interface::PLATO2Hardware,
                      hardware_interface::SystemInterface)