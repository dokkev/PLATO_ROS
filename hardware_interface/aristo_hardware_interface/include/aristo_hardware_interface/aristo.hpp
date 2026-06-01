#ifndef ARISTO_HARDWARE_INTERFACE__ARISTO_HPP_
#define ARISTO_HARDWARE_INTERFACE__ARISTO_HPP_

#include <memory>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "aristo_hardware_interface/aristo_hand.hpp"
#include "aristo_hardware_interface/utils/visibility_control.h"

namespace aristo_hardware_interface
{

class AristoHardware : public hardware_interface::SystemInterface
{
public:
  ARISTO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ARISTO_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::unique_ptr<aristo_hand::Hand> hand_;
  bool zeroing_requested_ = false;
};

}  // namespace aristo_hardware_interface

#endif  // ARISTO_HARDWARE_INTERFACE__ARISTO_HPP_
