#ifndef ARISTO_HARDWARE_INTERFACE__UTILS__ACTUATOR_CONFIG_LOADER_HPP_
#define ARISTO_HARDWARE_INTERFACE__UTILS__ACTUATOR_CONFIG_LOADER_HPP_

#include <array>
#include <string>
#include <vector>

#include "aristo_hardware_interface/actuator.hpp"

namespace aristo_actuator
{

const std::array<const char *, 8> & expected_aristo_actuator_names();

std::vector<Config> load_aristo_actuator_configs();
std::vector<Config> load_aristo_actuator_configs(const std::string & yaml_path);

}  // namespace aristo_actuator

#endif  // ARISTO_HARDWARE_INTERFACE__UTILS__ACTUATOR_CONFIG_LOADER_HPP_
