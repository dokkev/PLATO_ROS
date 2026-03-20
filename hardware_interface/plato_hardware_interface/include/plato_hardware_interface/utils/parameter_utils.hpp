#ifndef PLATO_HARDWARE_INTERFACE__PARAMETER_UTILS_HPP_
#define PLATO_HARDWARE_INTERFACE__PARAMETER_UTILS_HPP_

#include <stdexcept>
#include <string>

namespace plato_hardware_interface::utils
{

inline bool parse_bool_parameter(const std::string & value, const char * parameter_name)
{
  if (value == "true" || value == "True" || value == "1") {
    return true;
  }

  if (value == "false" || value == "False" || value == "0") {
    return false;
  }

  throw std::invalid_argument(
          std::string("Hardware parameter '") + parameter_name +
          "' must be one of: true, false, 1, 0");
}

}  // namespace plato_hardware_interface::utils

#endif  // PLATO_HARDWARE_INTERFACE__PARAMETER_UTILS_HPP_
