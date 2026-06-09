#ifndef ARISTO_CONTROLLER__CONFIG__ARISTO_CONFIG_HPP_
#define ARISTO_CONTROLLER__CONFIG__ARISTO_CONFIG_HPP_

#include <Eigen/Core>

#include <string>

#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/control/plato_control_architecture.hpp"

namespace aristo_controller::config
{

struct InitializeConfig
{
  plato_robot_system::StateId id{0};
  double duration_sec{2.0};
  Eigen::VectorXd target_jpos;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};

struct AristoConfig
{
  int num_joints{8};
  bool fixed_thumb{false};
  bool debug_enabled{false};
  plato_robot_system::DriverPdGainsConfig driver_gains;
  InitializeConfig initialize;
};

std::string default_aristo_config_path();
AristoConfig load_aristo_config(const std::string & yaml_path);

}  // namespace aristo_controller::config

#endif  // ARISTO_CONTROLLER__CONFIG__ARISTO_CONFIG_HPP_
