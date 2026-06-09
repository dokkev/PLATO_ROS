#ifndef ARISTO_STATE_MACHINES__CONFIG__ARISTO_CONFIG_HPP_
#define ARISTO_STATE_MACHINES__CONFIG__ARISTO_CONFIG_HPP_

#include <Eigen/Core>

#include <string>

#include "plato_robot_system/control/state_machine/state_machine.hpp"

namespace aristo_state_machines::config
{

struct InitializeConfig
{
  plato_robot_system::StateId id{0};
  Eigen::VectorXd target_jpos;
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;
};

struct AristoConfig
{
  int num_joints{8};
  bool fixed_thumb{false};
  bool debug_enabled{false};
  InitializeConfig initialize;
};

std::string default_aristo_config_path();
AristoConfig load_aristo_config(const std::string & yaml_path);

}  // namespace aristo_state_machines::config

#endif  // ARISTO_STATE_MACHINES__CONFIG__ARISTO_CONFIG_HPP_
