#ifndef ARISTO_CONTROLLER__CONFIG__ARISTO_CONFIG_HPP_
#define ARISTO_CONTROLLER__CONFIG__ARISTO_CONFIG_HPP_

#include <Eigen/Core>

#include <string>

#include "aristo_controller/state_machines/grasp_force.hpp"
#include "aristo_controller/state_machines/grasp_teleop.hpp"
#include "aristo_controller/state_machines/joint_teleop.hpp"
#include "aristo_controller/state_machines/robust_grasp_mpc.hpp"
#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/control/plato_control_architecture.hpp"

namespace aristo_controller::config
{

struct StateConfig
{
  plato_robot_system::StateId id{0};
  plato_robot_system::StateLifecycle lifecycle;
};

struct GraspTeleopConfig : public StateConfig
{
  aristo_controller::state_machines::GraspTeleopStateConfig state;
};

struct GraspForceConfig : public StateConfig
{
  aristo_controller::state_machines::GraspForceStateConfig state;
};

struct JointTeleopConfig : public StateConfig
{
  aristo_controller::state_machines::JointTeleopStateConfig state;
};

struct JointPositionConfig : public StateConfig
{
  double duration_sec{2.0};
  Eigen::VectorXd target_jpos;
  Eigen::VectorXd kp_task;
  Eigen::VectorXd kd_task;
};

struct RobustGraspMpcConfig : public StateConfig
{
  aristo_controller::state_machines::RobustGraspMpcStateConfig state;
};

struct RobotModelConfig
{
  std::string urdf_path;
  bool is_floating_base{false};
  std::string base_frame;
  bool fixed_thumb{false};
};

struct AristoConfig
{
  int num_joints{8};
  bool debug_enabled{false};
  RobotModelConfig robot_model;
  plato_robot_system::DriverPdGainsConfig driver_gains;
  StateConfig idle;
  JointPositionConfig initialize;
  JointPositionConfig poke;
  JointPositionConfig grasp_ready;
  JointTeleopConfig joint_teleop;
  GraspTeleopConfig grasp_teleop;
  GraspForceConfig grasp_force;
  RobustGraspMpcConfig robust_grasp_mpc;
};

std::string default_aristo_config_path();
AristoConfig load_aristo_config(const std::string & yaml_path);

}  // namespace aristo_controller::config

#endif  // ARISTO_CONTROLLER__CONFIG__ARISTO_CONFIG_HPP_
