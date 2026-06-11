#ifndef ARISTO_CONTROLLER__STATE_MACHINES__ROBUST_GRASP_MPC_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__ROBUST_GRASP_MPC_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/policy/robust_grasp_policy.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/robot/robot_system.hpp"

namespace aristo_controller::state_machines
{

struct RobustGraspMpcDebugConfig
{
  bool print_status{true};
  double print_status_interval_s{0.5};
};

struct RobustGraspMpcSafetyConfig
{
  double max_reference_tracking_error_rad{0.25};
  double max_qdot_cmd_rad_s{0.5};
  double max_tau_cmd_nm{0.05};
  double max_tau_rate_nm_s{1.0};
  bool clamp_q_cmd_to_model_limits{true};
};

struct RobustGraspMpcTactileConfig
{
  double thumb_normal_axis_sign{1.0};
  double index_normal_axis_sign{1.0};
};

struct RobustGraspMpcStateConfig
{
  mppi_core::RobustGraspPolicyConfig policy;
  RobustGraspMpcSafetyConfig safety;
  RobustGraspMpcTactileConfig tactile;
  RobustGraspMpcDebugConfig debug;
};

class RobustGraspMpcState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "robust_grasp_mpc";

  RobustGraspMpcState(
    plato_robot_system::StateId id,
    plato_robot_system::RobotSystem * robot);

  bool ConfigureTask(const RobustGraspMpcStateConfig & config);
  void OnEnter() override;
  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

private:
  using TactileStateVector =
    std::vector<mppi_core::TactileState, Eigen::aligned_allocator<mppi_core::TactileState>>;

  bool ConfigureContactKinematics();
  bool BuildObservation(mppi_core::GraspObservation * observation) const;
  bool BuildTactileContexts(
    const TactileStateVector & tactile_meas,
    std::vector<mppi_core::TactileSensorContext> * contexts) const;
  const mppi_core::PinocchioContactKinematicsContext * KinematicsForTactile(
    const mppi_core::TactileState & tactile,
    std::size_t tactile_index) const;
  bool CanUseLastCommandReference(const plato_robot_system::RobotState & state) const;
  bool ApplyCommandSafety(plato_robot_system::RobotCommand * command) const;
  bool PopulateHoldCommand(plato_robot_system::RobotCommand * command) const;
  void PrintStatus(
    double time_s,
    const mppi_core::GraspObservation & observation,
    const plato_robot_system::RobotCommand & command) const;
  bool ShouldPrintStatus(double time_s) const;

  plato_robot_system::RobotSystem * robot_{nullptr};
  RobustGraspMpcStateConfig config_;
  mutable mppi_core::RobustGraspPolicy policy_;
  bool configured_{false};
  std::array<mppi_core::PinocchioContactKinematicsContext, 2> contact_kinematics_{};
  mutable plato_robot_system::RobotCommand last_command_;
  mutable uint64_t tick_index_{0};
  mutable double last_status_print_time_s_{-1.0e100};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__ROBUST_GRASP_MPC_HPP_
