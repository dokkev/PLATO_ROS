#ifndef ARISTO_CONTROLLER__STATE_MACHINES__MPPI_GRASP_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__MPPI_GRASP_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "mppi_core/config/rollout_config.hpp"
#include "mppi_core/contact/contact_kinematics.hpp"
#include "mppi_core/core/mppi_config.hpp"
#include "mppi_core/core/mppi_optimizer.hpp"
#include "mppi_core/logging/mppi_rollout_logger.hpp"
#include "mppi_core/rollout/grasp_state_rollout_model.hpp"
#include "mppi_core/task/task_config.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/robot/robot_system.hpp"

namespace aristo_controller::state_machines
{

struct MPPIGraspSafetyConfig
{
  double max_reference_tracking_error_rad{0.25};
  double max_qdot_cmd_rad_s{0.5};
  double max_tau_cmd_nm{0.05};
  double max_tau_rate_nm_s{1.0};
  bool clamp_q_cmd_to_model_limits{true};
};

struct MPPIGraspTactileConfig
{
  double thumb_normal_axis_sign{1.0};
  double index_normal_axis_sign{1.0};
};

struct MPPIGraspStateConfig
{
  mppi_core::MPPIConfig mppi;
  mppi_core::GraspStateRolloutConfig rollout;
  mppi_core::TactileTransitionConfig tactile_transition;
  mppi_core::TaskConfig task;
  MPPIGraspSafetyConfig safety;
  MPPIGraspTactileConfig tactile;
  mppi_core::logging::MppiRolloutLoggerConfig logging;
  bool exit_on_contact_lost{true};
};

class MPPIGraspState final : public plato_robot_system::State
{
public:
  static constexpr const char * kName = "mppi_grasp";

  MPPIGraspState(
    plato_robot_system::StateId id,
    plato_robot_system::RobotSystem * robot);

  bool ConfigureTask(const MPPIGraspStateConfig & config);
  void OnEnter() override;
  bool IsFinished() const override;
  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;
  bool exit_requested() const { return exit_requested_; }

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
  void RequestExitOnContactLoss(
    uint64_t tick_index,
    double time_s,
    const mppi_core::GraspState & state) const;
  mppi_core::logging::MppiTickLogRecord BuildTickLogRecord(
    uint64_t tick_index,
    const mppi_core::GraspObservation * observation,
    const plato_robot_system::RobotCommand & command,
    const Eigen::VectorXd & selected_action,
    double nominal_total_cost,
    bool used_fallback,
    const std::string & phase) const;
  void LogTickAndEvent(
    uint64_t tick_index,
    double time_s,
    const mppi_core::GraspObservation * observation,
    const plato_robot_system::RobotCommand & command,
    bool used_fallback,
    const std::string & event,
    const std::string & detail) const;
  bool ShouldLogRollout(uint64_t tick_index) const;

  plato_robot_system::RobotSystem * robot_{nullptr};
  MPPIGraspStateConfig config_;
  mppi_core::TactileTransitionConfig tactile_transition_;
  mutable mppi_core::MPPIOptimizer optimizer_;
  bool configured_{false};

  std::array<mppi_core::PinocchioContactKinematicsContext, 2> contact_kinematics_{};
  mutable plato_robot_system::RobotCommand last_command_;
  mutable mppi_core::logging::MppiRolloutLogger logger_;
  mutable uint64_t tick_index_{0};
  mutable std::string last_phase_{"mppi_grasp"};
  mutable bool has_entered_mppi_ready_{false};
  mutable bool exit_requested_{false};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__MPPI_GRASP_HPP_
