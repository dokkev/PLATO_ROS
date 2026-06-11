#ifndef ARISTO_CONTROLLER__STATE_MACHINES__MPPI_MOTION_GRASP_HPP_
#define ARISTO_CONTROLLER__STATE_MACHINES__MPPI_MOTION_GRASP_HPP_

#include <Eigen/Core>

#include <array>
#include <string>

#include "mppi_core/core/mppi_config.hpp"
#include "mppi_core/core/mppi_optimizer.hpp"
#include "mppi_core/costs/motion_tracking_cost.hpp"
#include "plato_robot_system/control/state_machine/state_machine.hpp"
#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/task/grasp_task.hpp"

namespace aristo_controller::state_machines
{

enum class GraspInitiationPhase
{
  kBothClosing,
  kThumbContactWaitIndex,
  kIndexContactWaitThumb,
  kBothContactAlignment,
  kForceRamp,
  kForceTracking,
};

const char * ToString(GraspInitiationPhase phase);

struct MPPIMotionGraspInput
{
  double u{1.0};
  double phi{0.5};
  double desired_force_n{1.0};
};

struct MPPIMotionGraspInitiationConfig
{
  int contact_enter_debounce_ticks{3};
  int contact_exit_debounce_ticks{3};
  double min_contact_force_n{0.05};
  bool use_tactile_presence_for_contact{true};

  double contacted_finger_hold_weight{50.0};
  double moving_finger_target_weight{10.0};
  double posture_weight{1.0};

  double max_reference_tracking_error_rad{0.5};
};

struct MPPIMotionGraspSafetyConfig
{
  double max_velocity_rad_s{1.0};
  double max_torque_nm{0.2};
  double max_torque_rate_nm_per_s{2.0};
};

struct MPPIMotionGraspStateConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool enabled{true};
  double default_u{1.0};
  double default_phi{0.5};
  double default_desired_force_n{1.0};

  MPPIMotionGraspInitiationConfig initiation;
  MPPIMotionGraspSafetyConfig safety;
  mppi_core::MPPIConfig mppi;
  mppi_core::MotionTrackingCostConfig cost;
  plato_robot_system::task::GraspTaskConfig grasp_task;
};

class MPPIMotionGraspState final : public plato_robot_system::State
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static constexpr const char * kName = "mppi_motion_grasp";

  MPPIMotionGraspState(
    plato_robot_system::StateId id,
    plato_robot_system::RobotSystem * robot);

  bool ConfigureTask(const MPPIMotionGraspStateConfig & config);
  void SetInput(const MPPIMotionGraspInput & input);

  void OnEnter() override;
  void OnExit() override;
  bool PopulateCommand(plato_robot_system::RobotCommand * command) const override;

  GraspInitiationPhase phase() const { return phase_; }
  const MPPIMotionGraspInput & input() const { return input_; }
  double default_desired_force_n() const { return config_.default_desired_force_n; }
  bool thumb_latch_active() const { return thumb_latch_active_; }
  bool index_latch_active() const { return index_latch_active_; }
  double reference_tracking_error_rad() const { return reference_tracking_error_rad_; }

private:
  bool ConfigureActiveJointIndices();
  bool BuildObservation(mppi_core::GraspObservation * observation) const;
  bool BuildPhaseMotionTarget(
    const plato_robot_system::RobotState & state,
    Eigen::VectorXd * q_target,
    Eigen::VectorXd * q_weight) const;
  bool UpdateContactDebounce(
    const plato_robot_system::RobotState & state,
    bool * thumb_contact,
    bool * index_contact) const;
  bool HasEnoughThumbContact(const plato_robot_system::RobotState & state) const;
  bool HasEnoughIndexContact(const plato_robot_system::RobotState & state) const;
  bool HasEnoughContactForFrame(
    const plato_robot_system::RobotState & state,
    const std::string & frame_name) const;
  void TransitionToPhase(
    GraspInitiationPhase phase,
    const plato_robot_system::RobotState & state,
    const Eigen::VectorXd & current_target) const;
  void LatchThumb(const plato_robot_system::RobotState & state) const;
  void LatchIndex(const plato_robot_system::RobotState & state) const;
  void LatchFullTarget(
    const plato_robot_system::RobotState & state,
    const Eigen::VectorXd & current_target) const;
  Eigen::VectorXd LatchReference(const plato_robot_system::RobotState & state) const;
  bool CanUseLastCommandReference(const plato_robot_system::RobotState & state) const;
  bool ApplyCommandSafety(plato_robot_system::RobotCommand * command) const;
  bool PopulateHoldCommand(plato_robot_system::RobotCommand * command) const;
  void ResetPhaseState() const;

  plato_robot_system::RobotSystem * robot_{nullptr};
  MPPIMotionGraspStateConfig config_;
  mutable plato_robot_system::task::GraspTask grasp_task_;
  mutable mppi_core::MPPIOptimizer optimizer_;
  bool configured_{false};

  std::array<int, plato_robot_system::task::kThumbIndexActiveJoints.size()> active_q_indices_{};
  std::array<int, plato_robot_system::task::kThumbIndexActiveJoints.size()> active_v_indices_{};

  MPPIMotionGraspInput input_;
  mutable GraspInitiationPhase phase_{GraspInitiationPhase::kBothClosing};
  mutable plato_robot_system::RobotCommand last_command_;
  mutable int thumb_contact_counter_{0};
  mutable int index_contact_counter_{0};
  mutable int thumb_lost_counter_{0};
  mutable int index_lost_counter_{0};
  mutable bool thumb_latch_active_{false};
  mutable bool index_latch_active_{false};
  mutable Eigen::Vector2d thumb_latched_q_{Eigen::Vector2d::Zero()};
  mutable Eigen::Vector2d index_latched_q_{Eigen::Vector2d::Zero()};
  mutable bool full_target_latch_active_{false};
  mutable Eigen::VectorXd full_latched_q_;
  mutable bool used_hold_fallback_{false};
  mutable double reference_tracking_error_rad_{0.0};
};

}  // namespace aristo_controller::state_machines

#endif  // ARISTO_CONTROLLER__STATE_MACHINES__MPPI_MOTION_GRASP_HPP_
