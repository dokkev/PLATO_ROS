#ifndef PLATO_ROBOT_SYSTEM__TASK__GRASP_TASK_HPP_
#define PLATO_ROBOT_SYSTEM__TASK__GRASP_TASK_HPP_

#include <Eigen/Core>

#include <array>

#include <pinocchio/multibody/model.hpp>

#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/sensor/tactile_grip_observation.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace plato_robot_system::task
{

enum class GraspTaskMode
{
  kMotionTeleop,
  kForceTracking,
};

struct GraspTaskConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class ForceAggregation
  {
    kMin,
    kAverage,
  };

  // Full-model reference posture used as the base command. The active thumb
  // and index joints are overwritten by the normalized grasp command; other
  // joints remain at this reference.
  Eigen::VectorXd q_ready;

  int force_enter_debounce_ticks{3};
  int force_exit_contact_lost_ticks{3};
  double force_exit_u_threshold{0.75};

  double min_contact_force_n{0.05};
  bool use_tactile_presence_for_contact{true};
  bool force_feedback_enabled{true};
  double lpf_alpha{1.0};
  bool debug_print_contact_states{false};
  double debug_print_contact_interval_s{0.5};

  // Current proportional/derivative force feedback directly offsets the
  // normalized grasp command. This will be replaced by admittance state next.
  double kp_tactile_u_fb{0.02};
  double kd_tactile_u_fb{0.0};
  double kp_tactile_phi_fb{0.0};
  double kd_tactile_phi_fb{0.0};
  ForceAggregation force_aggregation{ForceAggregation::kMin};

  // Parameters copied from the working parallel_grasp_controller geometry.
  // The normalized aperture command is u_open:
  //   u = 0 -> closed
  //   u = 0.5 -> ready/parallel reference
  //   u = 1 -> open
  double parallel_tip_radius_m{0.06};
  double parallel_lateral_offset_m{0.022};
  double parallel_qmin_rad{-0.7853981633974483};
  double parallel_qmax_rad{0.0};
  double parallel_q5_min_rad{1.0e-3};
  double parallel_midpoint_u{0.5};
  double parallel_max_flexion_rad{0.785};
};

struct GraspTaskCommand
{
  double u{0.0};
  double phi{0.0};
  double desired_force_n{1.0};
};

struct GraspTaskStatus
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspTaskMode mode{GraspTaskMode::kMotionTeleop};
  double u{0.0};
  double phi{0.0};
  double effective_u{0.0};
  double effective_phi{0.0};
  double u_parallel{0.5};
  double desired_force_n{1.0};
  double measured_force_n{0.0};
  double force_a_n{0.0};
  double force_b_n{0.0};
  bool contact_a{false};
  bool contact_b{false};
  bool enough_contact_a{false};
  bool enough_contact_b{false};
  bool lost_contact_a{true};
  bool lost_contact_b{true};
  bool valid_force{false};
  int contact_count{0};
  int enough_contact_count{0};
  int force_enter_counter{0};
  int force_exit_contact_lost_counter{0};
  double force_error_n{0.0};
  Eigen::VectorXd q_target;
};

class GraspTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool Configure(
    const pinocchio::Model & model,
    const GraspTaskConfig & config);

  void Reset();

  // Assumes RobotSystem has already accepted `state` and updated kinematics.
  bool OnEnter(RobotSystem & robot, const RobotState & state);

  bool PopulateCommand(
    RobotSystem & robot,
    const RobotState & state,
    const GraspTaskCommand & input,
    double dt_sec,
    RobotCommand * command);

  bool BuildMotionTarget(
    RobotSystem & robot,
    const RobotState & state,
    const GraspTaskCommand & input,
    double dt_sec,
    Eigen::VectorXd * q_target,
    GraspTaskStatus * status = nullptr);

  bool configured() const { return configured_; }
  int active_dof() const { return active_dof_; }
  GraspTaskMode mode() const { return mode_; }
  const GraspTaskStatus & status() const { return status_; }

private:
  bool HasCompatibleState(const RobotSystem & robot, const RobotState & state) const;
  bool CaptureReferencePosture(const RobotState & state);
  sensor::TactileGripObservation EstimateGripForceFromTactile(
    const RobotState & state) const;
  void PrintContactStatesIfNeeded(
    const RobotState & state,
    const sensor::TactileGripObservation & estimate);
  void UpdateMode(
    const sensor::TactileGripObservation & estimate,
    double u);
  double ParallelQ5Geometry(double q3) const;
  bool BuildParallelJointPositionTarget(
    const RobotSystem & robot,
    const RobotState & state,
    const GraspTaskCommand & input,
    const sensor::TactileGripObservation & estimate,
    double dt_sec,
    Eigen::VectorXd * q_target);

  GraspTaskConfig config_;
  bool configured_{false};
  int active_dof_{0};

  std::array<int, kThumbIndexActiveJoints.size()> active_q_indices_{};
  Eigen::VectorXd q_reference_;
  Eigen::VectorXd q_target_lpf_;
  bool has_reference_posture_{false};
  bool has_q_target_lpf_{false};

  GraspTaskMode mode_{GraspTaskMode::kMotionTeleop};
  int force_enter_counter_{0};
  int force_exit_contact_lost_counter_{0};
  double last_force_error_n_{0.0};
  bool has_last_force_error_{false};
  double last_contact_debug_print_time_s_{-1.0e100};

  GraspTaskStatus status_;
};

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__TASK__GRASP_TASK_HPP_
