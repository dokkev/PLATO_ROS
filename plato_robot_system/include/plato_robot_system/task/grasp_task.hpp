#ifndef PLATO_ROBOT_SYSTEM__TASK__GRASP_TASK_HPP_
#define PLATO_ROBOT_SYSTEM__TASK__GRASP_TASK_HPP_

#include <Eigen/Core>

#include <array>
#include <memory>

#include <pinocchio/multibody/model.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

#include "plato_robot_system/robot/robot_system.hpp"
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

  double distance_closed_m{0.015};
  double distance_open_m{0.085};

  // Defaults below this line are intentionally kept in C++ so the Aristo YAML
  // stays focused on the few values commonly tuned during experiments.
  Eigen::Vector3d fallback_close_axis_base{0.0, 0.0, -1.0};

  int force_enter_debounce_ticks{3};
  int force_exit_contact_lost_ticks{3};
  double force_exit_u_threshold{0.75};

  double min_contact_force_n{0.05};
  bool use_tactile_presence_for_contact{true};

  // Host-side close-axis task feedback gains. These are not RobotCommand.kp/kd
  // driver-local impedance gains.
  double kp_task{20.0};
  double kd_task{2.0};

  // Host-side in-plane contact-point alignment task. u_lateral=0.5 requests zero
  // lateral offset; u_lateral=0/1 request +/- lateral_offset_limit_m.
  double lateral_offset_limit_m{0.02};
  double kp_lateral{20.0};
  double kd_lateral{2.0};

  Eigen::VectorXd q_posture;

  // Host-side tactile feedback gains. Force error is mapped to desired
  // close-axis acceleration; these do not tune embedded driver impedance.
  double kp_tactile_fb{0.02};
  double kd_tactile_fb{0.0};
  ForceAggregation force_aggregation{ForceAggregation::kMin};

  double w_task_motion{5.0};
  double w_task_tactile_mode{0.5};
  double w_lateral{5.0};
  double w_tactile{10.0};
  double w_posture{0.05};
  double damping_qp{1.0e-5};

  double max_qddot_rad_s2{20.0};
  double max_velocity_rad_s{1.0};
  double max_torque_nm{0.2};
  double max_torque_rate_nm_per_s{2.0};

  double min_axis_distance_m{1.0e-4};
  bool use_inverse_dynamics{true};
  double joint_damping_nm_per_rad_s{0.0};
};

struct GraspTaskCommand
{
  double u_close{1.0};
  double u_lateral{0.5};
  double desired_force_n{1.0};
};

struct GraspTaskGripForceEstimate
{
  double measured_force_n{0.0};
  double force_a_n{0.0};
  double force_b_n{0.0};
  bool enough_contact_a{false};
  bool enough_contact_b{false};
  bool lost_contact_a{true};
  bool lost_contact_b{true};
  bool valid_force{false};
};

struct GraspTaskStatus
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspTaskMode mode{GraspTaskMode::kMotionTeleop};
  double u_close{1.0};
  double u_lateral{0.5};
  double desired_force_n{1.0};
  double measured_force_n{0.0};
  int force_enter_counter{0};
  int force_exit_contact_lost_counter{0};
  double close_error_m{0.0};
  double lateral_error_m{0.0};
  double force_error_n{0.0};
  bool qp_solved{false};
  Eigen::VectorXd qddot_active;
  Eigen::VectorXd tau_cmd_active;
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

  bool configured() const { return configured_; }
  int active_dof() const { return active_dof_; }
  GraspTaskMode mode() const { return mode_; }
  const GraspTaskStatus & status() const { return status_; }

private:
  bool HasCompatibleState(const RobotSystem & robot, const RobotState & state) const;
  bool CaptureEntryGeometry(RobotSystem & robot);
  GraspTaskGripForceEstimate EstimateGripForceFromTactile(
    const RobotState & state) const;
  void UpdateMode(
    const GraspTaskGripForceEstimate & estimate,
    double u);
  bool SolveActiveAcceleration(
    RobotSystem & robot,
    const RobotState & state,
    const GraspTaskCommand & input,
    const GraspTaskGripForceEstimate & estimate,
    double dt_sec,
    Eigen::VectorXd * qddot_active);
  Eigen::MatrixXd RestrictToActiveVelocityColumns(const Eigen::MatrixXd & jacobian) const;
  bool BuildAccelerationBounds(
    const pinocchio::Model & model,
    const RobotState & state,
    double dt_sec,
    Eigen::VectorXd * lower,
    Eigen::VectorXd * upper) const;
  bool BuildCommand(
    RobotSystem & robot,
    const RobotState & state,
    const Eigen::VectorXd & qddot_active,
    double dt_sec,
    RobotCommand * command);
  void ClampAndRateLimitTorque(double dt_sec, RobotCommand * command);

  GraspTaskConfig config_;
  bool configured_{false};
  int active_dof_{0};

  pinocchio::FrameIndex frame_a_id_{0};
  pinocchio::FrameIndex frame_b_id_{0};
  std::array<int, kThumbIndexActiveJoints.size()> active_q_indices_{};
  std::array<int, kThumbIndexActiveJoints.size()> active_v_indices_{};

  Eigen::Vector3d close_axis_base_{0.0, 0.0, -1.0};
  Eigen::Vector3d lateral_axis_base_{1.0, 0.0, 0.0};
  bool has_entry_geometry_{false};

  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  bool qp_initialized_{false};

  GraspTaskMode mode_{GraspTaskMode::kMotionTeleop};
  int force_enter_counter_{0};
  int force_exit_contact_lost_counter_{0};
  double last_force_error_n_{0.0};
  bool has_last_force_error_{false};

  Eigen::VectorXd last_tau_cmd_;
  bool has_last_tau_cmd_{false};
  GraspTaskStatus status_;
};

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__TASK__GRASP_TASK_HPP_
