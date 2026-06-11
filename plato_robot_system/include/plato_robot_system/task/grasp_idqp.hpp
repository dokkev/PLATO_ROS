#ifndef PLATO_ROBOT_SYSTEM__TASK__GRASP_IDQP_HPP_
#define PLATO_ROBOT_SYSTEM__TASK__GRASP_IDQP_HPP_

#include <Eigen/Core>

#include <array>
#include <string>

#include <pinocchio/multibody/model.hpp>

#include "plato_robot_system/robot/robot_system.hpp"
#include "plato_robot_system/sensor/tactile_grip_observation.hpp"
#include "plato_robot_system/task/thumb_index_grasp_constants.hpp"

namespace plato_robot_system::task
{

enum class GraspIDQPMode
{
  kMotionTeleop,
  kForceTracking,
};

struct GraspIDQPConfig
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

  double kp_tactile_u_fb{0.02};
  double kd_tactile_u_fb{0.0};
  double kp_tactile_phi_fb{0.0};
  double kd_tactile_phi_fb{0.0};
  sensor::TactileForceAggregation force_aggregation{
    sensor::TactileForceAggregation::kMin};

  double parallel_tip_radius_m{0.06};
  double parallel_lateral_offset_m{0.022};
  double parallel_qmin_rad{-0.7853981633974483};
  double parallel_qmax_rad{0.0};
  double parallel_q5_min_rad{1.0e-3};
  double parallel_midpoint_u{0.5};
  double parallel_max_flexion_rad{0.785};

  bool use_pinocchio_parallel_solver{true};

  std::string index_contact_point_frame{
    kThumbIndexFrameA.data(), kThumbIndexFrameA.size()};
  std::string thumb_contact_point_frame{
    kThumbIndexFrameB.data(), kThumbIndexFrameB.size()};
  Eigen::Vector3d index_contact_normal_axis_frame{0.0, 0.0, -1.0};
  Eigen::Vector3d thumb_contact_normal_axis_frame{0.0, 0.0, -1.0};

  double parallel_solver_w_aperture{1000.0};
  double parallel_solver_w_parallel{10.0};
  double parallel_solver_w_moment{1000.0};
  double parallel_solver_w_posture{1.0};
  double parallel_solver_w_smooth{0.1};
  double parallel_solver_damping{1.0e-6};
  double parallel_solver_fd_eps_rad{1.0e-4};
  double parallel_solver_max_step_rad{0.05};
  int parallel_solver_max_iters{10};
};

struct GraspIDQPCommand
{
  double u{0.0};
  double phi{0.0};
  double desired_force_n{1.0};
};

struct GraspIDQPStatus
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GraspIDQPMode mode{GraspIDQPMode::kMotionTeleop};
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

  double aperture_des_m{0.0};
  double aperture_m{0.0};
  double parallel_axis_error{0.0};
  double moment_arm_m{0.0};
  double solver_cost{0.0};
  int solver_iters{0};
  bool used_pinocchio_parallel_solver{false};
};

class GraspIDQP
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool Configure(
    const pinocchio::Model & model,
    const GraspIDQPConfig & config);

  void Reset();

  // Assumes RobotSystem has already accepted `state` and updated kinematics.
  bool OnEnter(RobotSystem & robot, const RobotState & state);

  bool PopulateCommand(
    RobotSystem & robot,
    const RobotState & state,
    const GraspIDQPCommand & input,
    double dt_sec,
    RobotCommand * command);

  bool BuildMotionTarget(
    RobotSystem & robot,
    const RobotState & state,
    const GraspIDQPCommand & input,
    double dt_sec,
    Eigen::VectorXd * q_target,
    GraspIDQPStatus * status = nullptr);

  bool configured() const { return configured_; }
  int active_dof() const { return active_dof_; }
  GraspIDQPMode mode() const { return mode_; }
  const GraspIDQPStatus & status() const { return status_; }

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
  bool BuildTeleopSeedTarget(
    const RobotSystem & robot,
    const RobotState & state,
    const GraspIDQPCommand & input,
    const sensor::TactileGripObservation & estimate,
    double dt_sec,
    Eigen::VectorXd * q_seed);
  bool BuildParallelJointPositionTarget(
    const RobotSystem & robot,
    const RobotState & state,
    const GraspIDQPCommand & input,
    const sensor::TactileGripObservation & estimate,
    double dt_sec,
    Eigen::VectorXd * q_target);

  GraspIDQPConfig config_;
  bool configured_{false};
  int active_dof_{0};

  std::array<int, kThumbIndexActiveJoints.size()> active_q_indices_{};
  pinocchio::FrameIndex index_contact_point_frame_id_{0};
  pinocchio::FrameIndex thumb_contact_point_frame_id_{0};
  bool has_pinocchio_parallel_frames_{false};

  Eigen::VectorXd q_reference_;
  Eigen::VectorXd q_target_lpf_;
  bool has_reference_posture_{false};
  bool has_q_target_lpf_{false};

  GraspIDQPMode mode_{GraspIDQPMode::kMotionTeleop};
  int force_enter_counter_{0};
  int force_exit_contact_lost_counter_{0};
  double last_force_error_n_{0.0};
  bool has_last_force_error_{false};
  double last_contact_debug_print_time_s_{-1.0e100};

  GraspIDQPStatus status_;
};

}  // namespace plato_robot_system::task

#endif  // PLATO_ROBOT_SYSTEM__TASK__GRASP_IDQP_HPP_
