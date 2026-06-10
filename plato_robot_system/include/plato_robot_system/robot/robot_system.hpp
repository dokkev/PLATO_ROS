// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cmath>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "plato_robot_system/sensor/tactile_state.hpp"

namespace plato_robot_system {

using TactileSensorVector =
    std::vector<sensor::TactileState, Eigen::aligned_allocator<sensor::TactileState>>;

// Robot state container.
//
// RobotState is module-agnostic.
// Units inside plato_robot_system are SI: q is rad or m, qdot is rad/s or m/s,
// and tau is Nm or N.
//
// For real robot feedback:
//   q/qdot/tau are the accepted current values for this control tick.
//   tau may include measured motor torque, embedded feedback effects, contact
//   reaction, filtering, and unmodeled dynamics.
//
// For MPPI rollout:
//   q/qdot are predicted ideal states.
//   tau is the model torque computed by RNEA(q, qdot, qddot_sol).
//
// RobotState::tau is not necessarily tau_cmd, tau_ff_cmd, tau_fb_cmd,
// tau_meas, or tau_applied. Those names belong to command, measurement, or
// applied-torque layers.
struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};

  Eigen::VectorXd q;
  Eigen::VectorXd qdot;
  Eigen::VectorXd tau;
  TactileSensorVector tactile_sensors;
  double time_s{0.0};
};

inline RobotState MakeRobotState(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot,
                                 const Eigen::VectorXd& tau,
                                 const TactileSensorVector& tactile_sensors,
                                 double time_s = 0.0) {
  RobotState robot;
  robot.q = q;
  robot.qdot = qdot;
  robot.tau = tau;
  robot.tactile_sensors = tactile_sensors;
  robot.time_s = time_s;
  robot.valid = q.size() > 0 && qdot.size() > 0 && qdot.size() == tau.size() &&
                q.allFinite() && qdot.allFinite() && tau.allFinite() &&
                std::isfinite(time_s);
  return robot;
}

inline RobotState MakeRobotState(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot,
                                 const Eigen::VectorXd& tau,
                                 double time_s = 0.0) {
  return MakeRobotState(q, qdot, tau, TactileSensorVector{}, time_s);
}

inline bool IsValid(const RobotState& robot) {
  return robot.valid && robot.q.size() > 0 && robot.qdot.size() > 0 &&
         robot.qdot.size() == robot.tau.size() && robot.q.allFinite() &&
         robot.qdot.allFinite() && robot.tau.allFinite() &&
         std::isfinite(robot.time_s);
}

// Final robot/low-level-controller command packet. This is not the MPPI action
// and does not carry the solver acceleration. The host MPPI solves qddot_sol,
// integrates it to q_cmd/qdot_cmd, computes tau_ff_cmd with Pinocchio RNEA,
// and stores the final host-to-driver torque as tau_cmd.
//
// Ownership convention:
//   q_cmd, qdot_cmd, tau_cmd: populated by task/state/planner code.
//   kp, kd: populated only by ControlArchitecture::FinalizeCommand().
//           These are driver-local impedance gains, not host task gains.
//
// Units inside plato_robot_system are SI:
//   q_cmd: rad or m
//   qdot_cmd: rad/s or m/s
//   tau_cmd: Nm or N
//   kp: Nm/rad or N/m
//   kd: Nm/(rad/s) or N/(m/s)
//
// The embedded driver may internally apply:
//   tau_driver = tau_cmd + kp * (q_cmd - q) + kd * (qdot_cmd - qdot).
struct RobotCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};

  // Populated by task/state/planner code.
  Eigen::VectorXd q_cmd;
  Eigen::VectorXd qdot_cmd;
  Eigen::VectorXd tau_cmd;

  // Populated only by ControlArchitecture::FinalizeCommand().
  // Driver-local impedance gains.
  Eigen::VectorXd kp;
  Eigen::VectorXd kd;

  double stamp_sec{0.0};

  void Resize(int nq, int nv = -1) {
    if (nq < 0) {
      throw std::invalid_argument(
          "RobotCommand::Resize: nq must be nonnegative");
    }
    if (nv < 0) {
      nv = nq;
    }
    if (nv < 0) {
      throw std::invalid_argument(
          "RobotCommand::Resize: nv must be nonnegative");
    }

    q_cmd = Eigen::VectorXd::Zero(nq);
    qdot_cmd = Eigen::VectorXd::Zero(nv);
    tau_cmd = Eigen::VectorXd::Zero(nv);
    kp = Eigen::VectorXd::Zero(nv);
    kd = Eigen::VectorXd::Zero(nv);
  }

  bool HasValidDimensions() const {
    const Eigen::Index nq = q_cmd.size();
    const Eigen::Index nv = qdot_cmd.size();
    if (nq <= 0 || nv <= 0) {
      return false;
    }
    if (tau_cmd.size() != nv || kp.size() != nv || kd.size() != nv) {
      return false;
    }

    return true;
  }

  bool AllFinite() const {
    return q_cmd.allFinite() && qdot_cmd.allFinite() && tau_cmd.allFinite() &&
           kp.allFinite() && kd.allFinite() && std::isfinite(stamp_sec);
  }

  bool IsUsable() const { return valid && HasValidDimensions() && AllFinite(); }
};

inline RobotCommand MakeZeroHoldRobotCommand(
    const Eigen::VectorXd& q_current, const Eigen::VectorXd& qdot_current) {
  RobotCommand command;
  command.Resize(static_cast<int>(q_current.size()),
                 static_cast<int>(qdot_current.size()));
  command.q_cmd = q_current;
  command.qdot_cmd.setZero();
  // Driver-local gains are attached during controller command finalization.
  command.valid = command.HasValidDimensions() && command.AllFinite() &&
                  qdot_current.allFinite();
  return command;
}

inline RobotCommand MakeInvalidRobotCommand(int nq, int nv = -1) {
  RobotCommand command;
  command.Resize(nq, nv);
  command.valid = false;
  return command;
}

// Pinocchio-backed robot model and state holder.
//
// RobotSystem owns the Pinocchio model/data pair loaded from URDF and stores
// measured robot feedback. It does not own tactile state or GraspState; those
// are composed at the MPPI/grasp layer.
class RobotSystem {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotSystem() = default;
  explicit RobotSystem(const pinocchio::Model& model);
  RobotSystem(const std::string& urdf_path, bool verbose = false);
  RobotSystem(const std::string& urdf_path,
              const pinocchio::JointModelVariant& root_joint,
              bool verbose = false);

  void LoadModel(const pinocchio::Model& model);
  void LoadUrdf(const std::string& urdf_path, bool verbose = false);
  void LoadUrdf(const std::string& urdf_path,
                const pinocchio::JointModelVariant& root_joint,
                bool verbose = false);

  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  bool hasModel() const { return has_model_; }
  bool hasState() const { return has_state_; }

  const pinocchio::Model& model() const { return model_; }
  pinocchio::Model& model() { return model_; }
  const pinocchio::Data& data() const { return data_; }
  pinocchio::Data& data() { return data_; }

  // Hot-path state update. Assumes a model was loaded during construction or
  // Load*(); keep model/configuration checks out of the per-loop path.
  void UpdateState(const Eigen::Ref<const Eigen::VectorXd>& q,
                   const Eigen::Ref<const Eigen::VectorXd>& qdot,
                   double time_s = 0.0);
  void UpdateState(const Eigen::Ref<const Eigen::VectorXd>& q,
                   const Eigen::Ref<const Eigen::VectorXd>& qdot,
                   const Eigen::Ref<const Eigen::VectorXd>& tau,
                   double time_s = 0.0);
  void UpdateState(const RobotState& state);
  void UpdateTactileSensors(const TactileSensorVector& tactile_sensors);

  const RobotState& state() const { return state_; }
  const Eigen::VectorXd& q() const { return state_.q; }
  const Eigen::VectorXd& qdot() const { return state_.qdot; }
  const TactileSensorVector& tactile_sensors() const {
    return state_.tactile_sensors;
  }

  void ComputeAllTerms();
  void UpdateKinematics();
  Eigen::VectorXd InverseDynamics(
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& qdot,
      const Eigen::Ref<const Eigen::VectorXd>& qddot);
  Eigen::VectorXd Gravity(const Eigen::Ref<const Eigen::VectorXd>& q);
  Eigen::MatrixXd MassMatrix(const Eigen::Ref<const Eigen::VectorXd>& q);
  Eigen::VectorXd NonlinearEffects(
      const Eigen::Ref<const Eigen::VectorXd>& q,
      const Eigen::Ref<const Eigen::VectorXd>& qdot);

  pinocchio::FrameIndex FrameId(const std::string& frame_name) const;
  pinocchio::SE3 FramePose(pinocchio::FrameIndex frame_id) const;
  pinocchio::SE3 FramePoseWorld(const std::string& frame_name) const;
  pinocchio::SE3 FramePoseWorld(pinocchio::FrameIndex frame_id) const;
  pinocchio::Motion FrameVelocity(
      pinocchio::FrameIndex frame_id,
      pinocchio::ReferenceFrame reference_frame = pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Matrix<double, 6, Eigen::Dynamic> FrameJacobian(
      pinocchio::FrameIndex frame_id,
      pinocchio::ReferenceFrame reference_frame);
  // Backward-compatible helper: returns a LOCAL_WORLD_ALIGNED frame Jacobian.
  Eigen::Matrix<double, 6, Eigen::Dynamic> FrameJacobianWorld(
      const std::string& frame_name);
  Eigen::Matrix<double, 6, Eigen::Dynamic> FrameJacobianWorld(
      pinocchio::FrameIndex frame_id);
  Eigen::Matrix<double, 3, Eigen::Dynamic> PointJacobianWorld(
      pinocchio::FrameIndex frame_id,
      const Eigen::Ref<const Eigen::Vector3d>& point_in_frame);

 private:
  void ResetStateToNeutral();
  void CheckHasModel(const char* caller) const;
  void CheckStateDimensions(const RobotState& state, const char* caller) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
  RobotState state_;
  bool has_model_{false};
  bool has_state_{false};
};

}  // namespace plato_robot_system
