// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/robot/robot_system.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <stdexcept>

namespace plato_robot_system {

RobotSystem::RobotSystem(const pinocchio::Model& model) { LoadModel(model); }

RobotSystem::RobotSystem(const std::string& urdf_path, bool verbose) {
  LoadUrdf(urdf_path, verbose);
}

RobotSystem::RobotSystem(const std::string& urdf_path,
                         const pinocchio::JointModelVariant& root_joint,
                         bool verbose) {
  LoadUrdf(urdf_path, root_joint, verbose);
}

void RobotSystem::LoadModel(const pinocchio::Model& model) {
  model_ = model;
  data_ = pinocchio::Data(model_);
  has_model_ = true;
  ResetStateToNeutral();
}

void RobotSystem::LoadUrdf(const std::string& urdf_path, bool verbose) {
  pinocchio::urdf::buildModel(urdf_path, model_, verbose);
  data_ = pinocchio::Data(model_);
  has_model_ = true;
  ResetStateToNeutral();
}

void RobotSystem::LoadUrdf(const std::string& urdf_path,
                           const pinocchio::JointModelVariant& root_joint,
                           bool verbose) {
  pinocchio::urdf::buildModel(urdf_path, root_joint, model_, verbose);
  data_ = pinocchio::Data(model_);
  has_model_ = true;
  ResetStateToNeutral();
}

void RobotSystem::UpdateState(const Eigen::Ref<const Eigen::VectorXd>& q,
                              const Eigen::Ref<const Eigen::VectorXd>& qdot,
                              double time_s) {
  if (q.size() != model_.nq || qdot.size() != model_.nv) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state dimension mismatch");
  }
  if (!q.allFinite() || !qdot.allFinite() || !std::isfinite(time_s)) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state contains non-finite values");
  }
  state_.q = q;
  state_.qdot = qdot;
  state_.tau.setZero();
  state_.time_s = time_s;
  state_.valid = true;
  has_state_ = true;
}

void RobotSystem::UpdateState(const Eigen::Ref<const Eigen::VectorXd>& q,
                              const Eigen::Ref<const Eigen::VectorXd>& qdot,
                              const Eigen::Ref<const Eigen::VectorXd>& tau,
                              double time_s) {
  if (q.size() != model_.nq || qdot.size() != model_.nv ||
      tau.size() != model_.nv) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state dimension mismatch");
  }
  if (!q.allFinite() || !qdot.allFinite() || !tau.allFinite() ||
      !std::isfinite(time_s)) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state contains non-finite values");
  }
  state_.q = q;
  state_.qdot = qdot;
  state_.tau = tau;
  state_.time_s = time_s;
  state_.valid = true;
  has_state_ = true;
}

void RobotSystem::UpdateState(const RobotState& state) {
  CheckStateDimensions(state, "RobotSystem::UpdateState");
  if (!IsValid(state)) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state contains non-finite values");
  }
  state_ = state;
  has_state_ = true;
}

void RobotSystem::UpdateTactileSensors(
    const TactileSensorVector& tactile_sensors) {
  state_.tactile_sensors = tactile_sensors;
}

void RobotSystem::ComputeAllTerms() {
  CheckHasModel("RobotSystem::ComputeAllTerms");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::ComputeAllTerms: state is not set");
  }
  pinocchio::computeAllTerms(model_, data_, state_.q, state_.qdot);
}

void RobotSystem::UpdateKinematics() {
  CheckHasModel("RobotSystem::UpdateKinematics");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::UpdateKinematics: state is not set");
  }
  pinocchio::forwardKinematics(model_, data_, state_.q, state_.qdot);
  pinocchio::updateFramePlacements(model_, data_);
}

pinocchio::FrameIndex RobotSystem::FrameId(
    const std::string& frame_name) const {
  CheckHasModel("RobotSystem::FrameId");
  const auto frame_id = model_.getFrameId(frame_name);
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    throw std::invalid_argument("RobotSystem::FrameId: unknown frame '" +
                                frame_name + "'");
  }
  return frame_id;
}

pinocchio::SE3 RobotSystem::FramePoseWorld(
    const std::string& frame_name) const {
  return FramePoseWorld(FrameId(frame_name));
}

pinocchio::SE3 RobotSystem::FramePoseWorld(
    pinocchio::FrameIndex frame_id) const {
  CheckHasModel("RobotSystem::FramePoseWorld");
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    throw std::invalid_argument("RobotSystem::FramePoseWorld: invalid frame id");
  }
  return data_.oMf[frame_id];
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotSystem::FrameJacobianWorld(
    const std::string& frame_name) {
  return FrameJacobianWorld(FrameId(frame_name));
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotSystem::FrameJacobianWorld(
    pinocchio::FrameIndex frame_id) {
  CheckHasModel("RobotSystem::FrameJacobianWorld");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::FrameJacobianWorld: state is not set");
  }
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model_.nframes)) {
    throw std::invalid_argument(
        "RobotSystem::FrameJacobianWorld: invalid frame id");
  }

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model_.nv);
  jacobian.setZero();
  pinocchio::computeFrameJacobian(
      model_, data_, state_.q, frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
      jacobian);
  return jacobian;
}

void RobotSystem::ResetStateToNeutral() {
  state_ = MakeRobotState(pinocchio::neutral(model_),
                          Eigen::VectorXd::Zero(model_.nv),
                          Eigen::VectorXd::Zero(model_.nv), 0.0);
  has_state_ = false;
}

void RobotSystem::CheckHasModel(const char* caller) const {
  if (!has_model_) {
    throw std::logic_error(std::string(caller) + ": model is not loaded");
  }
}

void RobotSystem::CheckStateDimensions(const RobotState& state,
                                       const char* caller) const {
  if (!has_model_) {
    if (state.q.size() <= 0 || state.qdot.size() <= 0 ||
        state.tau.size() != state.qdot.size()) {
      throw std::invalid_argument(std::string(caller) +
                                  ": state dimension mismatch");
    }
    return;
  }

  if (state.q.size() != model_.nq || state.qdot.size() != model_.nv ||
      state.tau.size() != model_.nv) {
    throw std::invalid_argument(std::string(caller) +
                                ": state dimension mismatch");
  }
}

}  // namespace plato_robot_system
