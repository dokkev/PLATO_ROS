// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "plato_robot_system/robot/robot_system.hpp"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <stdexcept>

namespace plato_robot_system {
namespace {

void CheckConfigVector(const pinocchio::Model& model,
                       const Eigen::Ref<const Eigen::VectorXd>& q,
                       const char* caller) {
  if (q.size() != model.nq) {
    throw std::invalid_argument(std::string(caller) +
                                ": configuration dimension mismatch");
  }
  if (!q.allFinite()) {
    throw std::invalid_argument(std::string(caller) +
                                ": configuration contains non-finite values");
  }
}

void CheckTangentVector(const pinocchio::Model& model,
                        const Eigen::Ref<const Eigen::VectorXd>& value,
                        const char* caller,
                        const char* name) {
  if (value.size() != model.nv) {
    throw std::invalid_argument(std::string(caller) + ": " + name +
                                " dimension mismatch");
  }
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(caller) + ": " + name +
                                " contains non-finite values");
  }
}

void CheckFrameIndex(const pinocchio::Model& model,
                     pinocchio::FrameIndex frame_id,
                     const char* caller) {
  if (frame_id >= static_cast<pinocchio::FrameIndex>(model.nframes)) {
    throw std::invalid_argument(std::string(caller) + ": invalid frame id");
  }
}

Eigen::Matrix3d Skew(const Eigen::Vector3d& value) {
  Eigen::Matrix3d skew;
  skew << 0.0, -value.z(), value.y(),
          value.z(), 0.0, -value.x(),
          -value.y(), value.x(), 0.0;
  return skew;
}

}  // namespace

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

Eigen::VectorXd RobotSystem::InverseDynamics(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& qdot,
    const Eigen::Ref<const Eigen::VectorXd>& qddot) {
  CheckHasModel("RobotSystem::InverseDynamics");
  CheckConfigVector(model_, q, "RobotSystem::InverseDynamics");
  CheckTangentVector(model_, qdot, "RobotSystem::InverseDynamics", "qdot");
  CheckTangentVector(model_, qddot, "RobotSystem::InverseDynamics", "qddot");
  return pinocchio::rnea(model_, data_, q, qdot, qddot);
}

Eigen::VectorXd RobotSystem::Gravity(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  CheckHasModel("RobotSystem::Gravity");
  CheckConfigVector(model_, q, "RobotSystem::Gravity");
  return pinocchio::computeGeneralizedGravity(model_, data_, q);
}

Eigen::MatrixXd RobotSystem::MassMatrix(
    const Eigen::Ref<const Eigen::VectorXd>& q) {
  CheckHasModel("RobotSystem::MassMatrix");
  CheckConfigVector(model_, q, "RobotSystem::MassMatrix");
  Eigen::MatrixXd mass = pinocchio::crba(model_, data_, q);
  for (Eigen::Index row = 0; row < mass.rows(); ++row) {
    for (Eigen::Index col = row + 1; col < mass.cols(); ++col) {
      mass(col, row) = mass(row, col);
    }
  }
  return mass;
}

Eigen::VectorXd RobotSystem::NonlinearEffects(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& qdot) {
  CheckHasModel("RobotSystem::NonlinearEffects");
  CheckConfigVector(model_, q, "RobotSystem::NonlinearEffects");
  CheckTangentVector(model_, qdot, "RobotSystem::NonlinearEffects", "qdot");
  return pinocchio::nonLinearEffects(model_, data_, q, qdot);
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

pinocchio::SE3 RobotSystem::FramePose(pinocchio::FrameIndex frame_id) const {
  return FramePoseWorld(frame_id);
}

pinocchio::SE3 RobotSystem::FramePoseWorld(
    const std::string& frame_name) const {
  return FramePoseWorld(FrameId(frame_name));
}

pinocchio::SE3 RobotSystem::FramePoseWorld(
    pinocchio::FrameIndex frame_id) const {
  CheckHasModel("RobotSystem::FramePoseWorld");
  CheckFrameIndex(model_, frame_id, "RobotSystem::FramePoseWorld");
  return data_.oMf[frame_id];
}

pinocchio::Motion RobotSystem::FrameVelocity(
    pinocchio::FrameIndex frame_id,
    pinocchio::ReferenceFrame reference_frame) {
  CheckHasModel("RobotSystem::FrameVelocity");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::FrameVelocity: state is not set");
  }
  CheckFrameIndex(model_, frame_id, "RobotSystem::FrameVelocity");
  pinocchio::forwardKinematics(model_, data_, state_.q, state_.qdot);
  pinocchio::updateFramePlacements(model_, data_);
  return pinocchio::getFrameVelocity(
      model_, data_, frame_id, reference_frame);
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotSystem::FrameJacobian(
    pinocchio::FrameIndex frame_id,
    pinocchio::ReferenceFrame reference_frame) {
  CheckHasModel("RobotSystem::FrameJacobian");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::FrameJacobian: state is not set");
  }
  CheckFrameIndex(model_, frame_id, "RobotSystem::FrameJacobian");

  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian(6, model_.nv);
  jacobian.setZero();
  pinocchio::computeFrameJacobian(
      model_, data_, state_.q, frame_id, reference_frame, jacobian);
  return jacobian;
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotSystem::FrameJacobianWorld(
    const std::string& frame_name) {
  return FrameJacobianWorld(FrameId(frame_name));
}

Eigen::Matrix<double, 6, Eigen::Dynamic> RobotSystem::FrameJacobianWorld(
    pinocchio::FrameIndex frame_id) {
  return FrameJacobian(frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
}

Eigen::Matrix<double, 3, Eigen::Dynamic> RobotSystem::PointJacobianWorld(
    pinocchio::FrameIndex frame_id,
    const Eigen::Ref<const Eigen::Vector3d>& point_in_frame) {
  CheckHasModel("RobotSystem::PointJacobianWorld");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::PointJacobianWorld: state is not set");
  }
  CheckFrameIndex(model_, frame_id, "RobotSystem::PointJacobianWorld");
  if (!point_in_frame.allFinite()) {
    throw std::invalid_argument(
        "RobotSystem::PointJacobianWorld: point contains non-finite values");
  }
  pinocchio::forwardKinematics(model_, data_, state_.q, state_.qdot);
  pinocchio::updateFramePlacements(model_, data_);
  const pinocchio::SE3 frame_pose = FramePoseWorld(frame_id);
  const Eigen::Vector3d point_from_origin_world =
      frame_pose.rotation() * point_in_frame;
  const auto frame_jacobian =
      FrameJacobian(frame_id, pinocchio::LOCAL_WORLD_ALIGNED);
  return frame_jacobian.topRows<3>() -
         Skew(point_from_origin_world) * frame_jacobian.bottomRows<3>();
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
