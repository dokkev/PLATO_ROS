// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/robot/robot_system.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace mppi_core {

RobotSystem::RobotSystem(const pinocchio::Model& model) {
  LoadModel(model);
}

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

void RobotSystem::UpdateState(
    const Eigen::Ref<const Eigen::VectorXd>& q_des,
    const Eigen::Ref<const Eigen::VectorXd>& qdot_des, double time_s) {
  CheckHasModel("RobotSystem::UpdateState");
  const Eigen::VectorXd qddot_des = Eigen::VectorXd::Zero(model_.nv);
  const Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(model_.nv);
  UpdateState(q_des, qdot_des, qddot_des, tau_ff, time_s);
}

void RobotSystem::UpdateState(
    const Eigen::Ref<const Eigen::VectorXd>& q_des,
    const Eigen::Ref<const Eigen::VectorXd>& qdot_des,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_des,
    const Eigen::Ref<const Eigen::VectorXd>& tau_ff, double time_s) {
  CheckHasModel("RobotSystem::UpdateState");
  RobotState next = MakeRobotState(q_des, qdot_des, qddot_des, tau_ff, time_s);
  CheckStateDimensions(next, "RobotSystem::UpdateState");
  if (!IsValidRobotState(next)) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state contains non-finite values");
  }
  state_ = next;
  has_state_ = true;
}

void RobotSystem::UpdateState(const RobotState& state) {
  CheckHasModel("RobotSystem::UpdateState");
  CheckStateDimensions(state, "RobotSystem::UpdateState");
  if (!IsValidRobotState(state)) {
    throw std::invalid_argument(
        "RobotSystem::UpdateState: state contains non-finite values");
  }
  state_ = state;
  has_state_ = true;
}

void RobotSystem::ComputeAllTerms() {
  CheckHasModel("RobotSystem::ComputeAllTerms");
  if (!has_state_) {
    throw std::logic_error("RobotSystem::ComputeAllTerms: state is not set");
  }
  pinocchio::computeAllTerms(model_, data_, state_.q_des, state_.qdot_des);
}

void RobotSystem::ResetStateToNeutral() {
  state_ = MakeRobotState(pinocchio::neutral(model_),
                          Eigen::VectorXd::Zero(model_.nv),
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
  if (state.q_des.size() != model_.nq ||
      state.qdot_des.size() != model_.nv ||
      state.qddot_des.size() != model_.nv ||
      state.tau_ff.size() != model_.nv) {
    throw std::invalid_argument(std::string(caller) +
                                ": state dimension mismatch");
  }
}

}  // namespace mppi_core
