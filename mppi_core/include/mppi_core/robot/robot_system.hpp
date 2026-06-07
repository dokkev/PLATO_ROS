// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <string>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "mppi_core/robot/robot_state.hpp"

namespace mppi_core {

// Pinocchio-backed robot model and state holder.
//
// RobotSystem owns the Pinocchio model/data pair loaded from URDF and returns a
// RobotState snapshot for rollout code. It does not own tactile state or
// GraspState; those are composed at the MPPI/grasp layer.
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

  void UpdateState(const Eigen::Ref<const Eigen::VectorXd>& q_des,
                   const Eigen::Ref<const Eigen::VectorXd>& qdot_des,
                   double time_s = 0.0);
  void UpdateState(const Eigen::Ref<const Eigen::VectorXd>& q_des,
                   const Eigen::Ref<const Eigen::VectorXd>& qdot_des,
                   const Eigen::Ref<const Eigen::VectorXd>& qddot_des,
                   const Eigen::Ref<const Eigen::VectorXd>& tau_ff,
                   double time_s = 0.0);
  void UpdateState(const RobotState& state);

  const RobotState& state() const { return state_; }
  const Eigen::VectorXd& q() const { return state_.q_des; }
  const Eigen::VectorXd& qdot() const { return state_.qdot_des; }

  void ComputeAllTerms();

 private:
  void ResetStateToNeutral();
  void CheckHasModel(const char* caller) const;
  void CheckStateDimensions(const RobotState& state,
                            const char* caller) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
  RobotState state_;
  bool has_model_{false};
  bool has_state_{false};
};

}  // namespace mppi_core
