// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/robot/robot_command_builder.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <cmath>
#include <stdexcept>

namespace mppi_core {
namespace {

bool HasPinocchioModelData(const RobotSystem* robot_system) {
  return robot_system != nullptr && robot_system->hasModel() &&
         robot_system->data().oMi.size() == robot_system->model().joints.size();
}

bool HasPinocchioConfigurationSpace(const GraspObservation& observation,
                                    std::size_t action_dim) {
  const RobotSystem* robot_system = observation.robot_system;
  return HasPinocchioModelData(robot_system) &&
         observation.q_ref_current.size() ==
             static_cast<Eigen::Index>(robot_system->nq()) &&
         static_cast<Eigen::Index>(action_dim) ==
             static_cast<Eigen::Index>(robot_system->nv());
}

bool HasCompatibleReferenceConfiguration(const GraspObservation& observation,
                                         std::size_t action_dim) {
  return observation.q_ref_current.size() ==
             static_cast<Eigen::Index>(action_dim) ||
         HasPinocchioConfigurationSpace(observation, action_dim);
}

Eigen::VectorXd ReferenceVelocityOrZero(const GraspObservation& observation,
                                        std::size_t action_dim) {
  if (observation.qdot_ref_current.size() ==
      static_cast<Eigen::Index>(action_dim)) {
    if (!observation.qdot_ref_current.allFinite()) {
      throw std::invalid_argument(
          "MakeRobotCommandFromQddot: qdot_ref_current must be finite when provided");
    }
    return observation.qdot_ref_current;
  }
  return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(action_dim));
}

Eigen::VectorXd IntegrateReferenceStep(
    const GraspObservation& observation,
    const Eigen::Ref<const Eigen::VectorXd>& tangent_step) {
  if (HasPinocchioConfigurationSpace(
          observation, static_cast<std::size_t>(tangent_step.size()))) {
    const RobotSystem* robot_system = observation.robot_system;
    return pinocchio::integrate(robot_system->model(),
                                observation.q_ref_current, tangent_step);
  }
  if (observation.q_ref_current.size() == tangent_step.size()) {
    return observation.q_ref_current + tangent_step;
  }
  throw std::invalid_argument(
      "MakeRobotCommandFromQddot: q_ref_current dimension mismatch");
}

struct RobotStateView {
  const Eigen::VectorXd* q{nullptr};
  const Eigen::VectorXd* qdot{nullptr};
};

RobotStateView SelectCurrentStateForRnea(const GraspObservation& observation,
                                         std::size_t action_dim) {
  RobotSystem* robot_system = observation.robot_system;
  if (!HasPinocchioModelData(robot_system) ||
      static_cast<Eigen::Index>(action_dim) != robot_system->nv()) {
    return {};
  }

  if (robot_system->hasState()) {
    const RobotState& state = robot_system->state();
    if (IsValid(state) &&
        state.q.size() == static_cast<Eigen::Index>(robot_system->nq()) &&
        state.qdot.size() == static_cast<Eigen::Index>(robot_system->nv())) {
      return RobotStateView{&state.q, &state.qdot};
    }
  }

  if (observation.q_meas.size() ==
          static_cast<Eigen::Index>(robot_system->nq()) &&
      observation.qdot_meas.size() ==
          static_cast<Eigen::Index>(robot_system->nv()) &&
      observation.q_meas.allFinite() && observation.qdot_meas.allFinite()) {
    return RobotStateView{&observation.q_meas, &observation.qdot_meas};
  }

  return {};
}

Eigen::VectorXd CommandFeedForwardTorqueOrZero(
    const GraspObservation& observation, const Eigen::VectorXd& qddot_sol) {
  Eigen::VectorXd tau_ff_cmd = Eigen::VectorXd::Zero(qddot_sol.size());
  RobotSystem* robot_system = observation.robot_system;
  const RobotStateView current_state = SelectCurrentStateForRnea(
      observation, static_cast<std::size_t>(qddot_sol.size()));
  if (robot_system == nullptr || current_state.q == nullptr ||
      current_state.qdot == nullptr) {
    return tau_ff_cmd;
  }

  const Eigen::VectorXd rnea =
      pinocchio::rnea(robot_system->model(), robot_system->data(),
                      *current_state.q, *current_state.qdot, qddot_sol);
  if (rnea.size() == tau_ff_cmd.size() && rnea.allFinite()) {
    tau_ff_cmd = rnea;
  }
  return tau_ff_cmd;
}

}  // namespace

RobotCommand MakeRobotCommandFromQddot(
    const GraspObservation& observation,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol, double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    throw std::invalid_argument("MakeRobotCommandFromQddot: dt must be positive");
  }
  const auto action_dim = static_cast<std::size_t>(qddot_sol.size());
  if (action_dim == 0 || !qddot_sol.allFinite()) {
    throw std::invalid_argument(
        "MakeRobotCommandFromQddot: qddot_sol is empty or nonfinite");
  }
  if (!HasCompatibleReferenceConfiguration(observation, action_dim) ||
      !observation.q_ref_current.allFinite()) {
    throw std::invalid_argument(
        "MakeRobotCommandFromQddot: q_ref_current dimension mismatch or nonfinite");
  }
  if (observation.tau_meas.size() !=
          static_cast<Eigen::Index>(action_dim) ||
      !observation.tau_meas.allFinite()) {
    throw std::invalid_argument(
        "MakeRobotCommandFromQddot: tau_meas dimension mismatch or nonfinite");
  }

  RobotCommand command;
  command.Resize(static_cast<int>(observation.q_ref_current.size()),
                 static_cast<int>(action_dim));
  command.stamp_sec = observation.time_s;
  const Eigen::VectorXd qdot_ref_current =
      ReferenceVelocityOrZero(observation, action_dim);
  command.qdot_cmd = qdot_ref_current + qddot_sol * dt;
  command.q_cmd = IntegrateReferenceStep(observation, command.qdot_cmd * dt);
  if (!command.q_cmd.allFinite()) {
    throw std::invalid_argument("MakeRobotCommandFromQddot: q_cmd nonfinite");
  }
  command.tau_cmd = CommandFeedForwardTorqueOrZero(observation, qddot_sol);
  command.valid = command.HasValidDimensions() && command.AllFinite();
  return command;
}

}  // namespace mppi_core
