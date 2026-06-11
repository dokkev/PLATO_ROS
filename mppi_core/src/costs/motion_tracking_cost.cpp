// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#include "mppi_core/costs/motion_tracking_cost.hpp"

#include <algorithm>
#include <cmath>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>
#include <utility>

namespace mppi_core {
namespace {

constexpr double kLargeCost = 1.0e30;

bool IsFiniteAndNonnegative(const double value) {
  return std::isfinite(value) && value >= 0.0;
}

bool HasModelData(const RobotSystem* robot_system) {
  return robot_system != nullptr && robot_system->hasModel() &&
         robot_system->data().oMi.size() == robot_system->model().joints.size();
}

bool HasValidMotionTarget(const GraspState& state,
                          const GraspObservation& observation,
                          const RobotSystem* robot_system) {
  if (!observation.motion_target_valid ||
      observation.q_motion_target.size() == 0 ||
      !observation.q_motion_target.allFinite()) {
    return false;
  }
  if (HasModelData(robot_system)) {
    if (observation.q_motion_target.size() !=
        static_cast<Eigen::Index>(robot_system->nq())) {
      return false;
    }
  } else if (observation.q_motion_target.size() != state.robot.q.size()) {
    return false;
  }
  if (observation.qdot_motion_target.size() > 0 &&
      (observation.qdot_motion_target.size() != state.robot.qdot.size() ||
       !observation.qdot_motion_target.allFinite())) {
    return false;
  }
  return true;
}

double WeightedSquaredNorm(const Eigen::VectorXd& error,
                           const Eigen::VectorXd& weights) {
  if (weights.size() == error.size() && weights.allFinite()) {
    return weights.cwiseProduct(error.cwiseAbs2()).sum();
  }
  return error.squaredNorm();
}

double Square(const double value) { return value * value; }

}  // namespace

MotionTrackingCost::MotionTrackingCost(MotionTrackingCostConfig config)
    : config_(std::move(config)) {
  if (!IsFiniteAndNonnegative(config_.q_target_weight) ||
      !IsFiniteAndNonnegative(config_.qdot_target_weight) ||
      !IsFiniteAndNonnegative(config_.qddot_weight) ||
      !IsFiniteAndNonnegative(config_.tau_weight) ||
      !IsFiniteAndNonnegative(config_.joint_limit_weight) ||
      !IsFiniteAndNonnegative(config_.joint_limit_margin_rad) ||
      !IsFiniteAndNonnegative(config_.line_of_action_weight) ||
      !config_.close_axis_base.allFinite()) {
    throw std::invalid_argument(
        "MotionTrackingCost: weights, margins, and axes must be finite and "
        "nonnegative where applicable");
  }
  const double axis_norm = config_.close_axis_base.norm();
  if (config_.enable_line_of_action_cost) {
    if (axis_norm <= 1.0e-12 || config_.frame_a_name.empty() ||
        config_.frame_b_name.empty()) {
      throw std::invalid_argument(
          "MotionTrackingCost: line-of-action cost requires a nonzero axis "
          "and two frame names");
    }
    config_.close_axis_base /= axis_norm;
  }
}

double MotionTrackingCost::Evaluate(
    const GraspState& state, const Eigen::Ref<const Eigen::VectorXd>& action,
    const CostContext& context) const {
  if (context.rollout == nullptr || context.rollout->observation == nullptr ||
      !state.valid || !IsValid(state.robot) || !action.allFinite()) {
    return kLargeCost;
  }

  const auto& observation = *context.rollout->observation;
  RobotSystem* robot_system = context.rollout->robot_system;
  if (!HasValidMotionTarget(state, observation, robot_system)) {
    return kLargeCost;
  }

  double cost = 0.0;
  cost += TargetConfigurationCost(state, observation);
  cost += TargetVelocityCost(state, observation);
  cost += config_.qddot_weight * action.squaredNorm();
  if (state.robot.tau.size() > 0 && state.robot.tau.allFinite()) {
    cost += config_.tau_weight * state.robot.tau.squaredNorm();
  }
  cost += JointLimitCost(state, robot_system);
  cost += LineOfActionCost(state, robot_system);
  return std::isfinite(cost) ? std::min(cost, kLargeCost) : kLargeCost;
}

double MotionTrackingCost::TargetConfigurationCost(
    const GraspState& state,
    const GraspObservation& observation) const {
  Eigen::VectorXd error;
  const RobotSystem* robot_system =
      observation.robot_system != nullptr ? observation.robot_system : nullptr;
  if (HasModelData(robot_system) &&
      state.robot.q.size() == static_cast<Eigen::Index>(robot_system->nq()) &&
      observation.q_motion_target.size() ==
          static_cast<Eigen::Index>(robot_system->nq())) {
    error = pinocchio::difference(
        robot_system->model(), state.robot.q, observation.q_motion_target);
  } else if (state.robot.q.size() == observation.q_motion_target.size()) {
    error = state.robot.q - observation.q_motion_target;
  } else {
    return kLargeCost;
  }

  if (error.size() == 0 || !error.allFinite()) {
    return kLargeCost;
  }
  return config_.q_target_weight *
         WeightedSquaredNorm(error, observation.q_motion_weight);
}

double MotionTrackingCost::TargetVelocityCost(
    const GraspState& state,
    const GraspObservation& observation) const {
  Eigen::VectorXd qdot_target =
      Eigen::VectorXd::Zero(state.robot.qdot.size());
  if (observation.qdot_motion_target.size() > 0) {
    qdot_target = observation.qdot_motion_target;
  }
  if (qdot_target.size() != state.robot.qdot.size() ||
      !qdot_target.allFinite()) {
    return kLargeCost;
  }
  const Eigen::VectorXd error = state.robot.qdot - qdot_target;
  return config_.qdot_target_weight *
         WeightedSquaredNorm(error, observation.qdot_motion_weight);
}

double MotionTrackingCost::JointLimitCost(
    const GraspState& state,
    const RobotSystem* robot_system) const {
  if (!HasModelData(robot_system) ||
      state.robot.q.size() != static_cast<Eigen::Index>(robot_system->nq())) {
    return 0.0;
  }

  const auto& model = robot_system->model();
  if (model.lowerPositionLimit.size() != state.robot.q.size() ||
      model.upperPositionLimit.size() != state.robot.q.size()) {
    return 0.0;
  }

  double cost = 0.0;
  for (Eigen::Index i = 0; i < state.robot.q.size(); ++i) {
    const double lower = model.lowerPositionLimit[i];
    const double upper = model.upperPositionLimit[i];
    if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper) {
      continue;
    }
    const double low_error =
        lower + config_.joint_limit_margin_rad - state.robot.q[i];
    const double high_error =
        state.robot.q[i] - (upper - config_.joint_limit_margin_rad);
    if (low_error > 0.0) {
      cost += config_.joint_limit_weight * Square(low_error);
    }
    if (high_error > 0.0) {
      cost += config_.joint_limit_weight * Square(high_error);
    }
  }
  return cost;
}

double MotionTrackingCost::LineOfActionCost(
    const GraspState& state,
    RobotSystem* robot_system) const {
  if (!config_.enable_line_of_action_cost || !HasModelData(robot_system) ||
      state.robot.q.size() != static_cast<Eigen::Index>(robot_system->nq())) {
    return 0.0;
  }

  const auto frame_a_id = robot_system->model().getFrameId(config_.frame_a_name);
  const auto frame_b_id = robot_system->model().getFrameId(config_.frame_b_name);
  if (frame_a_id >= robot_system->model().frames.size() ||
      frame_b_id >= robot_system->model().frames.size()) {
    return kLargeCost;
  }

  pinocchio::forwardKinematics(
      robot_system->model(), robot_system->data(), state.robot.q);
  pinocchio::updateFramePlacements(robot_system->model(), robot_system->data());
  const Eigen::Vector3d p_a = robot_system->data().oMf[frame_a_id].translation();
  const Eigen::Vector3d p_b = robot_system->data().oMf[frame_b_id].translation();
  const Eigen::Vector3d d = p_b - p_a;
  if (!d.allFinite()) {
    return kLargeCost;
  }
  const Eigen::Vector3d tangent_error =
      d - d.dot(config_.close_axis_base) * config_.close_axis_base;
  return config_.line_of_action_weight * tangent_error.squaredNorm();
}

}  // namespace mppi_core
