// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct TactileTransitionConfig;
struct ContactForceCorrectionState;
struct ContactForceProjectionConfig;
struct ContactForceRolloutConfig;
struct PinocchioContactKinematicsContext;

struct GraspObservation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd q_meas;
  Eigen::VectorXd qdot_meas;

  // tau_meas is required as robot feedback. The current MVP rollout does not
  // use tau_meas for residual projection. Residual-based GraspState correction
  // will be added later as an observation-time preprocessing step.
  Eigen::VectorXd tau_meas;

  Eigen::VectorXd q_ref_current;
  Eigen::VectorXd qdot_ref_current;

  std::vector<TactileState, Eigen::aligned_allocator<TactileState>>
      tactile_meas;

  // Pinocchio RNEA updates Data as a computation cache, so rollout code needs a
  // mutable RobotSystem even when the observation values themselves are const.
  RobotSystem* robot_system{nullptr};
  std::vector<TactileSensorContext> tactile_contexts{};
  const TactileTransitionConfig* tactile_transition_config{nullptr};

  // Reserved for residual/contact-force correction experiments. The current
  // GraspStateRolloutModel does not use these pointers in horizon dynamics.
  const ContactForceProjectionConfig* contact_force_projection_config{nullptr};
  const ContactForceRolloutConfig* contact_force_rollout_config{nullptr};
  const ContactForceCorrectionState* contact_force_correction_state{nullptr};
  double time_s{0.0};
};

}  // namespace mppi_core
