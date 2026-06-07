// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <vector>

#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/state/grasp_observation.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"

namespace mppi_core {

struct GraspRolloutConfig;
struct TactileTransitionConfig;
struct ContactForceCorrectionState;
struct ContactForceProjectionConfig;
struct ContactForceRolloutConfig;
struct PinocchioContactKinematicsContext;

struct RolloutContext {
  const GraspObservation* observation{nullptr};
  const GraspState* initial_reference_state{nullptr};
  RobotSystem* robot_system{nullptr};
  std::vector<TactileSensorContext> tactile_contexts{};
  const TactileTransitionConfig* tactile_transition_config{nullptr};
  const GraspRolloutConfig* grasp_rollout_config{nullptr};

  // Reserved for residual/contact-force correction experiments. The current
  // GraspStateRolloutModel does not use these pointers in horizon dynamics.
  const ContactForceProjectionConfig* contact_force_projection_config{nullptr};
  const ContactForceRolloutConfig* contact_force_rollout_config{nullptr};
  const ContactForceCorrectionState* contact_force_correction_state{nullptr};
};

class RolloutModelBase {
 public:
  virtual ~RolloutModelBase() = default;

  virtual std::size_t actionDim() const = 0;

  virtual void Step(const GraspState& state,
                    const Eigen::Ref<const Eigen::VectorXd>& action,
                    const RolloutContext& context, double dt,
                    GraspState* next_state) const = 0;
};

}  // namespace mppi_core
