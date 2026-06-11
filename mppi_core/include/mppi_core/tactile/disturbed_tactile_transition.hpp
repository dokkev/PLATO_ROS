// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <vector>

#include "mppi_core/contact/hemisphere_motion.hpp"
#include "mppi_core/disturbance/grasp_disturbance.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"
#include "mppi_core/tactile/tactile_state.hpp"
#include "mppi_core/tactile/tactile_transition.hpp"

namespace mppi_core {

struct DisturbedTactileTransitionConfig {
  TactileTransitionConfig base;

  double normal_stiffness_n_per_m{500.0};
  double max_normal_force_n{10.0};
  double min_contact_force_n{0.02};

  double max_abs_shear_m{0.010};
  double max_abs_rotation_rad{0.30};

  bool lose_contact_below_min_force{true};
};

TactileState StepTactileStateWithDisturbance(
    const TactileState& tactile, const RobotState& robot,
    const RobotState& next_robot, const std::vector<HemisphereMotion>& motions,
    const TactileSensorContext& sensor_context,
    const DisturbedTactileTransitionConfig& config,
    const TactileSensorDisturbance& disturbance, double dt);

}  // namespace mppi_core
