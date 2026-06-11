// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/state/grasp_observation.hpp"

namespace mppi_core {

RobotCommand MakeRobotCommandFromQddot(
    const GraspObservation& observation,
    const Eigen::Ref<const Eigen::VectorXd>& qddot_sol, double dt);

}  // namespace mppi_core
