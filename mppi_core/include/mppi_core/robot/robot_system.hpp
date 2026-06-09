// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include "plato_robot_system/robot/robot_system.hpp"

namespace mppi_core {

using RobotState = plato_robot_system::RobotState;
using RobotCommand = plato_robot_system::RobotCommand;
using RobotSystem = plato_robot_system::RobotSystem;

using plato_robot_system::IsValid;
using plato_robot_system::MakeInvalidRobotCommand;
using plato_robot_system::MakeRobotState;
using plato_robot_system::MakeZeroHoldRobotCommand;

}  // namespace mppi_core
