// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

namespace mppi_core {

struct PinocchioContactKinematicsContext;

struct TactileSensorContext {
  int sensor_index{-1};

  const PinocchioContactKinematicsContext* kinematics{nullptr};

  // Optional future fields:
  // const TactileSensorGeometry* geometry{nullptr};
  // const TactileTransitionConfig* transition_config{nullptr};
};

}  // namespace mppi_core
