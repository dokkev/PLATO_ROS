// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace mppi_core {

// Pinocchio model/data used for robot-wide integration and dynamics.
//
// Contact kinematics contexts additionally carry tactile sensor frame
// information. Keep this context frame-free so robot rollout does not need to
// select an arbitrary tactile sensor as the dynamics owner.
struct RobotDynamicsContext {
  const pinocchio::Model* model{nullptr};
  pinocchio::Data* data{nullptr};
};

inline bool IsValidRobotDynamicsContext(const RobotDynamicsContext& context) {
  return context.model != nullptr && context.data != nullptr &&
         context.data->oMi.size() == context.model->joints.size();
}

}  // namespace mppi_core
