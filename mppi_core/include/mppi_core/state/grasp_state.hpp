// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/StdVector>
#include <cstddef>
#include <vector>

#include "mppi_core/object/object_contact_belief.hpp"
#include "mppi_core/robot/robot_system.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct GraspState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool valid{false};

  RobotState robot;

  std::vector<TactileState, Eigen::aligned_allocator<TactileState>>
      tactile_sensors;

  VirtualObjectBelief object_belief;

  std::size_t tactileSensorCount() const { return tactile_sensors.size(); }
  bool hasObjectBelief() const {
    return HasVirtualObjectBelief(object_belief);
  }

  bool hasAnyTactileContact() const {
    for (const auto& tactile : tactile_sensors) {
      if (tactile.hasActiveHemisphereContact()) {
        return true;
      }
    }
    return false;
  }

  std::size_t activeTactileSensorCount() const {
    std::size_t count = 0;
    for (const auto& tactile : tactile_sensors) {
      if (tactile.hasActiveHemisphereContact()) {
        ++count;
      }
    }
    return count;
  }

  std::size_t activeHemisphereCountTotal() const {
    std::size_t count = 0;
    for (const auto& tactile : tactile_sensors) {
      count += tactile.activeHemisphereCount();
    }
    return count;
  }
};

inline bool HasValidTactileSensors(const GraspState& state) {
  for (const auto& tactile : state.tactile_sensors) {
    if (!tactile.valid) {
      return false;
    }
  }
  return true;
}

inline GraspState MakeGraspState(
    const RobotState& robot,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const VirtualObjectBelief& object_belief = {}) {
  GraspState state;
  state.robot = robot;
  state.tactile_sensors = tactile_sensors;
  state.object_belief = object_belief;
  state.valid = IsValid(robot) && HasValidTactileSensors(state) &&
                IsValidVirtualObjectBelief(state.object_belief);
  return state;
}

inline GraspState MakeGraspState(const RobotState& robot,
                                 const TactileState& first_tactile,
                                 const TactileState& second_tactile,
                                 const VirtualObjectBelief& object_belief = {}) {
  std::vector<TactileState, Eigen::aligned_allocator<TactileState>>
      tactile_sensors;
  tactile_sensors.reserve(2);
  tactile_sensors.push_back(first_tactile);
  tactile_sensors.push_back(second_tactile);
  return MakeGraspState(robot, tactile_sensors, object_belief);
}

inline GraspState MakeGraspState(
    const Eigen::VectorXd& q, const Eigen::VectorXd& qdot,
    const Eigen::VectorXd& tau,
    const std::vector<TactileState, Eigen::aligned_allocator<TactileState>>&
        tactile_sensors,
    const VirtualObjectBelief& object_belief = {}) {
  return MakeGraspState(MakeRobotState(q, qdot, tau), tactile_sensors,
                        object_belief);
}

inline GraspState MakeGraspState(const Eigen::VectorXd& q,
                                 const Eigen::VectorXd& qdot,
                                 const Eigen::VectorXd& tau,
                                 const TactileState& first_tactile,
                                 const TactileState& second_tactile,
                                 const VirtualObjectBelief& object_belief = {}) {
  return MakeGraspState(MakeRobotState(q, qdot, tau), first_tactile,
                        second_tactile, object_belief);
}

struct GraspStartConfig {
  std::size_t min_enough_contact_sensors{1};
  std::size_t min_active_hemispheres_total{1};
};

inline bool ReadyForMppiStart(const GraspState& state,
                              const GraspStartConfig& config = {}) {
  if (!state.valid) {
    return false;
  }

  std::size_t enough_contact_sensors = 0;
  for (const auto& tactile : state.tactile_sensors) {
    if (tactile.readyForMppiStart()) {
      ++enough_contact_sensors;
    }
  }

  return enough_contact_sensors >= config.min_enough_contact_sensors &&
         state.activeHemisphereCountTotal() >=
             config.min_active_hemispheres_total;
}

}  // namespace mppi_core
