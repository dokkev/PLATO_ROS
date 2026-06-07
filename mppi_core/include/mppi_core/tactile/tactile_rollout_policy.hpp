// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

namespace mppi_core {

enum class TactileRolloutPolicy {
  // Contact-gated runtime mode. When a single active tactile sensor makes
  // residual projection well-defined, residual-aware grasp-state transition
  // must succeed; otherwise the rollout step is invalid. Multi-sensor active
  // contact uses kinematic fallback until a coupled residual solver exists.
  kResidualRequired = 0,
  // Debug/ablation mode: use residual-aware transition first, then fall back to
  // kinematic contact-patch rollout. No heuristic proxy fallback exists.
  kResidualThenKinematicFallback = 1,
};

}  // namespace mppi_core
