// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/StdVector>

#include <cstddef>
#include <vector>

#include "mppi_core/object/object_prior.hpp"
#include "mppi_core/object/virtual_object_state.hpp"

namespace mppi_core {

struct VirtualObjectBelief {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<VirtualObjectState,
              Eigen::aligned_allocator<VirtualObjectState>>
      particles;
  ObjectGeometryHandle geometry;
  bool valid{false};

  std::size_t particleCount() const { return particles.size(); }
  bool hasParticles() const { return !particles.empty(); }
};

inline bool HasVirtualObjectBelief(const VirtualObjectBelief& belief) {
  return belief.valid || belief.hasParticles() ||
         HasObjectGeometry(belief.geometry);
}

inline bool IsValidVirtualObjectBelief(const VirtualObjectBelief& belief) {
  if (!HasVirtualObjectBelief(belief)) {
    return true;
  }
  if (!belief.valid || !IsValidObjectGeometry(belief.geometry) ||
      belief.particles.empty()) {
    return false;
  }
  for (const auto& particle : belief.particles) {
    if (!IsValidVirtualObjectState(particle)) {
      return false;
    }
  }
  return true;
}

}  // namespace mppi_core
