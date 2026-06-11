// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include <cstddef>

#include "mppi_core/rollout/rollout_model.hpp"
#include "mppi_core/state/grasp_state.hpp"
#include "mppi_core/tactile/tactile_sensor_context.hpp"

namespace mppi_core {

struct RobustGraspStateCostConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t min_active_tactile_sensors{2};
  std::size_t min_active_hemisphere_total{2};

  double contact_loss_weight{200.0};
  double support_weight{20.0};

  double target_normal_force_n{1.0};
  double min_normal_force_per_sensor_n{0.1};
  double max_normal_force_per_sensor_n{5.0};
  double force_low_weight{30.0};
  double force_high_weight{5.0};
  double force_balance_weight{10.0};

  double shear_weight{20.0};
  double rotation_weight{20.0};
  double slip_score_weight{10.0};

  bool enable_contact_line_alignment{true};
  double contact_line_alignment_weight{50.0};
  Eigen::Vector3d close_axis_base{0.0, 0.0, 1.0};

  double qddot_weight{0.01};
  double tau_weight{0.0};
};

bool ComputeActiveTactileContactCentroidWorld(
    const GraspState& state, const TactileState& tactile,
    const TactileSensorContext& sensor_context,
    Eigen::Vector3d* centroid_world);

class RobustGraspStateCost {
 public:
  explicit RobustGraspStateCost(RobustGraspStateCostConfig config);

  double Evaluate(const GraspState& state,
                  const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
                  const RolloutContext& context) const;

 private:
  RobustGraspStateCostConfig config_;
};

}  // namespace mppi_core
