// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>

#include <cstddef>

#include "mppi_core/object/object_contact_support_evaluator.hpp"
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
  double force_balance_deadband_n{0.05};

  double shear_weight{20.0};
  double rotation_weight{20.0};
  double slip_score_weight{10.0};
  double shear_safe_limit_m{0.001};
  double rotation_safe_limit_rad{0.01};
  double slip_score_safe_limit{0.05};

  bool enable_contact_line_alignment{true};
  double contact_line_alignment_weight{50.0};
  double contact_line_alignment_deadband_m{0.002};
  Eigen::Vector3d close_axis_base{0.0, 0.0, 1.0};

  double qddot_weight{0.01};
  double tau_weight{0.0};

  ObjectContactSupportEvaluatorConfig object_support;
};

struct RobustGraspStateCostBreakdown {
  double tactile_contact_support_cost{0.0};
  double force_cost{0.0};
  double preload_cost{0.0};
  double force_balance_cost{0.0};
  double shear_cost{0.0};
  double contact_line_alignment_cost{0.0};
  double action_cost{0.0};
  double torque_cost{0.0};
  double object_support_cost{0.0};

  ObjectContactSupportEvaluation object_support;

  double totalCost() const {
    return tactile_contact_support_cost + force_cost + shear_cost +
           contact_line_alignment_cost + action_cost + torque_cost +
           object_support_cost;
  }
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
  double Evaluate(const GraspState& state,
                  const Eigen::Ref<const Eigen::VectorXd>& qddot_sol,
                  const RolloutContext& context,
                  RobustGraspStateCostBreakdown* breakdown) const;

 private:
  RobustGraspStateCostConfig config_;
};

}  // namespace mppi_core
