// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cstddef>

#include "mppi_core/costs/cost_term_base.hpp"
#include "mppi_core/tactile/tactile_state.hpp"

namespace mppi_core {

struct GraspStabilityCostConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::size_t min_active_tactile_sensors{1};
  std::size_t target_active_hemisphere_total{4};

  double contact_loss_weight{50.0};
  double support_weight{10.0};
  double shear_weight{5.0};
  double rotation_weight{5.0};
  double qddot_weight{1.0};
  double tau_weight{0.01};
};

class GraspStabilityCost final : public CostTermBase {
 public:
  explicit GraspStabilityCost(GraspStabilityCostConfig config);

  double Evaluate(const GraspState& state,
                  const Eigen::Ref<const Eigen::VectorXd>& action,
                  const CostContext& context) const override;

 private:
  double RobotEffortCost(
      const GraspState& state,
      const Eigen::Ref<const Eigen::VectorXd>& action) const;
  double TactileSensorsCost(const GraspState& state) const;

  GraspStabilityCostConfig config_;
};

}  // namespace mppi_core
