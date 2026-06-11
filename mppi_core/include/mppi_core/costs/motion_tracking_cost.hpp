// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <string>

#include "mppi_core/costs/cost_term_base.hpp"

namespace mppi_core {

struct MotionTrackingCostConfig {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double q_target_weight{20.0};
  double qdot_target_weight{1.0};
  double qddot_weight{0.01};
  double tau_weight{0.001};

  double joint_limit_weight{2.0};
  double joint_limit_margin_rad{0.10};

  bool enable_line_of_action_cost{false};
  double line_of_action_weight{5.0};
  std::string frame_a_name;
  std::string frame_b_name;
  Eigen::Vector3d close_axis_base{0.0, 0.0, 1.0};
};

class MotionTrackingCost final : public CostTermBase {
 public:
  explicit MotionTrackingCost(MotionTrackingCostConfig config);

  double Evaluate(const GraspState& state,
                  const Eigen::Ref<const Eigen::VectorXd>& action,
                  const CostContext& context) const override;

 private:
  double TargetConfigurationCost(
      const GraspState& state,
      const GraspObservation& observation) const;
  double TargetVelocityCost(
      const GraspState& state,
      const GraspObservation& observation) const;
  double JointLimitCost(
      const GraspState& state,
      const RobotSystem* robot_system) const;
  double LineOfActionCost(
      const GraspState& state,
      RobotSystem* robot_system) const;

  MotionTrackingCostConfig config_;
};

}  // namespace mppi_core
