// Copyright 2026
//
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Eigen/Core>
#include <cmath>

namespace mppi_core {

// Ideal/reference robot state used by MPPI rollout.
//
// Measured robot values should use explicit measured names such as q_meas,
// qdot_meas, and tau_meas. This type is intentionally the robot component of
// GraspState, not a mixed measured/predicted snapshot.
struct RobotState {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::VectorXd q_des;
  Eigen::VectorXd qdot_des;
  Eigen::VectorXd qddot_des;
  Eigen::VectorXd tau_ff;
  double time_s{0.0};
};

inline RobotState MakeRobotState(const Eigen::VectorXd& q_des,
                                 const Eigen::VectorXd& qdot_des,
                                 const Eigen::VectorXd& qddot_des,
                                 const Eigen::VectorXd& tau_ff,
                                 double time_s = 0.0) {
  RobotState robot;
  robot.q_des = q_des;
  robot.qdot_des = qdot_des;
  robot.qddot_des = qddot_des;
  robot.tau_ff = tau_ff;
  robot.time_s = time_s;
  return robot;
}

inline bool IsValidRobotState(const RobotState& robot) {
  return robot.q_des.size() > 0 && robot.qdot_des.size() > 0 &&
         robot.qdot_des.size() == robot.qddot_des.size() &&
         robot.qdot_des.size() == robot.tau_ff.size() &&
         robot.q_des.allFinite() && robot.qdot_des.allFinite() &&
         robot.qddot_des.allFinite() && robot.tau_ff.allFinite() &&
         std::isfinite(robot.time_s);
}

}  // namespace mppi_core
