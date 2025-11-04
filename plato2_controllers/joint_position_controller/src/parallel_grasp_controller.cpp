#include "joint_position_controller/parallel_grasp_controller.hpp"

ParallelGraspController::ParallelGraspController()
{
}

const std::vector<double>& ParallelGraspController::get_commands(double u_cmd) {
  // Feasible gap range is precomputed in the constructor
  const double u = std::clamp(u_cmd, 0.0, 1.0);
  const double d = d_min_ + u * (d_max_ - d_min_);

  // Closed-form solve for (q3,q5)
  const double dy = d - h;
  const double Lvec = std::hypot(w, dy);
  const double s = std::clamp(Lvec / (2.0 * L), 0.0, 1.0);

  const double m = std::atan2(w, dy);
  const double delta = std::asin(s);


  double q3 = m - delta;
  double q4 = -q3;
  double q5 = m + delta;
  double q6 = -q5;

  // Clamp to joint limits (hardcoded in header)
  q5 = std::clamp(q5, qmin, qmax);
  q3 = std::clamp(q3, qmin, qmax);

  // update internal state
  u_ = u;
  d_ = d;

  // Build and store the 8-element joint command vector (indices 0..7 -> joints 1..8).
  // Write directly into the internal buffer to avoid a temporary allocation.
  joint_position_commands_.assign(8, 0.0);
  joint_position_commands_[2] = q3;
  joint_position_commands_[3] = q4;
  joint_position_commands_[4] = q5;
  joint_position_commands_[5] = q6;

  return joint_position_commands_;
}
