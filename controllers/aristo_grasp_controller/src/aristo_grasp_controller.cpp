#include "aristo_grasp_controller/aristo_grasp_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace aristo_grasp_controller
{
namespace
{
constexpr const char * thumbTipFrame = "thumb_fingertip";
constexpr const char * indexTipFrame = "index_fingertip";
constexpr double alignmentWeightM = 0.02;
constexpr int gridSteps = 24;

std::vector<double> pad_positions(const std::vector<double> & positions)
{
  std::vector<double> padded(8, 0.0);
  const auto count = std::min(padded.size(), positions.size());
  std::copy_n(positions.begin(), count, padded.begin());
  return padded;
}

double lerp(double a, double b, double t)
{
  return a + (b - a) * t;
}
}  // namespace

AristoGraspController::AristoGraspController(std::shared_ptr<AristoKinematics> kinematics)
: kinematics_(std::move(kinematics))
{
  if (!kinematics_) {
    throw std::invalid_argument("AristoGraspController requires a kinematics instance");
  }
}

void AristoGraspController::update(
  const std::array<double, 3> & commands,
  const std::vector<double> & current_positions,
  double measured_force)
{
  const double u = std::clamp(commands[0], 0.0, 1.0);
  const double phi = std::clamp(commands[1], 0.0, 1.0);
  const double desired_force = commands[2];

  const bool force_requested = desired_force > 0.0;
  const bool force_detected = measured_force > 0.0;
  const bool can_close_for_force = u <= 0.6;

  if (force_requested && force_detected && can_close_for_force) {
    force_control_active_ = true;
  } else if (!can_close_for_force) {
    force_control_active_ = false;
  }

  double u_final = u;
  if (force_control_active_) {
    const double force_error = desired_force - measured_force;
    u_final = std::clamp(u - admittance_gain_ * force_error, 0.0, 0.6);
  }

  const auto proximal = solve_proximal_targets_(u_final, current_positions);
  joint_commands_ = pad_positions(current_positions);
  joint_commands_[0] = neutral;
  joint_commands_[1] = neutral;
  joint_commands_[2] = proximal.q3;
  joint_commands_[4] = proximal.q5;
  joint_commands_[6] = joint7Hold;
  joint_commands_[7] = joint8Hold;
  update_phi_(phi, joint_commands_);
}

AristoGraspController::ProximalSolution AristoGraspController::solve_proximal_targets_(
  double u_cmd,
  const std::vector<double> & current_positions)
{
  Eigen::VectorXd q = kinematics_->make_configuration(pad_positions(current_positions));

  double min_distance = std::numeric_limits<double>::infinity();
  double max_distance = -std::numeric_limits<double>::infinity();

  for (int i = 0; i <= gridSteps; ++i) {
    const double q3 = lerp(q3Min, q3Max, static_cast<double>(i) / gridSteps);
    for (int j = 0; j <= gridSteps; ++j) {
      const double q5 = lerp(q5Min, q5Max, static_cast<double>(j) / gridSteps);
      q[2] = q3;
      q[4] = q5;
      kinematics_->update(q);
      const double distance = kinematics_->frame_distance(thumbTipFrame, indexTipFrame);
      min_distance = std::min(min_distance, distance);
      max_distance = std::max(max_distance, distance);
    }
  }

  const double desired_distance = lerp(min_distance, max_distance, std::clamp(u_cmd, 0.0, 1.0));
  ProximalSolution best;
  double best_cost = std::numeric_limits<double>::infinity();

  for (int i = 0; i <= gridSteps; ++i) {
    const double q3 = lerp(q3Min, q3Max, static_cast<double>(i) / gridSteps);
    for (int j = 0; j <= gridSteps; ++j) {
      const double q5 = lerp(q5Min, q5Max, static_cast<double>(j) / gridSteps);
      q[2] = q3;
      q[4] = q5;
      kinematics_->update(q);

      const double distance = kinematics_->frame_distance(thumbTipFrame, indexTipFrame);
      const double alignment = std::abs(
        kinematics_->frame_axis_alignment(thumbTipFrame, indexTipFrame, 0));
      const double cost =
        std::abs(distance - desired_distance) + alignmentWeightM * (1.0 - alignment);

      if (cost < best_cost) {
        best_cost = cost;
        best.q3 = q3;
        best.q5 = q5;
      }
    }
  }

  return best;
}

void AristoGraspController::update_phi_(
  double phi_cmd,
  const std::vector<double> & proximal_targets)
{
  const double phi = std::clamp(phi_cmd, 0.0, 1.0);
  const double q3 = proximal_targets.size() > 2 ? proximal_targets[2] : 0.0;
  const double q5 = proximal_targets.size() > 4 ? proximal_targets[4] : 0.0;

  joint_commands_[3] = -q3 - phi * maxFlexionAngle;
  joint_commands_[5] = -q5 + phi * maxFlexionAngle;
}

}  // namespace aristo_grasp_controller
