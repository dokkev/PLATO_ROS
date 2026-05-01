#include "parallel_grasp_controller/fingertip_dist_controller.hpp"

#include <algorithm>
#include <stdexcept>

FingertipDistController::FingertipDistController() = default;

void FingertipDistController::update(const std::array<double, 5>& commands,
                                     const std::vector<double>& current_positions)
{
  const size_t available = std::min(joint_commands_.size(), current_positions.size());
  for (size_t i = 0; i < available; ++i) {
    joint_commands_[i] = current_positions[i];
  }
  for (size_t i = available; i < joint_commands_.size(); ++i) {
    joint_commands_[i] = 0.0;
  }

  if (state_ == 0) {
    joint_commands_.assign(kPokingPosture.begin(), kPokingPosture.end());
    return;
  }

  const auto& thumb = (state_ == 2) ? kMiddlePinchThumbPosture : kIndexPinchThumbPosture;
  joint_commands_[0] = thumb[0];
  joint_commands_[1] = thumb[1];
  joint_commands_[2] = thumb[2];

  // commands[2] is T_IP in the message format, but thumb IP is owned by the
  // thumb-state posture. Keep joint4 at its current position.
  (void)commands[2];

  if (state_ == 1) {
    const double index_mcp_target = map_distance_to_flexion(
      commands[0], kIndexProximalMin, kIndexProximalMax);
    joint_commands_[4] = index_mcp_target;
    joint_commands_[5] = map_phi_to_pip(
      index_mcp_target, commands[3], kMaxIndexPipFlexion);

    joint_commands_[6] = kParkedMcp;
    joint_commands_[7] = kParkedPip;
  } else if (state_ == 2) {
    joint_commands_[4] = kParkedMcp;
    joint_commands_[5] = kParkedPip;

    const double middle_mcp_target = map_distance_to_flexion(
      commands[1], kMiddleProximalMin, kMiddleProximalMax);
    joint_commands_[6] = middle_mcp_target;
    joint_commands_[7] = map_phi_to_pip(
      middle_mcp_target, commands[4], kMaxMiddlePipFlexion);
  }
}

void FingertipDistController::set_state(int state)
{
  if (state < 0 || state > 2) {
    throw std::out_of_range("state must be one of 0, 1, 2");
  }
  state_ = state;
}

double FingertipDistController::clamp01(double value)
{
  return std::clamp(value, 0.0, 1.0);
}

double FingertipDistController::map_distance_to_flexion(double value, double min, double max)
{
  // Distance command convention: 0.0 is closed/flexed, 1.0 is open/extended.
  const double u = clamp01(value);
  return max - u * (max - min);
}

double FingertipDistController::map_phi_to_pip(double proximal_target,
                                               double phi,
                                               double max_flexion)
{
  // Matches ParallelGraspController::update_phi for the finger side:
  // mirror the proximal angle, then add normalized flexion.
  return -proximal_target + clamp01(phi) * max_flexion;
}
