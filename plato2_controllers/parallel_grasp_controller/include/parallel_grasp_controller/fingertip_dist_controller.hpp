#pragma once

#include <array>
#include <vector>

class FingertipDistController {
public:
  static constexpr double kIndexProximalMin = -1.0472;
  static constexpr double kIndexProximalMax = 1.0472;
  static constexpr double kIndexPipMin = -1.0472;
  static constexpr double kIndexPipMax = 2.0944;
  static constexpr double kMiddleProximalMin = -1.0472;
  static constexpr double kMiddleProximalMax = 1.0472;
  static constexpr double kMaxIndexPipFlexion = 2.0944;
  static constexpr double kMaxMiddlePipFlexion = 2.0944;
  static constexpr double kParkedMcp = 0.785;
  static constexpr double kParkedPip = 1.5708;

  FingertipDistController();

  void update(const std::array<double, 5>& commands,
              const std::vector<double>& current_positions);
  void set_state(int state);

  const std::vector<double>& get_commands() const { return joint_commands_; }

private:
  static double clamp01(double value);
  static double map_distance_to_flexion(double value, double min, double max);
  static double map_phi_to_pip(double proximal_target, double phi, double max_flexion);

  std::vector<double> joint_commands_ = std::vector<double>(8, 0.0);
  static constexpr std::array<double, 8> kPokingPosture{
    0.0, 0.0, 0.10, -1.96, 0.0, 0.0, 0.97, 1.43};
  static constexpr std::array<double, 3> kIndexPinchThumbPosture{
    0.0, 0.0, 0.0};
  static constexpr std::array<double, 3> kMiddlePinchThumbPosture{
    -0.430, -0.450, 0.0};
  int state_ = 1;
};
