#ifndef ARISTO_GRASP_CONTROLLER__ARISTO_GRASP_CONTROLLER_HPP_
#define ARISTO_GRASP_CONTROLLER__ARISTO_GRASP_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "aristo_grasp_controller/aristo_kinematics.hpp"

namespace aristo_grasp_controller
{

class AristoGraspController
{
public:
  static constexpr double pi = 3.14159265358979323846;
  static constexpr double degToRad = pi / 180.0;

  explicit AristoGraspController(std::shared_ptr<AristoKinematics> kinematics);

  void update(
    const std::array<double, 3> & commands,
    const std::vector<double> & current_positions,
    double measured_force = 0.0);

  const std::vector<double> & commands() const { return joint_commands_; }

  void set_admittance_gain(double gain) { admittance_gain_ = gain; }

private:
  struct ProximalSolution
  {
    double q3 = 0.0;
    double q5 = 0.0;
  };

  ProximalSolution solve_proximal_targets_(
    double u_cmd,
    const std::vector<double> & current_positions);
  void update_phi_(double phi_cmd, const std::vector<double> & proximal_targets);

  std::shared_ptr<AristoKinematics> kinematics_;
  std::vector<double> joint_commands_ = std::vector<double>(8, 0.0);

  double admittance_gain_ = 0.05;
  bool force_control_active_ = false;

  static constexpr double q3Min = -45.0 * degToRad;
  static constexpr double q3Max = 0.0;
  static constexpr double q5Min = 1.0e-3;
  static constexpr double q5Max = 1.4;
  static constexpr double neutral = 0.0;
  static constexpr double joint7Hold = 0.785;
  static constexpr double joint8Hold = 1.5708;
  static constexpr double maxFlexionAngle = 0.785;
};

}  // namespace aristo_grasp_controller

#endif  // ARISTO_GRASP_CONTROLLER__ARISTO_GRASP_CONTROLLER_HPP_
