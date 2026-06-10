#ifndef PLATO_STATE_ESTIMATOR_REFERENCE_GRASP_FORCE_GENERATOR_HPP
#define PLATO_STATE_ESTIMATOR_REFERENCE_GRASP_FORCE_GENERATOR_HPP

#include <cstdint>

namespace plato_state_estimator
{

enum class SlipState
{
  NO_CONTACT,
  PARTIAL_CONTACT,
  STABLE_GRASP,
  TRANSLATIONAL_SLIP,
  ROTATIONAL_SLIP,
  COMBINED_SLIP
};

struct TactileData
{
  enum ContactState
  {
    NO_CONTACT = 0,
    FEW_CONTACTS = 1,
    ENOUGH_CONTACTS = 2
  };

  int32_t contact_state{NO_CONTACT};

  // Shear displacement in the tactile sensor frame: [mm, mm, rad].
  double shear_x{0.0};
  double shear_y{0.0};
  double shear_theta{0.0};

  // Forces [N]. The generator only uses force_z as normal force.
  double force_x{0.0};
  double force_y{0.0};
  double force_z{0.0};

  double timestamp{0.0};
};

struct ReferenceGraspForceConfig
{
  double base_force_n{1.0};
  double min_contact_force_n{0.5};
  double max_force_limit_n{30.0};

  double trans_deadband_mm{0.2};
  double rot_deadband_rad{0.03};

  double translational_slip_threshold_mm{0.5};
  double rotational_slip_threshold_rad{0.05};

  double k_trans_n_per_mm{2.0};
  double d_trans_n_per_mm_s{0.05};

  double k_rot_n_per_rad{10.0};
  double d_rot_n_per_rad_s{0.2};

  double tactile0_shear_x_sign{1.0};
  double tactile0_shear_y_sign{1.0};
  double tactile0_shear_theta_sign{1.0};
  double tactile1_shear_x_sign{1.0};
  double tactile1_shear_y_sign{1.0};
  double tactile1_shear_theta_sign{1.0};
};

struct ReferenceGraspForceOutput
{
  SlipState slip_state{SlipState::NO_CONTACT};

  double target_normal_force_n{0.0};
  double measured_normal_force_min_n{0.0};
  double measured_normal_force_avg_n{0.0};

  double shear_translation_mm{0.0};
  double shear_rotation_rad{0.0};

  bool reference_valid{false};
};

// ROS-independent, slip-aware normal force reference generator.
//
// This class does not estimate full object state and does not choose task
// behavior. It reports whether a force reference is valid; higher-level tasks
// decide whether to close, release, hold, or fall back to motion control.
class ReferenceGraspForceGenerator
{
public:
  explicit ReferenceGraspForceGenerator(const ReferenceGraspForceConfig & config);

  ReferenceGraspForceOutput update(
    const TactileData & tactile0,
    const TactileData & tactile1,
    double dt_sec);

  void reset();

  const ReferenceGraspForceConfig & getConfig() const { return config_; }
  void setConfig(const ReferenceGraspForceConfig & config);

private:
  struct CommonShear
  {
    double ux_avg_mm{0.0};
    double uy_avg_mm{0.0};
    double utheta_avg_rad{0.0};
  };

  bool hasContact(const TactileData & tactile) const;
  CommonShear commonShear(
    const TactileData & tactile0,
    const TactileData & tactile1) const;
  SlipState detectSlip(const CommonShear & shear) const;
  double targetNormalForce(const CommonShear & shear, double dt_sec);
  void resetFeedbackState();

  ReferenceGraspForceConfig config_;
  ReferenceGraspForceOutput last_output_;

  double last_u_trans_mm_{0.0};
  double last_u_rot_rad_{0.0};
  bool has_last_shear_{false};
};

const char * toString(SlipState slip_state);

}  // namespace plato_state_estimator

#endif  // PLATO_STATE_ESTIMATOR_REFERENCE_GRASP_FORCE_GENERATOR_HPP
