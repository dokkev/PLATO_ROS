#include "plato_state_estimator/reference_grasp_force_generator.hpp"

#include <algorithm>
#include <cmath>

namespace plato_state_estimator
{
namespace
{

double clampValue(double value, double lower, double upper)
{
  if (upper < lower) {
    upper = lower;
  }
  return std::max(lower, std::min(value, upper));
}

double nonnegativeFiniteOr(double value, double fallback)
{
  if (!std::isfinite(value) || value < 0.0) {
    return fallback;
  }
  return value;
}

double finiteSignOr(double value, double fallback)
{
  if (!std::isfinite(value) || std::abs(value) < 1.0e-12) {
    return fallback;
  }
  return value >= 0.0 ? 1.0 : -1.0;
}

ReferenceGraspForceConfig sanitizeConfig(ReferenceGraspForceConfig config)
{
  config.base_force_n = nonnegativeFiniteOr(config.base_force_n, 1.0);
  config.min_contact_force_n = nonnegativeFiniteOr(config.min_contact_force_n, 0.5);
  config.max_force_limit_n = nonnegativeFiniteOr(config.max_force_limit_n, 30.0);
  if (config.max_force_limit_n < config.min_contact_force_n) {
    config.max_force_limit_n = config.min_contact_force_n;
  }

  config.trans_deadband_mm = nonnegativeFiniteOr(config.trans_deadband_mm, 0.2);
  config.rot_deadband_rad = nonnegativeFiniteOr(config.rot_deadband_rad, 0.03);
  config.translational_slip_threshold_mm =
    nonnegativeFiniteOr(config.translational_slip_threshold_mm, 0.5);
  config.rotational_slip_threshold_rad =
    nonnegativeFiniteOr(config.rotational_slip_threshold_rad, 0.05);

  config.k_trans_n_per_mm = nonnegativeFiniteOr(config.k_trans_n_per_mm, 2.0);
  config.d_trans_n_per_mm_s = nonnegativeFiniteOr(config.d_trans_n_per_mm_s, 0.05);
  config.k_rot_n_per_rad = nonnegativeFiniteOr(config.k_rot_n_per_rad, 10.0);
  config.d_rot_n_per_rad_s = nonnegativeFiniteOr(config.d_rot_n_per_rad_s, 0.2);

  config.tactile0_shear_x_sign = finiteSignOr(config.tactile0_shear_x_sign, 1.0);
  config.tactile0_shear_y_sign = finiteSignOr(config.tactile0_shear_y_sign, 1.0);
  config.tactile0_shear_theta_sign =
    finiteSignOr(config.tactile0_shear_theta_sign, 1.0);
  config.tactile1_shear_x_sign = finiteSignOr(config.tactile1_shear_x_sign, 1.0);
  config.tactile1_shear_y_sign = finiteSignOr(config.tactile1_shear_y_sign, 1.0);
  config.tactile1_shear_theta_sign =
    finiteSignOr(config.tactile1_shear_theta_sign, 1.0);

  return config;
}

}  // namespace

ReferenceGraspForceGenerator::ReferenceGraspForceGenerator(
  const ReferenceGraspForceConfig & config)
{
  setConfig(config);
}

ReferenceGraspForceOutput ReferenceGraspForceGenerator::update(
  const TactileData & tactile0,
  const TactileData & tactile1,
  const double dt_sec)
{
  ReferenceGraspForceOutput output;

  const bool contact0 = hasContact(tactile0);
  const bool contact1 = hasContact(tactile1);

  if (contact0 && contact1) {
    output.measured_normal_force_min_n = std::min(tactile0.force_z, tactile1.force_z);
    output.measured_normal_force_avg_n = 0.5 * (tactile0.force_z + tactile1.force_z);
  } else if (contact0) {
    output.measured_normal_force_min_n = 0.0;
    output.measured_normal_force_avg_n = 0.5 * tactile0.force_z;
  } else if (contact1) {
    output.measured_normal_force_min_n = 0.0;
    output.measured_normal_force_avg_n = 0.5 * tactile1.force_z;
  }

  if (!contact0 && !contact1) {
    output.slip_state = SlipState::NO_CONTACT;
    output.reference_valid = false;
    output.target_normal_force_n = 0.0;
    resetFeedbackState();
    last_output_ = output;
    return output;
  }

  const CommonShear shear = commonShear(tactile0, tactile1);
  output.shear_translation_mm =
    std::sqrt(shear.ux_avg_mm * shear.ux_avg_mm + shear.uy_avg_mm * shear.uy_avg_mm);
  output.shear_rotation_rad = std::abs(shear.utheta_avg_rad);

  if (!contact0 || !contact1 ||
    output.measured_normal_force_min_n < config_.min_contact_force_n)
  {
    output.slip_state = SlipState::PARTIAL_CONTACT;
    output.reference_valid = false;
    output.target_normal_force_n = 0.0;
    resetFeedbackState();
    last_output_ = output;
    return output;
  }

  output.slip_state = detectSlip(shear);
  output.target_normal_force_n = targetNormalForce(shear, dt_sec);
  output.reference_valid = true;

  last_output_ = output;
  return output;
}

void ReferenceGraspForceGenerator::reset()
{
  resetFeedbackState();
  last_output_ = ReferenceGraspForceOutput();
}

void ReferenceGraspForceGenerator::setConfig(const ReferenceGraspForceConfig & config)
{
  config_ = sanitizeConfig(config);
  reset();
}

bool ReferenceGraspForceGenerator::hasContact(const TactileData & tactile) const
{
  return tactile.contact_state >= TactileData::FEW_CONTACTS;
}

ReferenceGraspForceGenerator::CommonShear ReferenceGraspForceGenerator::commonShear(
  const TactileData & tactile0,
  const TactileData & tactile1) const
{
  CommonShear shear;
  const double ux0 = config_.tactile0_shear_x_sign * tactile0.shear_x;
  const double uy0 = config_.tactile0_shear_y_sign * tactile0.shear_y;
  const double utheta0 = config_.tactile0_shear_theta_sign * tactile0.shear_theta;
  const double ux1 = config_.tactile1_shear_x_sign * tactile1.shear_x;
  const double uy1 = config_.tactile1_shear_y_sign * tactile1.shear_y;
  const double utheta1 = config_.tactile1_shear_theta_sign * tactile1.shear_theta;

  shear.ux_avg_mm = 0.5 * (ux0 + ux1);
  shear.uy_avg_mm = 0.5 * (uy0 + uy1);
  shear.utheta_avg_rad = 0.5 * (utheta0 + utheta1);
  return shear;
}

SlipState ReferenceGraspForceGenerator::detectSlip(const CommonShear & shear) const
{
  const double u_trans_mm =
    std::sqrt(shear.ux_avg_mm * shear.ux_avg_mm + shear.uy_avg_mm * shear.uy_avg_mm);
  const double u_rot_rad = std::abs(shear.utheta_avg_rad);

  const bool trans_slip = u_trans_mm > config_.translational_slip_threshold_mm;
  const bool rot_slip = u_rot_rad > config_.rotational_slip_threshold_rad;

  if (trans_slip && rot_slip) {
    return SlipState::COMBINED_SLIP;
  }
  if (trans_slip) {
    return SlipState::TRANSLATIONAL_SLIP;
  }
  if (rot_slip) {
    return SlipState::ROTATIONAL_SLIP;
  }
  return SlipState::STABLE_GRASP;
}

double ReferenceGraspForceGenerator::targetNormalForce(
  const CommonShear & shear,
  const double dt_sec)
{
  const double u_trans_mm =
    std::sqrt(shear.ux_avg_mm * shear.ux_avg_mm + shear.uy_avg_mm * shear.uy_avg_mm);
  const double u_rot_rad = std::abs(shear.utheta_avg_rad);

  const double trans_error_mm =
    std::max(0.0, u_trans_mm - config_.trans_deadband_mm);
  const double rot_error_rad =
    std::max(0.0, u_rot_rad - config_.rot_deadband_rad);

  double trans_rate_mm_s = 0.0;
  double rot_rate_rad_s = 0.0;
  if (has_last_shear_ && std::isfinite(dt_sec) && dt_sec > 0.0) {
    trans_rate_mm_s = std::max(0.0, (u_trans_mm - last_u_trans_mm_) / dt_sec);
    rot_rate_rad_s = std::max(0.0, (u_rot_rad - last_u_rot_rad_) / dt_sec);
  }

  last_u_trans_mm_ = u_trans_mm;
  last_u_rot_rad_ = u_rot_rad;
  has_last_shear_ = true;

  const double force_increment_n =
    config_.k_trans_n_per_mm * trans_error_mm +
    config_.d_trans_n_per_mm_s * trans_rate_mm_s +
    config_.k_rot_n_per_rad * rot_error_rad +
    config_.d_rot_n_per_rad_s * rot_rate_rad_s;

  return clampValue(
    config_.base_force_n + force_increment_n,
    config_.min_contact_force_n,
    config_.max_force_limit_n);
}

void ReferenceGraspForceGenerator::resetFeedbackState()
{
  last_u_trans_mm_ = 0.0;
  last_u_rot_rad_ = 0.0;
  has_last_shear_ = false;
}

const char * toString(const SlipState slip_state)
{
  switch (slip_state) {
    case SlipState::NO_CONTACT:
      return "NO_CONTACT";
    case SlipState::PARTIAL_CONTACT:
      return "PARTIAL_CONTACT";
    case SlipState::STABLE_GRASP:
      return "STABLE_GRASP";
    case SlipState::TRANSLATIONAL_SLIP:
      return "TRANSLATIONAL_SLIP";
    case SlipState::ROTATIONAL_SLIP:
      return "ROTATIONAL_SLIP";
    case SlipState::COMBINED_SLIP:
      return "COMBINED_SLIP";
  }
  return "UNKNOWN";
}

}  // namespace plato_state_estimator
