#include "plato2_state_estimator/object_state_estimator.hpp"
#include <algorithm>
#include <cmath>

namespace plato2_state_estimator {

ObjectStateEstimator::ObjectStateEstimator(const ObjectStateEstimatorConfig& config)
    : config_(config)
{
    // Initialize PID controllers for feedback correction
    pid_translational_x_ = std::make_unique<plato_utils::PIDController>(
        config_.pid_tx_p, config_.pid_tx_i, config_.pid_tx_d,
        0.0, config_.max_force_limit);

    pid_translational_y_ = std::make_unique<plato_utils::PIDController>(
        config_.pid_tx_p, config_.pid_tx_i, config_.pid_tx_d,
        0.0, config_.max_force_limit);

    pid_rotational_ = std::make_unique<plato_utils::PIDController>(
        config_.pid_theta_p, config_.pid_theta_i, config_.pid_theta_d,
        0.0, config_.max_force_limit);
}

ObjectStateEstimatorOutput ObjectStateEstimator::update(
    const TactileData& tactile0,
    const TactileData& tactile1,
    double dt)
{
    ObjectStateEstimatorOutput output;

    // Validate dt
    if (dt <= 0.0 || dt > 1.0) {
        dt = 0.01;  // Default to 10ms if invalid
    }

    // Check if we have valid contact data
    if (!hasBothContacts(tactile0, tactile1)) {
        output.slip_state = SlipState::NO_CONTACT;
        output.has_valid_data = false;

        // Reset PID controllers when contact is lost
        pid_translational_x_->reset();
        pid_translational_y_->reset();
        pid_rotational_->reset();

        last_output_ = output;
        return output;
    }

    // Detect slip state
    output.slip_state = detectSlip(tactile0, tactile1);

    // Calculate minimal force
    output.minimal_force = calculateMinimalForce(tactile0, tactile1, dt);

    // Aggregate forces from both sensors
    // Only normal force is available; tangential components are assumed zero.
    output.force_x = 0.0;
    output.force_y = 0.0;

    // TODO: Calculate moment_z based on sensor separation and force differential
    output.moment_z = 0.0;

    output.has_valid_data = true;
    last_output_ = output;

    return output;
}

void ObjectStateEstimator::reset() {
    pid_translational_x_->reset();
    pid_translational_y_->reset();
    pid_rotational_->reset();
    last_output_ = ObjectStateEstimatorOutput();
}

void ObjectStateEstimator::setConfig(const ObjectStateEstimatorConfig& config) {
    config_ = config;

    // Recreate PID controllers with new gains
    pid_translational_x_ = std::make_unique<plato_utils::PIDController>(
        config_.pid_tx_p, config_.pid_tx_i, config_.pid_tx_d,
        0.0, config_.max_force_limit);

    pid_translational_y_ = std::make_unique<plato_utils::PIDController>(
        config_.pid_tx_p, config_.pid_tx_i, config_.pid_tx_d,
        0.0, config_.max_force_limit);

    pid_rotational_ = std::make_unique<plato_utils::PIDController>(
        config_.pid_theta_p, config_.pid_theta_i, config_.pid_theta_d,
        0.0, config_.max_force_limit);
}

SlipState ObjectStateEstimator::detectSlip(
    const TactileData& tactile0,
    const TactileData& tactile1)
{
    // Check individual contact states
    bool sensor0_contact = tactile0.contact_state >= TactileData::FEW_CONTACTS;
    bool sensor1_contact = tactile1.contact_state >= TactileData::FEW_CONTACTS;

    if (!sensor0_contact || !sensor1_contact) {
        return SlipState::PARTIAL_CONTACT;
    }

    // Average shear displacements from both sensors
    double ux_avg = (tactile0.shear_x + tactile1.shear_x) / 2.0;
    double uy_avg = (tactile0.shear_y + tactile1.shear_y) / 2.0;
    double utheta_avg = (tactile0.shear_theta + tactile1.shear_theta) / 2.0;

    // Detect slip types
    bool trans_slip = detectTranslationalSlip(ux_avg, uy_avg);
    bool rot_slip = detectRotationalSlip(utheta_avg);

    if (trans_slip && rot_slip) {
        return SlipState::COMBINED_SLIP;
    } else if (trans_slip) {
        return SlipState::TRANSLATIONAL_SLIP;
    } else if (rot_slip) {
        return SlipState::ROTATIONAL_SLIP;
    } else {
        return SlipState::STABLE_GRASP;
    }
}

bool ObjectStateEstimator::detectTranslationalSlip(double ux, double uy) const {
    // Calculate magnitude of translational displacement
    double magnitude = std::sqrt(ux * ux + uy * uy);
    return magnitude > config_.translational_slip_threshold;
}

bool ObjectStateEstimator::detectRotationalSlip(double u_theta) const {
    return std::abs(u_theta) > config_.rotational_slip_threshold;
}

double ObjectStateEstimator::calculateMinimalForce(
    const TactileData& tactile0,
    const TactileData& tactile1,
    double dt)
{
    if (!hasBothContacts(tactile0, tactile1)) {
        return config_.min_contact_force;
    }

    // Average sensor measurements
    double ux_avg = (tactile0.shear_x + tactile1.shear_x) / 2.0;      // mm
    double uy_avg = (tactile0.shear_y + tactile1.shear_y) / 2.0;      // mm
    double utheta_avg = (tactile0.shear_theta + tactile1.shear_theta) / 2.0;  // rad

    // Tangential forces are unavailable from sensors; assume zero.
    double Fx_avg = 0.0;
    double Fy_avg = 0.0;

    // Calculate tangential force magnitude
    double F_tangential = std::sqrt(Fx_avg * Fx_avg + Fy_avg * Fy_avg);

    // Estimate moment from sensor data (simplified: assumes sensor separation, can be improved)
    // For now, use a placeholder - in real implementation, this would depend on gripper geometry
    double T_theta = 0.0;  // TODO: Calculate from force differential between sensors

    // ========== Feedforward Term: Physics-based force calculation ==========
    // Calculate required normal forces from both slip modes using equations (12) and (14)
    double F_N_trans = calculateTranslationalForce(F_tangential, ux_avg, uy_avg);
    double F_N_rot = calculateRotationalForce(T_theta, utheta_avg);

    // Sum physics-based contributions
    double F_feedforward = F_N_trans + F_N_rot;

    // ========== Feedback Term: PID correction for modeling errors ==========
    // PID acts on shear displacement error (goal: drive displacement to zero)
    double F_feedback = calculatePIDFeedback(ux_avg, uy_avg, utheta_avg, dt);

    // ========== Combined Control Output ==========
    // Total force = Feedforward (physics model) + Feedback (PID correction)
    double F_minimal = F_feedforward + F_feedback;

    // Ensure minimum contact force
    F_minimal = std::max(F_minimal, config_.min_contact_force);

    // Apply safety limits (C++14 compatible)
    F_minimal = std::min(F_minimal, config_.max_force_limit);
    F_minimal = std::max(F_minimal, config_.min_contact_force);

    return F_minimal;
}

double ObjectStateEstimator::calculateTranslationalForce(
    double F_tangential, double ux, double uy)
{
    // Based on equation (12) from Narita et al., ICRA 2020:
    // u_x = F_x / (G* * K_t * F_N^{1/(n+1)})
    // where K_t = ((n+1)/(2n) * 1/(E* * C_n * lambda_n^n))^{1/(n+1)}
    //
    // Rearranged to solve for F_N:
    // F_N = (F_x / (G* * u_x))^{(n+1)} / A
    // where A = (n+1)/(2n) * 1/(E* * C_n * lambda_n^n)

    constexpr double EPSILON = 1e-12;
    constexpr double SLIP_NOISE_THRESHOLD = 0.25e-3;  // 0.25 mm, ignore below this

    // Calculate magnitude of translational shear displacement
    double u_magnitude = std::sqrt(ux * ux + uy * uy);

    // If displacement is below noise threshold, no additional force needed
    if (u_magnitude <= SLIP_NOISE_THRESHOLD || F_tangential < EPSILON) {
        return 0.0;
    }

    // Compute contact model constant A
    double n = static_cast<double>(config_.n);
    double A = ((n + 1.0) / (2.0 * n)) *
               (1.0 / (config_.E_star * config_.C_n * std::pow(config_.lambda_n, n)));

    if (A < EPSILON) {
        return 0.0;  // Invalid parameters
    }

    // Calculate required normal force from equation (12)
    // F_N = (F_tangential / (G* * u_magnitude))^{n+1} / A
    double denominator = config_.G_star * (u_magnitude / 1000.0);  // Convert mm to m

    if (std::abs(denominator) < EPSILON) {
        return 0.0;
    }

    double ratio = F_tangential / denominator;

    if (ratio <= 0.0) {
        return 0.0;  // Physically invalid or opposing direction
    }

    double power = n + 1.0;
    double F_N_required = std::pow(ratio, power) / A;

    // Validate result
    if (!std::isfinite(F_N_required) || F_N_required < 0.0) {
        return 0.0;
    }

    return F_N_required;
}

double ObjectStateEstimator::calculateRotationalForce(double T_theta, double u_theta) {
    // Based on equation (14) from Narita et al., ICRA 2020:
    // u_theta = 3*T_theta / (2*G* * K_r * F_N^{2/(n+1)})
    // where K_r uses the same constant as K_t but with different exponent
    //
    // Rearranged to solve for F_N:
    // F_N = (3*T_theta / (2*G* * u_theta))^{(n+1)/2} / B
    // where B = (n+1)/(2n) * 1/(E* * C_n * lambda_n^n)  [same as A]

    constexpr double EPSILON = 1e-12;
    constexpr double SLIP_NOISE_THRESHOLD = 0.25e-3;  // 0.25 mm or rad, ignore below this

    // If rotational displacement is below noise threshold, no additional force needed
    if (std::abs(u_theta) <= SLIP_NOISE_THRESHOLD || std::abs(T_theta) < EPSILON) {
        return 0.0;
    }

    // Compute contact model constant B (same as A from translational case)
    double n = static_cast<double>(config_.n);
    double B = ((n + 1.0) / (2.0 * n)) *
               (1.0 / (config_.E_star * config_.C_n * std::pow(config_.lambda_n, n)));

    if (B < EPSILON) {
        return 0.0;  // Invalid parameters
    }

    // Calculate required normal force from equation (14)
    // F_N = (3*T_theta / (2*G* * u_theta))^{(n+1)/2} / B
    double denominator = 2.0 * config_.G_star * u_theta;

    if (std::abs(denominator) < EPSILON) {
        return 0.0;
    }

    double ratio = (3.0 * T_theta) / denominator;

    if (ratio <= 0.0) {
        return 0.0;  // Physically invalid or opposing direction
    }

    double power = (n + 1.0) / 2.0;
    double F_N_required = std::pow(ratio, power) / B;

    // Validate result
    if (!std::isfinite(F_N_required) || F_N_required < 0.0) {
        return 0.0;
    }

    return F_N_required;
}

double ObjectStateEstimator::calculatePIDFeedback(
    double ux, double uy, double u_theta, double dt)
{
    // PID feedback control to drive shear displacements to zero
    // This corrects for modeling errors in the feedforward physics equations
    //
    // The PID acts on the displacement error:
    // - Error = measured displacement (since goal is zero displacement)
    // - Negative sign: we want to oppose the displacement

    // Translational PID: control x and y displacements independently
    double feedback_x = (*pid_translational_x_)(-ux, dt);  // Negative: drive to zero
    double feedback_y = (*pid_translational_y_)(-uy, dt);

    // Combine translational feedback components (vector magnitude)
    double feedback_trans = std::sqrt(feedback_x * feedback_x + feedback_y * feedback_y);

    // Rotational PID: control angular displacement
    double feedback_rot = std::abs((*pid_rotational_)(-u_theta, dt));

    // Total feedback force: combine both modes
    // Use max instead of sum to avoid over-correction
    double feedback_total = std::max(feedback_trans, feedback_rot);

    return feedback_total;
}

bool ObjectStateEstimator::hasBothContacts(
    const TactileData& tactile0,
    const TactileData& tactile1) const
{
    return (tactile0.contact_state >= TactileData::FEW_CONTACTS) &&
           (tactile1.contact_state >= TactileData::FEW_CONTACTS);
}

}  // namespace plato2_state_estimator
