#ifndef PLATO2_STATE_ESTIMATOR_OBJECT_STATE_ESTIMATOR_HPP
#define PLATO2_STATE_ESTIMATOR_OBJECT_STATE_ESTIMATOR_HPP

#include <plato_utils/pid_controller.hpp>
#include <memory>
#include <cmath>

namespace plato2_state_estimator {

/**
 * @brief Slip state enumeration
 */
enum class SlipState {
    NO_CONTACT,          ///< No contact detected
    PARTIAL_CONTACT,     ///< Contact from only one sensor
    STABLE_GRASP,        ///< Both sensors in contact, no slip
    TRANSLATIONAL_SLIP,  ///< Translational slip detected
    ROTATIONAL_SLIP,     ///< Rotational slip detected
    COMBINED_SLIP        ///< Both translational and rotational slip
};

/**
 * @brief Tactile sensor data structure (ROS-independent)
 */
struct TactileData {
    // Contact state
    enum ContactState {
        NO_CONTACT = 0,
        FEW_CONTACTS = 1,
        ENOUGH_CONTACTS = 2
    };

    int32_t contact_state;

    // Shear displacement [mm, mm, rad]
    double shear_x;
    double shear_y;
    double shear_theta;

    // Forces [N]
    double force_x;
    double force_y;
    double force_z;

    // Timestamp (seconds)
    double timestamp;

    TactileData()
        : contact_state(NO_CONTACT)
        , shear_x(0.0), shear_y(0.0), shear_theta(0.0)
        , force_x(0.0), force_y(0.0), force_z(0.0)
        , timestamp(0.0)
    {}
};

/**
 * @brief Configuration parameters for ObjectStateEstimator
 */
struct ObjectStateEstimatorConfig {
    // Material properties
    double E_star;        ///< Effective Young's modulus [Pa]
    double G_star;        ///< Transverse elastic modulus [Pa]
    double C_n;           ///< Contact surface constant
    double lambda_n;      ///< Scaling factor for 3D contact
    int n;                ///< Order of contact surface (typically 2)

    // Slip detection thresholds
    double translational_slip_threshold;  ///< [mm]
    double rotational_slip_threshold;     ///< [rad]

    // Control parameters
    double min_contact_force;  ///< Minimum contact force [N]
    double max_force_limit;    ///< Maximum force limit for safety [N]

    // PID feedback gains for slip correction
    double pid_tx_p, pid_tx_i, pid_tx_d;      ///< Translational PID [P, I, D]
    double pid_theta_p, pid_theta_i, pid_theta_d;  ///< Rotational PID [P, I, D]

    ObjectStateEstimatorConfig()
        : E_star(1.0e6), G_star(0.4e6), C_n(1.0), lambda_n(1.0), n(2)
        , translational_slip_threshold(0.5), rotational_slip_threshold(0.05)
        , min_contact_force(0.5), max_force_limit(30.0)
        , pid_tx_p(1.8), pid_tx_i(0.0), pid_tx_d(4.5)
        , pid_theta_p(30.0), pid_theta_i(0.0), pid_theta_d(90.0)
    {}
};

/**
 * @brief Output data from ObjectStateEstimator
 */
struct ObjectStateEstimatorOutput {
    SlipState slip_state;
    double minimal_force;  ///< [N]
    double force_x;        ///< Tangential force [N] (assumed zero without measurement)
    double force_y;        ///< Tangential force [N] (assumed zero without measurement)
    double moment_z;       ///< Moment [Nm]
    bool has_valid_data;

    ObjectStateEstimatorOutput()
        : slip_state(SlipState::NO_CONTACT)
        , minimal_force(0.0), force_x(0.0), force_y(0.0), moment_z(0.0)
        , has_valid_data(false)
    {}
};

/**
 * @brief Core object state estimator (ROS-independent)
 *
 * Implements slip detection and minimal force calculation based on:
 * "Theoretical Derivation and Realization of Adaptive Grasping Based on
 * Rotational Incipient Slip Detection" (T. Narita et al., ICRA 2020)
 */
class ObjectStateEstimator {
public:
    /**
     * @brief Construct with configuration
     */
    explicit ObjectStateEstimator(const ObjectStateEstimatorConfig& config);

    ~ObjectStateEstimator() = default;

    /**
     * @brief Update with new sensor data
     * @param tactile0 Data from first sensor
     * @param tactile1 Data from second sensor
     * @param dt Time step [seconds]
     * @return Updated state estimate
     */
    ObjectStateEstimatorOutput update(
        const TactileData& tactile0,
        const TactileData& tactile1,
        double dt);

    /**
     * @brief Reset internal state
     */
    void reset();

    /**
     * @brief Get current configuration
     */
    const ObjectStateEstimatorConfig& getConfig() const { return config_; }

    /**
     * @brief Update configuration (will reset PIDs)
     */
    void setConfig(const ObjectStateEstimatorConfig& config);

private:
    // Slip detection
    SlipState detectSlip(const TactileData& tactile0, const TactileData& tactile1);
    bool detectTranslationalSlip(double ux, double uy) const;
    bool detectRotationalSlip(double u_theta) const;

    // Force calculation
    double calculateMinimalForce(
        const TactileData& tactile0,
        const TactileData& tactile1,
        double dt);

    double calculateTranslationalForce(double F_tangential, double ux, double uy);
    double calculateRotationalForce(double T_theta, double u_theta);

    double calculatePIDFeedback(double ux, double uy, double u_theta, double dt);

    // Helper functions
    bool hasBothContacts(const TactileData& tactile0, const TactileData& tactile1) const;

    // Configuration
    ObjectStateEstimatorConfig config_;

    // PID controllers for feedback correction
    std::unique_ptr<plato_utils::PIDController> pid_translational_x_;
    std::unique_ptr<plato_utils::PIDController> pid_translational_y_;
    std::unique_ptr<plato_utils::PIDController> pid_rotational_;

    // State
    ObjectStateEstimatorOutput last_output_;
};

}  // namespace plato2_state_estimator

#endif  // PLATO2_STATE_ESTIMATOR_OBJECT_STATE_ESTIMATOR_HPP
