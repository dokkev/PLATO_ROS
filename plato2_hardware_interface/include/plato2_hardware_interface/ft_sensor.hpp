#ifndef PLATO_HARDWARE_INTERFACE__FT_SENSOR_HPP_
#define PLATO_HARDWARE_INTERFACE__FT_SENSOR_HPP_

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/can_protocol.hpp"

#include <Eigen/Dense>
#include <iostream>

namespace sensor {

struct States {
    Eigen::Vector3f force_raw;
    Eigen::Vector3f torque_raw;
    Eigen::Vector3f force_filtered;
    Eigen::Vector3f torque_filtered;
};
    
struct Config {
    const uint32_t force_rx_id;
    const uint32_t torque_rx_id;
};

class FTSensor {
public:
    FTSensor(pcan_interface::PCANInterface &pcan_interface, const Config &config);
    ~FTSensor();
    
    void apply_low_pass_filter(States &states);
    void process_message(const TPCANMsg &msg);
    void calibrate_bias();
    void update_bias();
    
    States get_states() const { return states_; }
    uint get_force_rx_id() const { return config_.force_rx_id; }
    uint get_torque_rx_id() const { return config_.torque_rx_id; }

private:
    pcan_interface::PCANInterface &pcan_interface_;
    Config config_;             // Must be initialized first
    const float alpha_;         // Move this after config_

    States states_;
    
    Eigen::Vector3f bias_force_;
    Eigen::Vector3f bias_torque_;
    Eigen::Vector3f bias_accum_force_;
    Eigen::Vector3f bias_accum_torque_;
    
    int bias_samples_;
    int calibration_samples_;  // **Add this variable**
    bool bias_calibrated_;
};
    

} // namespace sensor

#endif // PLATO_HARDWARE_INTERFACE__FT_SENSOR_HPP_