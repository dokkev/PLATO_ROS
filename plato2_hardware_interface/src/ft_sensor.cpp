#include "plato2_hardware_interface/ft_sensor.hpp"
#include <iostream>

namespace sensor {

FTSensor::FTSensor(pcan_interface::PCANInterface &pcan_interface, const Config &config)
    : pcan_interface_(pcan_interface),
      config_(config),   
      alpha_(0.06f),
      states_(),
      bias_force_(Eigen::Vector3f::Zero()),
      bias_torque_(Eigen::Vector3f::Zero()),
      bias_accum_force_(Eigen::Vector3f::Zero()),
      bias_accum_torque_(Eigen::Vector3f::Zero()),
      bias_samples_(0),
      calibration_samples_(100), 
      bias_calibrated_(false) 
      {
        std::cout << "FT Sensor Initialized with Force ID: " << config_.force_rx_id 
                  << " Torque ID: " << config_.torque_rx_id << std::endl;
    }



FTSensor::~FTSensor() {
    std::cout << "FTSensor object destroyed." << std::endl;
}

void FTSensor::apply_low_pass_filter(States &states) {
    states.force_filtered = alpha_ * (states.force_raw - bias_force_) + (1 - alpha_) * states.force_filtered;
    states.torque_filtered = alpha_ * (states.torque_raw - bias_torque_) + (1 - alpha_) * states.torque_filtered;
}

void FTSensor::update_bias() {
    if (!bias_calibrated_
        && bias_samples_force_  >= calibration_samples_
        && bias_samples_torque_ >= calibration_samples_) {

        bias_force_  = bias_accum_force_  / static_cast<float>(bias_samples_force_);
        bias_torque_ = bias_accum_torque_ / static_cast<float>(bias_samples_torque_);
        bias_calibrated_ = true;

        std::cout << "Sensor bias calibrated:\n  Force  = "
                  << bias_force_.transpose()
                  << "\n  Torque = "
                  << bias_torque_.transpose() << std::endl;
    }
}

// 3. Re-write process_message() so each accumulator has its own counter ------
//    (the rest of the method is unchanged)
void FTSensor::process_message(const TPCANMsg &msg) {
    if (msg.ID == config_.force_rx_id) {
        can_protocol::MsgDecoder::retrieve_force(
            msg, states_.force_raw.x(), states_.force_raw.y(), states_.force_raw.z());

        if (!bias_calibrated_) {
            bias_accum_force_ += states_.force_raw;
            ++bias_samples_force_;                      // << NEW
            update_bias();
        }
    }
    else if (msg.ID == config_.torque_rx_id) {
        can_protocol::MsgDecoder::retrieve_torque(
            msg, states_.torque_raw.x(), states_.torque_raw.y(), states_.torque_raw.z());

        if (!bias_calibrated_) {
            bias_accum_torque_ += states_.torque_raw;
            ++bias_samples_torque_;                     // << NEW
            update_bias();
        }
    }
    else {
        std::cerr << "Unknown message ID received in FTSensor::process_message\n";
        return;
    }

    // Apply low-pass filter only after calibration is complete
    if (bias_calibrated_) {
        apply_low_pass_filter(states_);
    }
}

} // namespace sensor
