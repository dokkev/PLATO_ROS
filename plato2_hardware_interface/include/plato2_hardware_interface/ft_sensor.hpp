#ifndef FTSENSOR_HPP
#define FTSENSOR_HPP

#include "plato2_hardware_interface/pcan_interface.hpp"
#include "plato2_hardware_interface/can_protocol.hpp"

#include <vector>
#include <iostream>

namespace sensor {


struct States {
    float force_x_raw;
    float force_y_raw;
    float force_z_raw;

    float torque_x_raw;
    float torque_y_raw;
    float torque_z_raw;

    float force_x_filtered;
    float force_y_filtered;
    float force_z_filtered;

    float torque_x_filtered;
    float torque_y_filtered;
    float torque_z_filtered;
};
    
struct Config {
    const uint32_t force_rx_id;
    const uint32_t torque_rx_id;
};

class FTSensor {
public:
    /// @brief Constructor
    FTSensor(pcan_interface::PCANInterface &pcan_interface, Config &config);

    /// @brief Process received CAN message
    void process_message(const TPCANMsg &msg);


private:
    pcan_interface::PCANInterface &pcan_interface_;
    Config config_;
    can_protocol::MsgDecoder decoder_;
};

} // namespace sensor

#endif // FTSENSOR_HPP
