#ifndef PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_

#include "plato2_hardware_interface/actuator.hpp"

namespace plato2_hand{


enum class ControlMode {
    OFF,
    POSITION,
    VELOCITY,
    TORQUE
};

class Hand{
public:

    Hand();
    ~Hand();

    void init_actuators();

    void enable();

    void disable();

    void stop();

    void send_command();

    void receive_states();   

    void update();

    void set_control_mode(const ControlMode &control_mode);

private:
    pcan_interface::PCANInterface pcan_interface_;

    std::vector<actuator::Actuator> actuators_;

    ControlMode control_mode_;

};


} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_