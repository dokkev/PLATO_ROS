#ifndef PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO2_HAND_HPP_

namespace plato2_hand{

class ActuatorGains{
public:
    ActuatorGains() = default;

    std::vector<unit_32t> kp_velocity
    std::vector<unit_32t> ki_velocity

    std::vector<unit_32t> kp_position
    std::vector<unit_32t> ki_position
    std::vector<unit_32t> kd_position

}

class FiveBarLinkage{

}

class PLATO2Hand{

}

} // namespace plato2_hand

#endif // PLATO_HARDWARE_INTERFACE__PCAN_INTERFACE_HPP_