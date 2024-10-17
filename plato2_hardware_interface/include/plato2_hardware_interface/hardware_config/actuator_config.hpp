#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_

#include "plato2_hardware_interface/actuator.hpp"

// TODO: Change this to yaml file

namespace MotorTxID{
    constexpr uint8_t MOTOR1 = 0x11;
    constexpr uint8_t MOTOR2 = 0x12;
    constexpr uint8_t MOTOR3 = 0x13;
    constexpr uint8_t MOTOR4 = 0x14;
    constexpr uint8_t MOTOR5 = 0x15;
    constexpr uint8_t MOTOR6 = 0x16;
    constexpr uint8_t MOTOR7 = 0x17;
    constexpr uint8_t MOTOR8 = 0x18;
} // namespace MotorTxID

namespace MotorRxID{
    constexpr uint8_t MOTOR1 = 0x21;
    constexpr uint8_t MOTOR2 = 0x22;
    constexpr uint8_t MOTOR3 = 0x23;
    constexpr uint8_t MOTOR4 = 0x24;
    constexpr uint8_t MOTOR5 = 0x25;
    constexpr uint8_t MOTOR6 = 0x26;
    constexpr uint8_t MOTOR7 = 0x27;
    constexpr uint8_t MOTOR8 = 0x28;
} // namespace MotorRxID

namespace GIM3505{
    constexpr float GEAR_RATIO = 8.0f;
    constexpr float TORQUE_CONSTANT = 0.41 / 8.0f;
} // namespace GIM3505

namespace XM430{
    constexpr float GEAR_RATIO = 350.0f;
    constexpr float TORQUE_CONSTANT = 1.17f;
} // namespace XM430

namespace MotorOffset{
    constexpr float MOTOR1 = 0.0f;
    constexpr float MOTOR2 = 0.0f;
    constexpr float MOTOR3 = 0.0f;
    constexpr float MOTOR4 = 0.0f;
    constexpr float MOTOR5 = 0.0f;
    constexpr float MOTOR6 = 0.0f;
    constexpr float MOTOR7 = 0.0f;
    constexpr float MOTOR8 = 0.0f;
}


namespace MotorDirection{
    constexpr char MOTOR1 = -1;
    constexpr char MOTOR2 = -1;
    constexpr char MOTOR3 = 1;
    constexpr char MOTOR4 = 1;
    constexpr char MOTOR5 = 1;
    constexpr char MOTOR6 = 1;
    constexpr char MOTOR7 = -1;
    constexpr char MOTOR8 = -1;
}

namespace PlatoV2Config{

std::array<actuator::Config, 8> init_actuator_configs() {
    return {
        actuator::Config{MotorTxID::MOTOR1, MotorRxID::MOTOR1, MotorOffset::MOTOR1, MotorDirection::MOTOR1, XM430::TORQUE_CONSTANT, XM430::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR2, MotorRxID::MOTOR2, MotorOffset::MOTOR2, MotorDirection::MOTOR2, XM430::TORQUE_CONSTANT, XM430::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR3, MotorRxID::MOTOR3, MotorOffset::MOTOR3, MotorDirection::MOTOR3, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR4, MotorRxID::MOTOR4, MotorOffset::MOTOR4, MotorDirection::MOTOR4, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR5, MotorRxID::MOTOR5, MotorOffset::MOTOR5, MotorDirection::MOTOR5, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR6, MotorRxID::MOTOR6, MotorOffset::MOTOR6, MotorDirection::MOTOR6, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR7, MotorRxID::MOTOR7, MotorOffset::MOTOR7, MotorDirection::MOTOR7, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO},
        actuator::Config{MotorTxID::MOTOR8, MotorRxID::MOTOR8, MotorOffset::MOTOR8, MotorDirection::MOTOR8, GIM3505::TORQUE_CONSTANT, GIM3505::GEAR_RATIO}
    };
}



} // namespace plato_hardware_interface



#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_