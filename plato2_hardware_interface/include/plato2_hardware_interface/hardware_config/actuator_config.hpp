#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_

// TODO: Change this to yaml file

#include <cstdint>


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
    constexpr float TORQUE_CONSTANT = 0.41f / 8.0f;
} // namespace GIM3505

namespace XM430{
    constexpr float GEAR_RATIO = 350.0f;
    constexpr float TORQUE_CONSTANT = 1.17f;
} // namespace XM430W

namespace MotorOffset{

    constexpr float MOTOR1 = 0.0f;
    constexpr float MOTOR2 = 0.0f;

    constexpr float MOTOR4 = 3.11341f;
    constexpr float MOTOR3 = 1.33535f;

    constexpr float MOTOR6 = 2.78306f;
    constexpr float MOTOR5 = -3.8714f;
   
    constexpr float MOTOR8 = -1.005f;
    constexpr float MOTOR7 = -2.73995f;
    
}

namespace MotorDirection{
    constexpr char MOTOR1 = 1;
    constexpr char MOTOR2 = 1;
    constexpr char MOTOR3 = -1;
    constexpr char MOTOR4 = -1;
    constexpr char MOTOR5 = -1;
    constexpr char MOTOR6 = -1;
    constexpr char MOTOR7 = 1;
    constexpr char MOTOR8 = 1;
}


#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_