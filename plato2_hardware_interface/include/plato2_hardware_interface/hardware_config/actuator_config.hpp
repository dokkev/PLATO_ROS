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


    
    // constexpr float MOTOR4 = -5.73987f;
    // constexpr float MOTOR3 = -5.72614f;
    // constexpr float MOTOR6 = -11.7702f;
    // constexpr float MOTOR5 = -9.29942f;
    // constexpr float MOTOR8 = -5.45224f;
    // constexpr float MOTOR7 = -0.174525f;

}

namespace CommandOffset{
    constexpr float MOTOR1 = 0.0f;
    constexpr float MOTOR2 = 0.0f;
    constexpr float MOTOR3 = 2.84485f;
    constexpr float MOTOR4 = 0.0f;
    constexpr float MOTOR5 = 5.79328f;
    constexpr float MOTOR6 = 5.76429f;
    constexpr float MOTOR7 = -5.8338;
    constexpr float MOTOR8 = -5.84033f;
}

namespace MotorDirection{
    constexpr char MOTOR1 = 1;
    constexpr char MOTOR2 = 1;
    constexpr char MOTOR3 = 1;
    constexpr char MOTOR4 = 1;
    constexpr char MOTOR5 = 1;
    constexpr char MOTOR6 = 1;
    constexpr char MOTOR7 = 1;
    constexpr char MOTOR8 = 1;
}


namespace KpVelocity{
    constexpr uint32_t MOTOR1 = 0;
    constexpr uint32_t MOTOR2 = 0;
    constexpr uint32_t MOTOR3 = 0;
    constexpr uint32_t MOTOR4 = 0;
    constexpr uint32_t MOTOR5 = 0;
    constexpr uint32_t MOTOR6 = 0;
    constexpr uint32_t MOTOR7 = 0;
    constexpr uint32_t MOTOR8 = 0;
} // namespace KpVelocity   

namespace KiVelocity{
    constexpr uint32_t MOTOR1 = 0;
    constexpr uint32_t MOTOR2 = 0;
    constexpr uint32_t MOTOR3 = 0;
    constexpr uint32_t MOTOR4 = 0;
    constexpr uint32_t MOTOR5 = 0;
    constexpr uint32_t MOTOR6 = 0;
    constexpr uint32_t MOTOR7 = 0;
    constexpr uint32_t MOTOR8 = 0;
} // namespace KiVelocity


namespace KpPosition{
    constexpr uint32_t MOTOR1 = 0;
    constexpr uint32_t MOTOR2 = 0;
    constexpr uint32_t MOTOR3 = 10;
    constexpr uint32_t MOTOR4 = 10;
    constexpr uint32_t MOTOR5 = 10;
    constexpr uint32_t MOTOR6 = 10;
    constexpr uint32_t MOTOR7 = 10;
    constexpr uint32_t MOTOR8 = 10;
} // namespace KpPosition

namespace KiPosition{
    constexpr uint32_t MOTOR1 = 0;
    constexpr uint32_t MOTOR2 = 0;
    constexpr uint32_t MOTOR3 = 0;
    constexpr uint32_t MOTOR4 = 0;
    constexpr uint32_t MOTOR5 = 0;
    constexpr uint32_t MOTOR6 = 0;
    constexpr uint32_t MOTOR7 = 0;
    constexpr uint32_t MOTOR8 = 0;
} // namespace KiPosition

namespace KdPosition{
    constexpr uint32_t MOTOR1 = 0;
    constexpr uint32_t MOTOR2 = 0;
    constexpr uint32_t MOTOR3 = 15000;
    constexpr uint32_t MOTOR4 = 15000;
    constexpr uint32_t MOTOR5 = 15000;
    constexpr uint32_t MOTOR6 = 15000;
    constexpr uint32_t MOTOR7 = 15000;
    constexpr uint32_t MOTOR8 = 15000;


} // namespace KdPosition




#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_