#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_

// TODO: Change this to yaml file

#include <cstdint>
#include <plato2_hardware_interface/utils/useful_functions.hpp>


using namespace Plato;

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
    constexpr float TORQUE_CONSTANT = 0.41f;
} // namespace GIM3505

namespace XM430{
    constexpr float GEAR_RATIO = 350.0f;
    constexpr float TORQUE_CONSTANT = 1.17f;
} // namespace XM430W

namespace MotorOffset{

    constexpr float MOTOR1 = 0.0f;
    constexpr float MOTOR2 = 0.0f;

    constexpr float MOTOR4 = -2.35466f;
    constexpr float MOTOR3 = 0.532731f;

    constexpr float MOTOR6 = 0.918403f;
    constexpr float MOTOR5 = -4.27386f;
   
    constexpr float MOTOR8 = 4.64733f;
    constexpr float MOTOR7 = -2.10441f;
    
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

namespace MotorJointLimit{
    constexpr float XM430_MAX = 3.14159f; // joint limits of XM430 are handled inside OpenRB-150 frimware
    constexpr float XM430_MIN = -3.14159f;
    constexpr float MCP_MAX = deg2rad(60.0f);
    constexpr float MCP_MIN = deg2rad(-60.0f);
    constexpr float PIP_MAX = deg2rad(130.0f);
    constexpr float PIP_MIN = deg2rad(-45.0f);
}

#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_