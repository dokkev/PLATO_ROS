#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_

// TODO: Change this to yaml file

#include <cstdint>
#include <plato2_hardware_interface/utils/useful_functions.hpp>


using namespace Plato;

namespace MotorTxID{
    constexpr uint8_t MOTOR1 = 0xa;
    constexpr uint8_t MOTOR2 = 0xb;
    constexpr uint8_t MOTOR3 = 0xc;
    constexpr uint8_t MOTOR4 = 0xd;
    constexpr uint8_t MOTOR5 = 0xe;
    constexpr uint8_t MOTOR6 = 0xf;
    constexpr uint8_t MOTOR7 = 0x10;
    constexpr uint8_t MOTOR8 = 0x11;
} // namespace MotorTxID

namespace MotorRxID{
    constexpr uint8_t MOTOR1 = 0xa;
    constexpr uint8_t MOTOR2 = 0xb;
    constexpr uint8_t MOTOR3 = 0xc;
    constexpr uint8_t MOTOR4 = 0xd;
    constexpr uint8_t MOTOR5 = 0xe;
    constexpr uint8_t MOTOR6 = 0xf;
    constexpr uint8_t MOTOR7 = 0x10;
    constexpr uint8_t MOTOR8 = 0x11;
} // namespace MotorRxID

namespace GIM3505{
    constexpr float GEAR_RATIO = 8.0f;
    constexpr float TORQUE_CONSTANT = 0.41f;
} // namespace GIM3505

namespace MotorOffset{

    constexpr float MOTOR1 = 0.0f; // THUMB ROLL
    constexpr float MOTOR2 = 0.0f; // THUMB YAW
    constexpr float MOTOR3 = 0.0f; // THUMB MCP
    constexpr float MOTOR4 = 0.0f; // THUMB PIP
    constexpr float MOTOR5 = 0.0f; // INDEX MCP
    constexpr float MOTOR6 = 0.0f; // INDEX PIP
    constexpr float MOTOR7 = 0.0f; // MIDDLE MCP
    constexpr float MOTOR8 = 0.0f; // MIDDLE PIP

}

namespace MotorDirection{
    constexpr char MOTOR1 = 1; //THUMB ROLL
    constexpr char MOTOR2 = 1; //THUMB YAW
    constexpr char MOTOR3 = 1; //THUMB MCP
    constexpr char MOTOR4 = -1; //THUMB PIP
    constexpr char MOTOR5 = 1; //INDEX MCP
    constexpr char MOTOR6 = -1; //INDEX PIP
    constexpr char MOTOR7 = 1; //MIDDLE MCP
    constexpr char MOTOR8 = -1; //MIDDLE PIP
}

namespace MotorJointLimit{
    constexpr float THUMB_ROLL_MAX = deg2rad(45.0f);
    constexpr float THUMB_ROLL_MIN = deg2rad(-45.0f);
    constexpr float THUMB_YAW_MAX = deg2rad(90.0f);
    constexpr float THUMB_YAW_MIN = deg2rad(-90.0f);
    constexpr float MCP_MAX = deg2rad(60.0f);
    constexpr float MCP_MIN = deg2rad(-60.0f);
    constexpr float PIP_MAX = deg2rad(120.0f);
    constexpr float PIP_MIN = deg2rad(-60.0f);
}

#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_