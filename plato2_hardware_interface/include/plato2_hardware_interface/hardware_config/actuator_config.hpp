#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_

// TODO: Change this to yaml file

#include <cstdint>
#include <plato2_hardware_interface/utils/useful_functions.hpp>


// white CANH 
// yellow CANL

using namespace Plato;

namespace MotorTxID{
    constexpr uint8_t MOTOR1 = 0x0A; // 10
    constexpr uint8_t MOTOR2 = 0x0B; // 11
    constexpr uint8_t MOTOR3 = 0x0C; // 12
    constexpr uint8_t MOTOR4 = 0x0D; // 13
    constexpr uint8_t MOTOR5 = 0x0E; // 14
    constexpr uint8_t MOTOR6 = 0x0F; // 15
    constexpr uint8_t MOTOR7 = 0x10; // 16
    constexpr uint8_t MOTOR8 = 0x11; // 17
} // namespace MotorTxID

namespace MotorRxID{
    constexpr uint8_t MOTOR1 = 0x0A; // 10
    constexpr uint8_t MOTOR2 = 0x0B; // 11
    constexpr uint8_t MOTOR3 = 0x0C; // 12
    constexpr uint8_t MOTOR4 = 0x0D; // 13
    constexpr uint8_t MOTOR5 = 0x0E; // 14
    constexpr uint8_t MOTOR6 = 0x0F; // 15
    constexpr uint8_t MOTOR7 = 0x10; // 16
    constexpr uint8_t MOTOR8 = 0x11; // 17
} // namespace MotorRxID



// namespace CanMsgLimits{
//     constexpr float Pos_Max = 12.566f; // rad (4pi)
//     constexpr float Vel_Max = 52.36f;   // rad/s (500 rpm)
//     constexpr float T_Max   = 2.08f;   // Nm (4 Amp)
// } // namespace CanMsgLimits

namespace GIM3505{
    constexpr float GEAR_RATIO = 8.0f;
    constexpr float TORQUE_CONSTANT = 0.52;
    constexpr float PEAK_TORQUE = 1.27f; // Nm 
} // namespace GIM3505

namespace MotorOffset{

    constexpr float MOTOR1 = 0.0f; // THUMB CMC ROLL
    constexpr float MOTOR2 = 0.0f; // THUMB MCP YAW
    constexpr float MOTOR3 = 0.0f; // THUMB MCP PITCH
    constexpr float MOTOR4 = 0.0f; // THUMB PIP PITCH
    constexpr float MOTOR5 = 0.0f; // INDEX MCP PITCH
    constexpr float MOTOR6 = 0.0f; // INDEX PIP PITCH
    constexpr float MOTOR7 = 0.0f; // MIDDLE MCP PITCH
    constexpr float MOTOR8 = 0.0f; // MIDDLE PIP PITCH

}

namespace MotorDirection{
    constexpr char MOTOR1 = -1; //THUMB ROLL
    constexpr char MOTOR2 = 1; //THUMB YAW
    constexpr char MOTOR3 = -1; //THUMB MCP
    constexpr char MOTOR4 = -1; //THUMB PIP
    constexpr char MOTOR5 = -1; //INDEX MCP
    constexpr char MOTOR6 = -1; //INDEX PIP
    constexpr char MOTOR7 = 1; //MIDDLE MCP
    constexpr char MOTOR8 = 1; //MIDDLE PIP
}

namespace MotorJointLimit{
    constexpr float THUMB_ROLL_MAX = deg2rad(45.0f);
    constexpr float THUMB_ROLL_MIN = deg2rad(-45.0f);
    constexpr float THUMB_YAW_MAX = deg2rad(45.0f);
    constexpr float THUMB_YAW_MIN = deg2rad(-45.0f);
    constexpr float MCP_MAX = deg2rad(60.0f);
    constexpr float MCP_MIN = deg2rad(-60.0f);
    constexpr float PIP_MAX = deg2rad(120.0f);
    constexpr float PIP_MIN = deg2rad(-60.0f);
}

#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_