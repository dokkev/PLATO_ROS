#ifndef PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
#define PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_

// TODO: Change this to yaml file

#include <cstdint>
#include <plato_hardware_interface/utils/useful_functions.hpp>

// Motor
// white CANH 
// yellow CANL
// black RS458 A
// red RS485 B

// gray d-usb color
// red: can-0 low
// blue: can-0 high
// brown: can-1 high
// green: can-1 low
// orange/yellow: V+
// purple/black: GND
using namespace Plato;

// CAN IDs for motors
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


// not used for Aristo
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

namespace JointPositionLimit{
    constexpr float THUMB_ROLL_MAX = deg2rad(45.0f);
    constexpr float THUMB_ROLL_MIN = deg2rad(-15.0f);
    constexpr float THUMB_YAW_MAX = deg2rad(45.0f);
    constexpr float THUMB_YAW_MIN = deg2rad(-45.0f);
    constexpr float MCP_MAX = deg2rad(70.0f);
    constexpr float MCP_MIN = deg2rad(-60.0f);
    constexpr float PIP_MAX = deg2rad(180.0f);
    constexpr float PIP_MIN = deg2rad(-60.0f);
}

namespace JointVelocityLimit{
    constexpr float THUMB_ROLL = 40.21f; // rad/s
    constexpr float THUMB_YAW = 40.21f;  // rad/s
    constexpr float MCP = 40.21f;        // rad/s
    constexpr float PIP = 40.21f;        // rad/s
}

namespace JointEffortLimit{
    constexpr float THUMB_ROLL = GIM3505::PEAK_TORQUE; // Nm
    constexpr float THUMB_YAW = GIM3505::PEAK_TORQUE;  // Nm
    constexpr float MCP = GIM3505::PEAK_TORQUE;        // Nm
    constexpr float PIP = GIM3505::PEAK_TORQUE;        // Nm
}

namespace JointStiffnessLimit{
    constexpr float THUMB_ROLL = 5.0f; // Nm/rad
    constexpr float THUMB_YAW = 5.0f;  // Nm/rad
    constexpr float MCP = 5.0f;        // Nm/rad
    constexpr float PIP = 5.0f;        // Nm/rad
}

namespace JointDampingLimit{
    constexpr float THUMB_ROLL = 2.5f; // Nms/rad
    constexpr float THUMB_YAW = 2.5f;  // Nms/rad
    constexpr float MCP = 2.0f;        // Nms/rad
    constexpr float PIP = 2.0f;        // Nms/rad
}

#endif // PLATO_HARDWARE_INTERFACE__ACTUATOR_CONFIG_HPP_
