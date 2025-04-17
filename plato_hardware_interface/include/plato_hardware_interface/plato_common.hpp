#ifndef PLATO_HARDWARE_INTERFACE__PLATO_COMMON_HPP_
#define PLATO_HARDWARE_INTERFACE__PLATO_COMMON_HPP_

// CONSTANTS
#define PI 3.1415926535897932384626433832795

// Motor TX IDs for CAN (Motor Effort)
#define ESP0_CAN_TX_ID 0x10
#define ESP1_CAN_TX_ID 0x11
#define ESP2_CAN_TX_ID 0x12


// Motor RX IDs for CAN (Encoder Angles)
#define ESP0_CAN_RX_ID 0x0A
#define ESP1_CAN_RX_ID 0x0B
#define ESP2_CAN_RX_ID 0x0C

// CAN MSG ENCODING/DECODING CONSTANTS
#define ENCODER_MAX_SCALE_VALUE 3.2767
#define ENCODER_MIN_SCALE_VALUE -3.2768
#define COMMAND_MAX_SCALE_VALUE 3.2767
#define COMMAND_MIN_SCALE_VALUE -3.2768
#define SCALE_INT 65535

// MOTOR MAXIMUM CURRENT
#define MAX_CURRENT 0.8


// MOTOR ANGLE OFFSET (if thehta > pi, thetha = 2pi - theta)
#define MOTOR_0_ANGLE_OFFSET 0.0
#define MOTOR_1_ANGLE_OFFSET 0.0
#define MOTOR_2_ANGLE_OFFSET 0.0
#define MOTOR_3_ANGLE_OFFSET 0.0
#define MOTOR_4_ANGLE_OFFSET 0.0
#define MOTOR_5_ANGLE_OFFSET 0.0
#define MOTOR_6_ANGLE_OFFSET 0.0
#define MOTOR_7_ANGLE_OFFSET 0.0
#define MOTOR_8_ANGLE_OFFSET 0.0

// MOTOR DIRECTION IS CW
#define MOTOR_0_DIRECTION true
#define MOTOR_1_DIRECTION false
#define MOTOR_2_DIRECTION true
#define MOTOR_3_DIRECTION false
#define MOTOR_4_DIRECTION false
#define MOTOR_5_DIRECTION true
#define MOTOR_6_DIRECTION false
#define MOTOR_7_DIRECTION false
#define MOTOR_8_DIRECTION true

// MOTOR REDUCTION RATIO
#define MOTOR_0_REDUCTION_RATIO 1.0
#define MOTOR_1_REDUCTION_RATIO 1.0
#define MOTOR_2_REDUCTION_RATIO 1.29
#define MOTOR_3_REDUCTION_RATIO 1.0
#define MOTOR_4_REDUCTION_RATIO 1.0
#define MOTOR_5_REDUCTION_RATIO 1.29
#define MOTOR_6_REDUCTION_RATIO 1.0
#define MOTOR_7_REDUCTION_RATIO 1.0
#define MOTOR_8_REDUCTION_RATIO 1.29


// DEBUG MODE
// #define DEBUG_MODE

constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-3) {
  if (((d1 - d2) < epsilon) && ((d1 - d2) > -epsilon)) {
    return true;
  } else {
    return false;
  }
}

constexpr bool almost_zero(double d, double epsilon = 1.0e-3) {
  if ((d < epsilon) && (d > -epsilon)) {
    return true;
  } else {
    return false;
  }
}

#endif // PLATO_HARDWARE_INTERFACE__PLATO_COMMON_HPP_