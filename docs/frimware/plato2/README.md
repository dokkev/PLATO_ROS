# PLATO2 Firmware Notes

This directory contains the current PLATO2 firmware draft for the microcontroller
that bridges CAN commands to Dynamixel servos.

The directory is named `frimware` in the existing tree. Keep the path stable
unless a dedicated rename task updates references.

## Protocol Shape

One MCU manages multiple Dynamixel servos. The CAN frame ID identifies the MCU
board. The first payload byte identifies the command, and the second byte
identifies the target servo.

Command frame layout:

```text
DATA[0] = command
DATA[1] = servo_id
DATA[2..] = command payload
```

Response frame layout:

```text
DATA[0] = echoed command
DATA[1] = servo_id
DATA[2] = result code
DATA[3..] = response payload
```

Result codes:

```text
0x00 = success
0x01 = failure
0x02 = motor disabled, for commands that require torque/enable state
```

## Lifecycle Commands

Enable a servo:

```text
DATA[0] = 0x91
DATA[1] = servo_id
```

Disable a servo:

```text
DATA[0] = 0x92
DATA[1] = servo_id
```

Lifecycle response:

```text
DATA[0] = 0x91 or 0x92
DATA[1] = servo_id
DATA[2] = result code
```

## Position Command

Command:

```text
DATA[0] = 0x95
DATA[1] = servo_id
DATA[2..5] = float32 goal position, little-endian, radians
DATA[6..7] = uint16 current limit, little-endian, milliamps
```

Response:

```text
DATA[0] = 0x95
DATA[1] = servo_id
DATA[2] = result code
DATA[3..4] = uint16 packed position
DATA[5..6] = packed velocity bits
DATA[6..7] = packed torque bits
```

Note: the current draft response layout overlaps byte 6 between velocity and
torque. Check `CANProtocol.cpp` and firmware behavior before treating this as a
stable wire contract.

## Source Files

| File | Role |
| --- | --- |
| `plato2.ino` | Arduino entrypoint. |
| `CANInterface.*` | CAN device setup and frame IO. |
| `CANProtocol.*` | Command decoding and response packing. |
| `Actuator.*` | Servo command/state wrapper and joint limit handling. |
| `Configs.h` | Board, servo, and limit constants. |

## Validation Notes

- Validate byte layout with `candump can0` before updating ROS-side protocol
  code.
- Keep command limits conservative while testing firmware.
- Update this README whenever `CANProtocol.*` changes the wire format.
