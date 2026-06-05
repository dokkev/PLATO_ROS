# USB-CAN Setup

This runbook configures a USB-CAN adapter for SocketCAN on Linux. Use it before
launching hardware interfaces that expect `can0`.

Reference video: [Getting started with SocketCAN (can-utils)](https://www.youtube.com/watch?v=my-mBFQCIZ0&ab_channel=TheEVEngineer)

## Install Tools

```bash
sudo apt install can-utils
```

## Option 1: SLCAN Firmware

SLCAN is the common setup for CANable-style adapters that appear as
`/dev/ttyACM*`.

Connect the USB-CAN adapter and find the device:

```bash
ls /dev/ttyACM*
```

### Optional Static Device Name

`ttyACM*` numbers can change after unplug/replug. A udev symlink makes launch
and shell commands repeatable.

Find the vendor and product IDs:

```bash
lsusb
```

Example output:

```text
Bus 001 Device 042: ID 16d0:117e MCS CANable2 b158aa7 GitHub - normaldotcom/canable2-fw
```

Create a udev rule:

```bash
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```

Replace the IDs with the values from `lsusb`:

```text
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", SYMLINK+="ttycan"
```

Reload rules, then unplug and reconnect the adapter:

```bash
sudo udevadm control --reload-rules
ls -l /dev/ttycan
```

The symlink should point to the current `/dev/ttyACM*` device.

### Bring Up can0

For a udev symlink:

```bash
sudo slcand -o -c -s8 /dev/ttycan can0
```

Without a symlink:

```bash
sudo slcand -o -c -s8 /dev/ttyACM0 can0
```

`-s8` selects 1 Mbps. On some real-time kernels the bitrate must be set in
`slcand`; setting bitrate later with `ip link` may fail with
`RTNETLINK answers: Operation not supported`.

Bring the interface up:

```bash
ip link ls
sudo ip link set up can0 type can
sudo ip link set can0 txqueuelen 1000
ip -details link show can0
```

`state UNKNOWN` can be normal for CAN interfaces.

## Option 2: Candlelight Firmware

With Candlelight firmware, `can0` should appear directly in `ip link`.

```bash
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
ip -details link show can0
```

For isolated adapter debugging, enable loopback while the interface is down:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can loopback on
sudo ip link set can0 up
```

Disable loopback the same way:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can loopback off
sudo ip link set can0 up
```

## Test The Bus

Terminal 1:

```bash
candump can0
```

Terminal 2:

```bash
cansend can0 123#1122334455667788
```

The frame should appear in `candump` if the interface and bus are working.

## Shut Down

```bash
sudo ip link set can0 down
```

If using `slcand`, stop the corresponding process when done:

```bash
pgrep -a slcand
sudo pkill slcand
```

## Safety Notes

- Confirm wiring, bus termination, bitrate, and motor power state before
  launching hardware nodes.
- Do not send arbitrary frames on a live robot bus.
- Prefer `candump` observation before enabling actuators.
- Keep hardware launch commands in `docs/COMMANDS.md` and operator command
  examples in `cli_commands.md`.
