# Setting up your USB-CAN Interface

Reference Video: [Getting started with SocketCAN (can-utils)](https://www.youtube.com/watch?v=my-mBFQCIZ0&ab_channel=TheEVEngineer) 

Install  `can-utils`

```sudo apt install can-utils```

Setting up your USB-CAN connection will vary depending on what firmware you have on your USB-CAN converter.  You can update your firmware from https://canable.io/updater/. I used slcan firmware for my USB-CAN converter.


## Option 1: Slcan (Recommended)
connect USB-CAN adaptor to PC

Run the following command:
```
ls /dev/ttyACM*
```

==> The MKS CANable adapter appears as /dev/ttyACMx (an arbitrary number x depending on the number of USB devices connected to the PC)

---
#### **NOTE**: Assign a static name by assigning a symlink (Optional, but recommended)
ttyACM tends to be dynamically allocated whenever it creates CAN driver is ran by slacand command. If you unplug and re-plug in the USB-CAN module, the /dev/ttyACMx value will be changed while x is an arbitrary number

To avoid this inconvenience, create a symlink in /dev using /etc/udev/rules.  Firstly, check USB devices connected to your system:

Run `lsusb` to list all USB devices connected to your system. You will see a list of USB devices connected to your system. Find the USB-CAN device and note the Vendor ID and Product ID. It will return somethi    ng like this:
    
```
==> Bus 001 Device 042: ID 16d0:117e MCS CANable2 b158aa7 GitHub - normaldotcom/canable2-fw

```
`16d0` is your ATTRS{idVendor}, and `117e` is your ATTRS{idProduct}

Run the following command to create a new rule file (use 99 ~ 90 to prevent override):
```
sudo nano /etc/udev/rules.d/99-usb-serial.rules
```
Within the file, add the following line (replace `16d0` and `117e` with your own Vendor ID and Product ID):

```
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="16d0", ATTRS{idProduct}=="117e", SYMLINK+="ttycan"
```
and ctrl + x  and y to save and exit (for me, SYMLINK name had to be all lowercase in order to work).

 

reload the udev/rules.d  by: 
```
sudo udevadm control --reload-rules
```
Unplug and plug back in the USB-CAN module just in case, and run:

```
ls -l /dev/ttycan
```

it should return something like this:
```
==> lrwxrwxrwx 1 root root 7 Sep 12 14:01 /dev/ttycan -> ttyACM5
```
You can see that `ttycan` is linked to `ttyACMx`

Lastly, compare `udevadm info -a /dev/ttyACMx` and `udevadm info -a /dev/ttycan` to make sure they output the same device info.

---
#### Connect CAN driver and Setup CAN chaneel `can-utils`

Run:

```
sudo slcand -o -c -s8 /dev/ttycan can0 (If you had set-up symlink in udev.rules)
```
or if you skipped setting up the symlink, run:
```
sudo slcand -o -c -s8 /dev/ttyACMx can0
``` 

-s8 sets the bitrate 1 Mbps. On a normal kernel, bitrate can be set later, however on the RT kernel this must be set here, as you will get a `RTNETLINK answers: Operation not supported` error. See [Getting Started - CANable](https://canable.io/getting-started.html#socketcan-linux) to see the various bitrates.

then

```
ip link ls
```

You should see: 
```
==> can0: <NOARP> mtu 16 qdisc noop state DOWN mode DEFAULT group default qlen 10
```
Note that can0 is down. To set it to `UP` state, run:

```
sudo ip link set up can0 type can 
```

Confirm by:

```
ip link ls
```
You should see:
```
==> <NOARP,UP,LOWER_UP> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10
    link/can
```
Somethimes, it's shown as `state UNKNOWN` instead of `state UP`. This is normal.

To set buffer:

```
ip link set can0 txqueuelen 1000
```

#### Testing SocketCAN in Terminal
After you set the CAN state to `UP`, run:

```
candump can0
```
And open another terminal and run:

```
cansend can0 123#1122334455667788
```
You will see somethime like this in the terminal where `candump can0` is running:
![alt text](/docs/img/candump.png)


To stop the CAN interface, run:

```
sudo ip link set can0 down
```
## Option 2: Candlelight

`can0` should appear as down state when you run `ip link ls`

To set CAN state to up to 1Mbps:

```sudo ip link set can0 type can bitrate 1000000```


To set buffer to 1000:

```sudo ip link set canX txqueuelen 1000```


<!-- Note -->
> If the USB-CAN device with candlelight firmware is not connected to any other CAN devices (such as ESP32 + CAN Transceiver), it will not process CAN messages and will not show any data in `candump` or `cansniffer` after processing 2~3 messages. In order to debug the isolated USB-CAN device, you have to enable to loopback on

```
sudo ip link set can0 type can loopback on 
```

Note that CAN bus state has be to `DOWN` in order to enable the loopback

To set the loopback off, run:
```
sudo ip link set can0 type can loopback off 
```


 To stop the CAN interface, run:

```sudo ip link set can0 down```