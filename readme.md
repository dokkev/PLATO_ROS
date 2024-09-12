# PLATO ROS 2 PACKAGE

Welcome to PLATO ROS 2 Project Documentation! This readme file will guide you through the installation and usage of the PLATO ROS 2 package.




## Installation

#### ROS 2 Installation

This package uses ros `humble` distribution with Ubuntu 22.04. To install, follow the instructions on the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

#### Depedencies

<!-- TODO: Change this into rosdep install with proper package.xml -->

```
sudo apt install ros-humble-hardware-interface ros-humble-can-msgs ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-position-controllers
```

#### Building the Packages

Create a workspace and source folder (Assuming you are creating the workspace in your home directory).
```
mkdir -p ~/plato_ws/src
cd ~/plato_ws/src
```
Clone the package into your workspace source folder

```
git clone https://github.com/dokkev/PLATO_ROS.git 
```
Build it using colcon. `symlink-install` creates symbolic links to install directory so that you can edit the files in the source directory and the changes will be reflected in the install directory.
```
cd ~/plato_ws && colcon build --symlink-install
```

<!-- Note -->
> You may get error because CMakeLists.txt looks for empty directories which are not tracked by git. You can either manully create the directories or modify the `install(DIRECTORY DESTINATION "EMPTYDIRNAME" share/${PROJECT_NAME})` part of the CMakeLists.txt to remove your "EMPTYDIRNAME" directory. 

and source the workspace
```
source ~/plato_ws/install/setup.bash
```
The command can be added into your `~/.bashrc` file to avoid sourcing the workspace every time you open a new terminal.

## Packages Overview

- [`optiman_description`](https://github.com/dokkev/PLATO_ROS/tree/master/optiman_description) Contains the URDF files of the Roboligent's Optimo Robot. Note that its called "optim**a**n" to avoid conflict with [Roboligent SDK](https://roboligent.bitbucket.io/) ROS 2 Package called `optimo_description`. 
- [`plato_description`](https://github.com/dokkev/PLATO_ROS/tree/master/plato_description) includes the URDF files and ros2_control parameters of the PLATO Hand V1 and combined with the Optimo Arm (called PLATO Manipulator).
- [`plato_gazebo`](https://github.com/dokkev/PLATO_ROS/tree/master/plato_gazebo) contains the Gazebo Classic simulation files for the PLATO Hand V1 and PLATO Manipulator. Gazebo sim utilizes [`gazebo_ros2_control`](https://github.com/ros-controls/gazebo_ros2_control) to integrate ros2_control architecture with Gazebo.
- [`plato_hardware_interface`](https://github.com/dokkev/PLATO_ROS/tree/master/plato_hardware_interface) contains the ros2_control hardware interface including SocketCAN interface with pre-defined CAN IDs and parameters for motor direction, offset, and pulley reduction ratios for the PLATO Hand V1. 

## Getting Started

#### Visualize the URDF with Rviz
```
ros2 launch plato_description plato_hand_rviz.launch.py 
```
This launch file will open Rviz with the PLATO Hand V1 URDF model with joint state publisher (GUI) and robot state publisher.
![alt text](/docs/img/image.png)


#### Run the PLATO Hand V1 Hardware

Make sure to setup the CAN bus on your system based on your CAN hardware.

>[How to setup your USB-CAN Interface with SocketCAN](https://github.com/dokkev/PLATO_ROS/blob/master/docs/usbcan_setup.md)

Connect the CANH and CANL wires to the CAN bus of the PLATO Hand V1 and set the CAN state to `UP`:

```
sudo ip link set up can0 type can
```

![alt text](/docs/img/plato_connect.png)

There are 12 V DC voltage regulators inside, use the VIN int the range of 12 V to 48 V. Optimo provides 48 V DC with 7 A current, so make sure that voltage regulator are installed properly.

Upon powering the PLATO Hand, the ESP32 and SimpeFOC mini boards will light up and start transmitting motor encoder data over the CAN bus with specified CAN IDs and pre-defined sampling rate defined in the [`Common.h` file in `plato_firmware`](https://bitbucket.org/roboligent/plato_firmware/src/4ed6533804259f32181148ceb040b2cf5c70dc8a/main/Common.h?at=feature%2FfastCAN)

>[Flashing and Understanding the PLATO Hand V1 Firmware](https://github.com/dokkev/sony_plato/wiki/PLATO-Hand-V1-Firmware)

If you didn't make any changes to the firmware, the default CAN IDs are as follows: 

| CAN ID | Device  |
|--------|--------------|
| 0x0A   | ESP0 (Thumb) |   
| 0x0B   | ESP1 (Index) |  
| 0x0C   | ESP2 (pinky) |   

Verify that PLATO Hand is connected to the CAN bus by running the following command:
```
candump can0
```
You should see the some messages like the following:
![alt text](/docs/img/plato_candump.png)

<!-- >[Understanding PLATO Hardware Interface]() -->


Launch the PLATO Hand V1 Hardware Interface via ros2 launch command:

```
ros2 launch plato_hardware_interface plato_hardware.launch.py
```
You should see that `joint_states` are being updated in Rviz when you move the fingers.

![alt text](/docs/img/plato_rviz.gif)


If `joint_states` are not being properly updated, run `rqt_graph` to visualize the ros2 topics and nodes.

```
ros2 run rqt_graph rqt_graph 
```
![alt text](/docs/img/rqt_graph.png)



Verify that the hardware interface is sending commands over the CAN bus by running the following command:
```
candump can0
```
![alt text](/docs/img/hardware_candump.png)

Note that `0x10`, `0x11`, and `0x12` are the default CAN IDs to command the motors.

Upon the sucessful launch of the hardware interface, you can control the PLATO Hand via `ros2 topic`. The default controller is [`position_controllers`](https://control.ros.org/iron/doc/ros2_controllers/position_controllers/doc/userdoc.html) from `ros2_controllers` package.

