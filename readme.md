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

## Packages Overview
<list>
`optiman_description` - Contains the URDF files of the Roboligent's Optimo Robot. Note that its called "optim**a**n" to avoid conflict with [Roboligent SDK](https://roboligent.bitbucket.io/)'s ROS 2 Package called `optimo_description`. 
- 

## Getting Started

#### 




```mermaid
flowchart TB
```

