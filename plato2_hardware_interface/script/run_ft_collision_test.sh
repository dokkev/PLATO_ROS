#!/bin/bash

# FT Collision Test - Quick Start Script
# This script helps you quickly launch the collision detection test

echo "================================================"
echo "  PLATO2 FT Collision Detection Test"
echo "================================================"
echo ""

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 not sourced. Please run:"
    echo "  source /opt/ros/<distro>/setup.bash"
    echo "  source /home/optimo/CODE/plato_ws/install/setup.bash"
    exit 1
fi

# Default parameters
VELOCITY=${1:-0.1}
FORCE_THRESHOLD=${2:-2.0}
TORQUE_THRESHOLD=${3:-0.1}
AUTO_START=${4:-false}

echo "Launch Parameters:"
echo "  Joint 4 Velocity:    $VELOCITY rad/s"
echo "  Force Threshold:     $FORCE_THRESHOLD N"
echo "  Torque Threshold:    $TORQUE_THRESHOLD Nm"
echo "  Auto Start:          $AUTO_START"
echo ""
echo "Usage: $0 [velocity] [force_threshold] [torque_threshold] [auto_start]"
echo "Example: $0 0.15 3.0 0.15 true"
echo ""
echo "Starting in 3 seconds... (Ctrl+C to cancel)"
sleep 3

# Launch the node
ros2 launch plato2_hardware_interface ft_collision_test.launch.py \
    joint4_velocity:=$VELOCITY \
    force_threshold:=$FORCE_THRESHOLD \
    torque_threshold:=$TORQUE_THRESHOLD \
    auto_start:=$AUTO_START
