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
VELOCITY=${1:-0.05}
FORCE_THRESHOLD=${2:-0.6}
TORQUE_THRESHOLD=${3:-0.02}
AUTO_START=${4:-false}

echo "Launch Parameters:"
echo "  Joint 5 Velocity:    $VELOCITY rad/s"
echo "  Force Threshold:     $FORCE_THRESHOLD N"
echo "  Torque Threshold:    $TORQUE_THRESHOLD Nm"
echo "  Auto Start:          $AUTO_START"
echo ""
echo "Usage: $0 [velocity] [force_threshold] [torque_threshold] [auto_start]"
echo "Example: $0 0.15 3.0 0.15 true"
echo ""
echo "Starting in 3 seconds... (Ctrl+C to cancel)"
sleep 3

# Run the node directly
ros2 run aristo_hardware_interface ft_collision_test_node \
    --ros-args \
    -p joint5_velocity:=$VELOCITY \
    -p force_threshold:=$FORCE_THRESHOLD \
    -p torque_threshold:=$TORQUE_THRESHOLD \
    -p auto_start:=$AUTO_START
