#!/bin/bash
set -e
echo "=== Cleaning Gazebo + ROS2 processes and cache ==="
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
pkill -9 ros2 2>/dev/null || true
rm -rf ~/.gazebo/cache/*
rm -rf /tmp/gazebo-*
echo "=== Clean complete ==="
