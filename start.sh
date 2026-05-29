#!/bin/bash

# Exit immediately if any command fails
set -e

# Move to the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# ── Step 0: External dependency ────────────────────────────────────────────────
echo ""
echo ">> Checking bme_gazebo_sensors dependency..."

BME_LINK="src/bme_gazebo_sensors"
BME_TARGET="../../bme_gazebo_sensors"

if [ ! -L "$BME_LINK" ] && [ ! -d "$BME_LINK" ]; then
    echo "   Symlink not found — creating it now..."
    ln -s "$BME_TARGET" "$BME_LINK"
    echo "   Done."
else
    echo "   Already present, skipping."
fi

# ── Step 1: Source ROS2 ────────────────────────────────────────────────────────
echo ""
echo ">> Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

# ── Step 2: Build ──────────────────────────────────────────────────────────────
echo ""
echo ">> Building workspace with colcon..."
colcon build

# ── Step 3: Source local install ───────────────────────────────────────────────
echo ""
echo ">> Sourcing local workspace..."
source install/setup.bash

# ── Step 4: Launch ─────────────────────────────────────────────────────────────
echo ""
echo ">> Launching navigation system..."
ros2 launch action_server navigation_launch.py
