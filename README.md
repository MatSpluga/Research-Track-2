# ROS2 Navigation with Action Server and Client

## Overview

This workspace implements a goal-based navigation system for a simulated robot in Gazebo, using ROS2 Actions to coordinate movement. A C++ action server receives target poses and drives the robot using proportional control, while an interactive action client lets the user send goals, cancel them, or shut down the system from the terminal. The simulated robot is provided by the external package `bme_gazebo_sensors`.

The workspace is organized into two ROS2 packages:

- **`interfaces`**: defines the custom `Navigate` action type shared between server and client.
- **`action_server`**: contains the C++ nodes for the server and client, along with the launch file.

---

## Package Details

### interfaces

Defines the `Navigate.action` file with the following structure:

- **Goal**: target position and orientation (`float64 x`, `float64 y`, `float64 theta`)
- **Result**: outcome of the navigation (`bool success`)
- **Feedback**: distance still to cover (`float64 distance_remaining`)

### action_server

Contains the core navigation logic built on ROS2 Actions and `tf2`.

- **`nav_action_server_node`**: receives a goal pose and publishes velocity commands on `cmd_vel` to steer the robot. Pose estimation uses `tf2` transforms between `odom` and `base_footprint` as the primary source, with automatic fallback to direct `/odom` subscription if the TF tree is unavailable. The node also listens on `/shutdown` to terminate cleanly when the client exits.

- **`nav_action_client_node`**: provides a terminal-based interface for the user. Commands are handled in a dedicated thread so the node stays responsive while a goal is in progress.

- **`navigation_launch.py`**: brings up the full system — Gazebo simulation, ROS-Gazebo bridge, action server, and action client — with server and client each running in their own `gnome-terminal` window.

---

## Dependencies

- ROS2 Jazzy
- `ros_gz` (Gazebo Sim — Harmonic)
- `rviz2`, `robot_state_publisher`
- External package: `bme_gazebo_sensors` (symlinked automatically by `start.sh`)
- ROS2 C++ libraries: `rclcpp`, `rclcpp_action`, `rclcpp_components`
- Transform support: `tf2`, `tf2_ros`, `tf2_geometry_msgs`
- Message types: `geometry_msgs`, `nav_msgs`, `std_msgs`
- `gnome-terminal` (used by the launch file to open nodes in separate windows)

---

## Building and Running

The provided `start.sh` script handles everything in sequence: it symlinks `bme_gazebo_sensors` if needed, builds the workspace with `colcon`, sources the environment, and launches the system.

```bash
cd <workspace_root>
chmod +x start.sh
./start.sh
```

This will start the Gazebo simulation with the robot, the ROS-Gazebo bridge, and open two terminal windows — one for the action server and one for the action client.

---

## Using the Client

Once the client terminal is ready, the following prompt appears:

```
=== Navigation Control ===
 'g' — send a new goal
 'c' — cancel current goal
 'q' — quit
Input:
```

- **`g`**: send a new goal. You will be asked to enter the target coordinates as `x y theta` (e.g. `3.0 1.5 0.0` — position x=3.0, y=1.5, orientation theta=0.0).
- **`c`**: cancel the goal currently in execution and stop the robot.
- **`q`**: cancel any active goal, send a shutdown signal to the server, and close the entire system.
