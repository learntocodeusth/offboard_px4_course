# Offboard PX4, Gazebo, ROS2 course - ISEP 2026

This package now keeps only four runnable ROS 2 nodes:

- `offboard`
- `takeofflanding`
- `takeoff_point_hold`
- `mono_aruco_takeoff`

## Requirements

To run smoothly, you will need the following software packages (in addition to the PX4-Autopilot directory):

- **ROS 2 Humble**
- **PX4 SITL** with Gazebo Harmonic
- **MAVROS**: `sudo apt install ros-humble-mavros ros-humble-mavros-msgs`
- **Gazebo Python bindings** (used by the `mono_aruco_takeoff` node for direct image transport without a ROS bridge): `sudo apt install python3-gz-transport13 python3-gz-msgs10 python3-protobuf`
- **Computer Vision**: `sudo apt install ros-humble-cv-bridge python3-opencv`

## Recommended Working Flow

Use this flow for the ArUco + OFFBOARD scenario:

- PX4 vehicle: `gz_x500_mono_cam_down`
- Gazebo world: `aruco_takeoff_world`
- Image path: direct Gazebo transport
- Flight control path: MAVROS
- Main node: `mono_aruco_takeoff`

Use three terminals.

### Terminal 1: PX4 + Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=aruco_takeoff_world make px4_sitl gz_x500_mono_cam_down
```

### Terminal 2: MAVROS

```bash
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

### Terminal 3: Mono ArUco OFFBOARD Node

```bash
source offboard_px4_course/install/setup.bash
ros2 run offboard_px4_course mono_aruco_takeoff
```

Useful overrides:

```bash
ros2 run offboard_px4_course mono_aruco_takeoff --ros-args -p target_altitude:=2.5
ros2 run offboard_px4_course mono_aruco_takeoff --ros-args -p aruco_id:=0 -p show_camera:=false
```

## Other Nodes

### `offboard`

```bash
ros2 run offboard_px4_course offboard
```

Publishes local position setpoints, requests `OFFBOARD`, arms, and climbs while holding `x/y`.

### `takeofflanding`

```bash
ros2 run offboard_px4_course takeofflanding
```

Performs offboard takeoff, hover, landing, and disarm.

### `takeoff_point_hold`

```bash
ros2 run offboard_px4_course takeoff_point_hold
```

Takes off, flies to a target point, and hovers there.

Example:

```bash
ros2 run offboard_px4_course takeoff_point_hold --ros-args -p target_x_offset:=5.0 -p target_y_offset:=1.0 -p target_altitude:=3.0
```

## World

The package keeps `worlds/aruco_takeoff_world.sdf`.

It contains `model://arucotag` centered at `(0, 0)`, and its internal Gazebo world name is `aruco_takeoff_world` so it matches:

```bash
PX4_GZ_WORLD=aruco_takeoff_world
```
