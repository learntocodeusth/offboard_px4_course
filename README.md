# offboard_px4_course

This repository is now a ROS 2 Python package named `offboard_px4_course`.
It contains four **ROS 2 + MAVROS** nodes:

- `offboard`: publish local position setpoints, request `OFFBOARD`, arm the vehicle, and hold the current `x/y` position while climbing to a fixed altitude.
- `takeofflanding`: perform an offboard takeoff, hold, and landing mission.
- `takeoff_point_hold`: take off, fly to a target point, and keep hovering there.
- `aruco_takeoff`: take off directly above a ground ArUco marker, detect it from a downward RGB camera, and hover above it.

---

## Requirements

- ROS 2 Humble
- PX4 v1.15
- MAVROS
- OpenCV ArUco + `cv_bridge`

---

## Guidance

Theory and behavior notes for `offboard`, `takeofflanding`, and `takeoff_point_hold` are documented in [guidance.md](guidance.md).

---

## Run Instructions

**1. Build Workspace**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Start PX4 SITL for the basic x500**
```bash
make px4_sitl gz_x500
```

**3. Run MAVROS**
```bash
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@127.0.0.1:14557
```

**4. Run the OFFBOARD hold node**
```bash
ros2 run offboard_px4_course offboard
```

**5. Run the takeoff/landing node**
```bash
ros2 run offboard_px4_course takeofflanding
```

**6. Run the ArUco scenario**
Start PX4 SITL with the down-facing monocular camera and the ArUco world:
```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/path/to/offboard_px4_course/worlds
PX4_GZ_WORLD=aruco_takeoff_world make px4_sitl gz_x500_mono_cam_down
```

Bridge the Gazebo camera topic into ROS 2:
```bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
```

Run MAVROS:
```bash
ros2 launch mavros px4.launch.py fcu_url:=udp://:14540@127.0.0.1:14557
```

Run the ArUco takeoff/detection node:
```bash
ros2 run offboard_px4_course aruco_takeoff
```

The node subscribes to `/camera` by default, expects an RGB image topic rather than a depth image, publishes a debug image to `/aruco/debug_image`, and logs `Aruco detected (id=7)` when the marker is visible. PX4 takes off vertically above the marker and keeps hovering there.

**7. Run the takeoff-point-hold node**
```bash
ros2 run offboard_px4_course takeoff_point_hold
```

The node takes off, flies to a target point relative to home, and hovers there. Default target is `x=+3.0m`, `y=0.0m`, `z=2.5m`, and you can override it with ROS parameters such as:
```bash
ros2 run offboard_px4_course takeoff_point_hold --ros-args -p target_x_offset:=5.0 -p target_y_offset:=1.0 -p target_altitude:=3.0
```

---

## ArUco World

An ArUco world file is provided at `worlds/aruco_takeoff_world.sdf`.
The marker is a `DICT_4X4_50` marker with id `7`, sized to `1.6 m x 1.6 m`, and centered at `(0, 0)` so PX4 spawns directly above it.

---

## Flight Logic Summary

- `offboard` keeps publishing local position setpoints before requesting `OFFBOARD`.
- `offboard` requests `OFFBOARD` through `/mavros/set_mode`, arms through `/mavros/cmd/arming`, and holds the current `x/y` position at `TARGET_ALT = 8.0m`.
- `takeofflanding` sends preflight setpoints, switches to `OFFBOARD`, arms, climbs, holds altitude, then lands and disarms.
- `takeoff_point_hold` performs a vertical takeoff, then flies to a configurable point relative to the takeoff position and keeps hovering there.
- `aruco_takeoff` performs a vertical takeoff to `2.0m`, keeps `x/y` fixed above the takeoff point, detects the configured ArUco marker from the downward RGB camera, and continues hovering once detected.
- Both nodes keep publishing at `10 Hz` so PX4 remains in `OFFBOARD`.
