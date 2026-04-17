# offboard_px4_course

This repository is now a ROS 2 Python package named `offboard_px4_course`.
It contains two **ROS 2 + MAVROS** nodes:

- `offboard`: publish local position setpoints, request `OFFBOARD`, arm the vehicle, and hold the current `x/y` position while climbing to a fixed altitude.
- `takeofflanding`: perform an offboard takeoff, hold, and landing mission.
- `aruco_takeoff`: take off, search a ground ArUco marker with a zigzag path, and hover when it is detected.

---

## Requirements

- ROS 2 Humble
- PX4 v1.15
- MAVROS
- OpenCV ArUco + `cv_bridge`

---

## Run Instructions

**1. Build Workspace**
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**2. Start PX4 SITL**
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

**6. Run the ArUco takeoff/detection node**
```bash
ros2 run offboard_px4_course aruco_takeoff
```

The node subscribes to `/camera/down/image_raw`, publishes a debug image to `/aruco/debug_image`, flies a zigzag search pattern, and logs `Aruco detected (id=7)` when the marker is visible. After detection it hovers at the current position.

---

## ArUco World

An ArUco world file is provided at `worlds/aruco_takeoff_world.sdf`.
The marker is a `DICT_4X4_50` marker with id `7`, reduced to `1.2 m x 1.2 m`, and placed at `(2.2, 0.8)` so the UAV has to move away from the takeoff point and search for it.

---

## Flight Logic Summary

- `offboard` keeps publishing local position setpoints before requesting `OFFBOARD`.
- `offboard` requests `OFFBOARD` through `/mavros/set_mode`, arms through `/mavros/cmd/arming`, and holds the current `x/y` position at `TARGET_ALT = 8.0m`.
- `takeofflanding` sends preflight setpoints, switches to `OFFBOARD`, arms, climbs, holds altitude, then lands and disarms.
- `aruco_takeoff` performs a vertical takeoff to `2.5m`, flies a zigzag search path over the search area, then hovers at the detection position once the configured ArUco marker is found.
- Both nodes keep publishing at `10 Hz` so PX4 remains in `OFFBOARD`.
