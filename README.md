# Anatolian Rover GPS Navigation MVP

## Overview

This repository contains a minimal ROS 2-based autonomous navigation system for an Anatolian Rover-style mobile robot. The project is designed as the first-stage software architecture for a rover that navigates using corrected RTK GPS data and heading information.

The system is intentionally simple. It does not use Nav2, SLAM, wheel odometry, robot_localization, LiDAR, camera perception, or obstacle avoidance. Instead, it focuses on a clear and testable point-to-point GPS navigation pipeline.

The current implementation runs in Gazebo with the Leo Rover simulation. Gazebo odometry is used only as a simulation source to generate fake RTK GPS and IMU/heading data. The main navigation logic does not directly depend on Gazebo, so it can later be reused with real GPS, IMU, and motor hardware.

---

## Project Goal

The goal of this MVP is to make a simulated rover:

1. Receive a target GPS coordinate.
2. Read its current simulated GPS position.
3. Read its current heading.
4. Compute the bearing from its current position to the target GPS point.
5. Compare the target bearing with the current heading.
6. Rotate toward the target.
7. Move forward in short pulse commands.
8. Stop when it reaches the target within a configurable acceptance radius.

---

## Current Capabilities

The system currently supports:

* Simulated RTK GPS generation from Gazebo `/odom`
* Simulated IMU and heading generation
* GPS target publishing through a mission manager node
* Bearing and distance calculation from GPS coordinates
* Heading error calculation
* Pulse-based motion commands
* Gazebo `/cmd_vel` motor control
* Live terminal monitoring of rover state
* Runtime target updates through ROS 2 parameters or the parameter service
* Configurable acceptance radius and motion tuning parameters

---

## System Architecture

The MVP system contains the following main nodes:

```text
mission_manager_node
gps_data_node
imu9250_node
heading_node
navigation_node
motor_control_node
rover_monitor_node
```

---

## Data Flow

```text
Gazebo /odom
    ↓
gps_data_node
    ↓
/gps/fix
/gps/status
    ↓
navigation_node
```

```text
Gazebo /odom
    ↓
imu9250_node
    ↓
/imu/data
/imu/mag
    ↓
heading_node
    ↓
/heading/yaw
/heading/status
    ↓
navigation_node
```

```text
mission_manager_node
    ↓
/mission/current_goal
/mission/state
    ↓
navigation_node
```

```text
navigation_node
    ↓
/motor/pulse_cmd
    ↓
motor_control_node
    ↓
/cmd_vel
    ↓
Gazebo Leo Rover
```

```text
rover_monitor_node
    ↓
Subscribes to system topics
    ↓
Displays live rover status in the terminal
```

---

## Node Descriptions

### mission_manager_node

The `mission_manager_node` publishes the active GPS target and mission state.

It publishes:

```text
/mission/current_goal
/mission/state
```

The current MVP uses a single target GPS coordinate loaded from the configuration file. New targets can also be sent at runtime by changing node parameters.

---

### gps_data_node

The `gps_data_node` publishes the rover’s current GPS position.

In simulation, it converts Gazebo `/odom` position into fake GPS latitude and longitude using a fixed reference GPS origin.

It publishes:

```text
/gps/fix
/gps/status
```

In the real rover, this node can later be replaced with a hardware driver that reads corrected RTK GPS data from the LC29HEA GNSS module.

---

### imu9250_node

The `imu9250_node` publishes simulated IMU and magnetometer data.

In simulation, it uses Gazebo `/odom` orientation to generate IMU-like data.

It publishes:

```text
/imu/data
/imu/mag
/imu/status
```

In the real rover, this node can later be replaced with a driver for an MPU9250 / IMU9250 / GY-91 / GY-271 sensor setup.

---

### heading_node

The `heading_node` calculates rover heading.

It subscribes to:

```text
/imu/data
/imu/mag
```

It publishes:

```text
/heading/yaw
/heading/status
```

Heading convention:

```text
0 degrees   = North
90 degrees  = East
180 degrees = South
270 degrees = West
```

For Gazebo simulation, ROS yaw is converted into compass-style heading using:

```text
heading = 90 - ros_yaw
```

---

### navigation_node

The `navigation_node` is the main decision-making node.

It subscribes to:

```text
/mission/current_goal
/gps/fix
/gps/status
/heading/yaw
/heading/status
```

It publishes:

```text
/motor/pulse_cmd
/nav/status
```

The navigation node calculates:

* Distance to target
* Target bearing
* Current heading
* Heading error
* Motion decision

The basic decision logic is:

```text
If distance_to_target <= acceptance_radius:
    STOP

Else if heading_error is larger than threshold:
    TURN_LEFT or TURN_RIGHT

Else:
    FORWARD
```

The node outputs short pulse commands instead of continuous motion commands.

---

### motor_control_node

The `motor_control_node` executes pulse commands.

It subscribes to:

```text
/motor/pulse_cmd
```

It publishes:

```text
/cmd_vel
/motor/status
```

In simulation, it converts rover pulse commands into Gazebo `/cmd_vel` messages.

In the real rover, this node can later be replaced with a hardware motor controller interface.

---

### rover_monitor_node

The `rover_monitor_node` is a terminal-based live dashboard.

It displays:

* Mission state
* Active goal ID
* Current GPS
* Target GPS
* Acceptance radius
* Current heading
* Target bearing
* Heading error
* Distance to target
* Navigation state
* Motor command
* Motor status

This node is only for monitoring and debugging. It does not control the rover.

---

## Package Structure

```text
anatolian_rover_mvp_ws/
├── README.md
├── HOW_TO_RUN.md
├── .gitignore
└── src/
    ├── anatolian_rover_msgs/
    │   ├── msg/
    │   │   ├── GeoGoal.msg
    │   │   ├── GpsStatus.msg
    │   │   ├── HeadingStatus.msg
    │   │   ├── ImuStatus.msg
    │   │   ├── MissionState.msg
    │   │   ├── MotorPulseCommand.msg
    │   │   ├── MotorStatus.msg
    │   │   └── NavStatus.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── anatolian_rover_mvp/
        ├── anatolian_rover_mvp/
        │   ├── mission_manager_node.py
        │   ├── sim_gps_data_node.py
        │   ├── sim_imu9250_node.py
        │   ├── heading_node.py
        │   ├── navigation_node.py
        │   ├── sim_motor_control_node.py
        │   ├── rover_monitor_node.py
        │   └── math_utils.py
        ├── config/
        │   └── sim_params.yaml
        ├── launch/
        │   └── mvp_sim.launch.py
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

---

## Configuration

Main parameters are stored in:

```text
src/anatolian_rover_mvp/config/sim_params.yaml
```

Important parameters include:

```yaml
target_latitude: 39.776030
target_longitude: 30.520030
acceptance_radius: 0.05

heading_threshold_deg: 8.0

forward_pulse_duration: 0.08
turn_pulse_duration: 0.06

forward_speed: 0.08
turn_speed: 0.12

decision_rate_hz: 8.0
```

---

## Current Limitations

This MVP does not include:

* Nav2
* SLAM
* LiDAR
* Camera perception
* Obstacle avoidance
* Wheel encoder odometry
* robot_localization
* Full waypoint mission sequencing
* Real RTK GPS driver
* Real IMU driver
* Real motor controller interface

These features are planned for later development stages.

---

## Future Work

Planned improvements:

* Add waypoint list support to `mission_manager_node`
* Add real LC29HEA RTK GPS driver
* Add real IMU9250 / GY-91 / GY-271 driver
* Add magnetometer calibration support
* Add real motor controller interface
* Add safety supervisor node
* Add RSCP communication support
* Add camera-based ArUco / basalt / base gate detection
* Add 2D LiDAR-based base gate entry support
* Add launch profiles for simulation and real rover
* Add automated tests for bearing, heading error, and goal reaching logic

---

## License

This project is currently intended for educational and research use.
