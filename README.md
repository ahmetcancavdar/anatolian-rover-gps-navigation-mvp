# Anatolian Rover MVP ROS 2 Workspace

This workspace contains a minimal GPS-based autonomous rover system designed for the first simulation stage.

## Packages

- `anatolian_rover_msgs`: custom message definitions.
- `anatolian_rover_mvp`: Python ROS 2 nodes for mission management, simulated GPS, simulated IMU, heading, navigation, and motor control.

## MVP Node Graph

```text
mission_manager_node -> /mission/current_goal -> navigation_node
sim_gps_data_node -> /gps/fix, /gps/status -> navigation_node
sim_imu9250_node -> /imu/data, /imu/mag -> heading_node -> /heading/yaw -> navigation_node
navigation_node -> /motor/pulse_cmd -> sim_motor_control_node -> /cmd_vel
```

## Build

From the workspace root:

```bash
colcon build
source install/setup.bash
```

## Run

This launch file starts only the ROS nodes. Start Gazebo / Leo Rover separately so that an `/odom` topic is available.

```bash
ros2 launch anatolian_rover_mvp mvp_sim.launch.py
```

## Important Parameters

Edit:

```text
src/anatolian_rover_mvp/config/sim_params.yaml
```

Main parameters:

- `target_latitude`
- `target_longitude`
- `acceptance_radius`
- `reference_latitude`
- `reference_longitude`
- `heading_threshold_deg`
- `forward_pulse_duration`
- `turn_pulse_duration`
- `forward_speed`
- `turn_speed`

## Simulation Assumption

`sim_gps_data_node` converts the robot `/odom` position into fake RTK GPS coordinates using a reference latitude and longitude.

`sim_imu9250_node` uses `/odom` orientation to generate simulated IMU and magnetometer topics.

`sim_motor_control_node` converts `/motor/pulse_cmd` into `/cmd_vel`.

The core logic nodes (`mission_manager_node`, `heading_node`, and `navigation_node`) are intended to be reusable on the real rover.
