# How to Run the Anatolian Rover GPS Navigation MVP

This document explains how to build and run the Anatolian Rover GPS Navigation MVP in ROS 2 with the Leo Rover Gazebo simulation.

---

## 1. Workspace Location

Keep the workspace in a path without non-English characters.

Recommended:

```bash
~/anatolian_rover_mvp_ws
```

Avoid paths such as:

```bash
~/Masaüstü/anatolian_rover_mvp_ws
```

ROS 2 Humble and CMake tools may fail when the workspace path contains non-ASCII characters.

---

## 2. Requirements

The system assumes that the following are already installed:

* ROS 2 Humble
* Gazebo / Ignition Gazebo
* Leo Rover Gazebo simulation packages
* `colcon`
* Python 3
* A working Leo Rover simulation workspace, for example:

```bash
~/leo_corridor_ws
```

The MVP workspace is expected to be located at:

```bash
~/anatolian_rover_mvp_ws
```

---

## 3. Build the Workspace

Open a terminal:

```bash
cd ~/anatolian_rover_mvp_ws

source /opt/ros/humble/setup.bash
source ~/leo_corridor_ws/install/setup.bash

colcon build --symlink-install

source install/setup.bash
```

If the build succeeds, you should see both packages built:

```text
anatolian_rover_msgs
anatolian_rover_mvp
```

---

## 4. Run the Full System

Use three terminals.

---

### Terminal 1: Start Leo Rover Gazebo Simulation

```bash
source /opt/ros/humble/setup.bash
source ~/leo_corridor_ws/install/setup.bash

ros2 launch leo_gz_bringup leo_gz.launch.py
```

This should start Gazebo and spawn the Leo Rover model.

Expected bridge topics:

```text
/cmd_vel
/odom
```

---

### Terminal 2: Start Anatolian Rover MVP Nodes

```bash
source /opt/ros/humble/setup.bash
source ~/leo_corridor_ws/install/setup.bash
source ~/anatolian_rover_mvp_ws/install/setup.bash

ros2 launch anatolian_rover_mvp mvp_sim.launch.py
```

This starts:

```text
mission_manager_node
gps_data_node
imu9250_node
heading_node
navigation_node
motor_control_node
```

---

### Terminal 3: Start the Live Monitor

```bash
source /opt/ros/humble/setup.bash
source ~/leo_corridor_ws/install/setup.bash
source ~/anatolian_rover_mvp_ws/install/setup.bash

ros2 run anatolian_rover_mvp rover_monitor_node
```

The monitor displays:

```text
Mission State
Active Goal ID
Current GPS
Target GPS
Acceptance Radius
Current Heading
Target Bearing
Heading Error
Distance to Target
Navigation State
Motor Command
Motor Status
```

---

## 5. Check ROS 2 Topics

Open another terminal if needed:

```bash
source /opt/ros/humble/setup.bash
source ~/leo_corridor_ws/install/setup.bash
source ~/anatolian_rover_mvp_ws/install/setup.bash
```

List relevant topics:

```bash
ros2 topic list | grep -E "odom|cmd_vel|gps|heading|motor|nav|mission|imu"
```

Expected topics:

```text
/cmd_vel
/odom
/gps/fix
/gps/status
/imu/data
/imu/mag
/imu/status
/heading/yaw
/heading/status
/mission/current_goal
/mission/state
/motor/pulse_cmd
/motor/status
/nav/status
```

---

## 6. Send a New GPS Target

The current MVP has only one active target at a time.

If ROS 2 CLI daemon is working correctly, a new target can be sent using:

```bash
ros2 param set /mission_manager_node goal_id goal_2
ros2 param set /mission_manager_node target_latitude 39.77604797
ros2 param set /mission_manager_node target_longitude 30.52003000
```

The monitor should update:

```text
Active Goal ID : goal_2
Target GPS     : 39.77604797, 30.52003000
Target Reached : false
```

---

## 7. Alternative Goal Sending Method

Sometimes the ROS 2 CLI daemon may fail with the following error:

```text
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
```

In that case, restart the daemon:

```bash
ros2 daemon stop
pkill -f ros2_daemon
ros2 daemon start
```

If the issue still continues, use the Python parameter service method below:

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


def make_string_param(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue()
    p.value.type = ParameterType.PARAMETER_STRING
    p.value.string_value = value
    return p


def make_double_param(name, value):
    p = Parameter()
    p.name = name
    p.value = ParameterValue()
    p.value.type = ParameterType.PARAMETER_DOUBLE
    p.value.double_value = float(value)
    return p


rclpy.init()

node = Node('manual_goal_setter')

client = node.create_client(SetParameters, '/mission_manager_node/set_parameters')

if not client.wait_for_service(timeout_sec=5.0):
    print("ERROR: /mission_manager_node/set_parameters service not available.")
    print("Check if the MVP launch file is running.")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1)

req = SetParameters.Request()
req.parameters = [
    make_string_param('goal_id', 'goal_2'),
    make_double_param('target_latitude', 39.77604797),
    make_double_param('target_longitude', 30.52003000),
]

future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

if future.result() is None:
    print("ERROR: Parameter set request failed.")
else:
    for result in future.result().results:
        print(f"successful={result.successful}, reason={result.reason}")

print("New goal sent:")
print("  goal_id: goal_2")
print("  target_latitude: 39.77604797")
print("  target_longitude: 30.52003000")

node.destroy_node()
rclpy.shutdown()
PY
```

---

## 8. Example GPS Targets

Starting target:

```text
39.77603000, 30.52003000
```

Approximately 2 meters north:

```text
39.77604797, 30.52003000
```

Approximately 2 meters east:

```text
39.77603000, 30.52005338
```

Approximately 2 meters south:

```text
39.77601203, 30.52003000
```

Approximately 2 meters west:

```text
39.77603000, 30.52000662
```

---

## 9. Important Configuration File

Main simulation parameters are located in:

```bash
src/anatolian_rover_mvp/config/sim_params.yaml
```

Important parameters:

```yaml
acceptance_radius: 0.05
heading_threshold_deg: 8.0
forward_pulse_duration: 0.08
turn_pulse_duration: 0.06
forward_speed: 0.08
turn_speed: 0.12
decision_rate_hz: 8.0
```

---

## 10. Tuning Notes

### If the rover stops too far from the target

Decrease:

```yaml
acceptance_radius
```

Example:

```yaml
acceptance_radius: 0.05
```

---

### If the rover oscillates near the target

Decrease:

```yaml
forward_speed
forward_pulse_duration
turn_speed
turn_pulse_duration
```

---

### If the rover turns left and right repeatedly

Check:

```text
current_heading_deg
target_bearing_deg
heading_error_deg
```

The system contains a turn-direction latch to reduce left-right flipping near 180 degrees.

---

### If the rover does not move

Check:

```bash
ros2 topic echo /motor/pulse_cmd
ros2 topic echo /cmd_vel
```

If `/motor/pulse_cmd` exists but `/cmd_vel` does not change, check `motor_control_node`.

If `/cmd_vel` changes but Gazebo does not move, check the Leo Rover Gazebo bridge.

---

### If `/gps/fix` does not publish valid data

Check if `/odom` exists:

```bash
ros2 topic echo /odom
```

The simulation GPS node requires Gazebo `/odom`.

---

## 11. Clean Rebuild

If the workspace behaves unexpectedly:

```bash
cd ~/anatolian_rover_mvp_ws

rm -rf build install log

source /opt/ros/humble/setup.bash
source ~/leo_corridor_ws/install/setup.bash

colcon build --symlink-install

source install/setup.bash
```

---

## 12. Full Restart Procedure

Stop all running terminals with `Ctrl+C`.

Then run:

```bash
pkill -f ros2
pkill -f ign
pkill -f gazebo
pkill -f parameter_bridge
pkill -f robot_state_publisher
```

Restart the ROS 2 daemon:

```bash
ros2 daemon stop
ros2 daemon start
```

Then start the system again using:

1. Terminal 1: Gazebo
2. Terminal 2: MVP nodes
3. Terminal 3: live monitor

---

## 13. Current System Limitation

This MVP currently supports one active target at a time.

Automatic waypoint sequencing is not implemented yet.

The next recommended improvement is to extend `mission_manager_node` so it can read a list of GPS waypoints and publish the next waypoint automatically after the current one is reached.
