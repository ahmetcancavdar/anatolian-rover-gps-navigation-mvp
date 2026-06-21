import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from anatolian_rover_msgs.msg import GeoGoal, GpsStatus, HeadingStatus, MissionState, MotorPulseCommand, NavStatus

from .math_utils import bearing_deg, haversine_distance_m, wrap180


class NavigationNode(Node):
    """GPS + heading based pulse command generator."""

    def __init__(self):
        super().__init__('navigation_node')
        self.declare_parameter('heading_threshold_deg', 15.0)
        self.declare_parameter('default_acceptance_radius', 2.0)
        self.declare_parameter('forward_pulse_duration', 0.5)
        self.declare_parameter('turn_pulse_duration', 0.3)
        self.declare_parameter('forward_speed', 0.3)
        self.declare_parameter('turn_speed', 0.4)
        self.declare_parameter('decision_rate_hz', 2.0)
        self.declare_parameter('require_rtk_fixed', False)
        self.declare_parameter('gps_timeout', 2.0)
        self.declare_parameter('heading_timeout', 1.0)

        self.goal = None
        self.gps = None
        self.gps_status = None
        self.heading = None
        self.heading_status = None
        self.mission_state = None
        self.last_gps_time = None
        self.last_heading_time = None
        self.last_command = 'STOP'
        self.last_turn_direction = None

        self.cmd_pub = self.create_publisher(MotorPulseCommand, '/motor/pulse_cmd', 10)
        self.nav_pub = self.create_publisher(NavStatus, '/nav/status', 10)

        self.create_subscription(GeoGoal, '/mission/current_goal', self.goal_callback, 10)
        self.create_subscription(MissionState, '/mission/state', self.mission_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(GpsStatus, '/gps/status', self.gps_status_callback, 10)
        self.create_subscription(Float32, '/heading/yaw', self.heading_callback, 10)
        self.create_subscription(HeadingStatus, '/heading/status', self.heading_status_callback, 10)

        rate = float(self.get_parameter('decision_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.get_logger().info('navigation_node started')

    def goal_callback(self, msg: GeoGoal):
        self.goal = msg

    def mission_callback(self, msg: MissionState):
        self.mission_state = msg

    def gps_callback(self, msg: NavSatFix):
        self.gps = msg
        self.last_gps_time = self.get_clock().now()

    def gps_status_callback(self, msg: GpsStatus):
        self.gps_status = msg

    def heading_callback(self, msg: Float32):
        self.heading = float(msg.data)
        self.last_heading_time = self.get_clock().now()

    def heading_status_callback(self, msg: HeadingStatus):
        self.heading_status = msg

    def publish_motor_cmd(self, command: str, duration: float, linear: float, angular: float):
        msg = MotorPulseCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command = command
        msg.duration_sec = float(duration)
        msg.linear_speed = float(linear)
        msg.angular_speed = float(angular)
        self.cmd_pub.publish(msg)
        self.last_command = command

    def publish_nav_status(self, distance=0.0, bearing=0.0, error=0.0, reached=False, state='WAITING'):
        msg = NavStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.gps is not None:
            msg.current_latitude = float(self.gps.latitude)
            msg.current_longitude = float(self.gps.longitude)
        if self.goal is not None:
            msg.target_latitude = float(self.goal.latitude)
            msg.target_longitude = float(self.goal.longitude)
        msg.distance_to_target = float(distance)
        msg.target_bearing_deg = float(bearing)
        msg.current_heading_deg = float(self.heading) if self.heading is not None else 0.0
        msg.heading_error_deg = float(error)
        msg.target_reached = bool(reached)
        msg.nav_state = state
        msg.last_command = self.last_command
        self.nav_pub.publish(msg)

    def data_ok(self):
        now = self.get_clock().now()
        gps_timeout = float(self.get_parameter('gps_timeout').value)
        heading_timeout = float(self.get_parameter('heading_timeout').value)
        if self.goal is None or self.gps is None or self.heading is None:
            return False, 'WAITING_FOR_DATA'
        if self.last_gps_time is None or float((now - self.last_gps_time).nanoseconds) / 1e9 > gps_timeout:
            return False, 'GPS_TIMEOUT'
        if self.last_heading_time is None or float((now - self.last_heading_time).nanoseconds) / 1e9 > heading_timeout:
            return False, 'HEADING_TIMEOUT'
        if self.gps_status is not None and not self.gps_status.gps_ok:
            return False, 'GPS_NOT_OK'
        if self.heading_status is not None and not self.heading_status.heading_ok:
            return False, 'HEADING_NOT_OK'
        if bool(self.get_parameter('require_rtk_fixed').value):
            if self.gps_status is None or self.gps_status.fix_type != 'RTK_FIXED':
                return False, 'RTK_NOT_FIXED'
        if self.mission_state is not None and not self.mission_state.autonomous_enabled:
            return False, 'AUTONOMY_DISABLED'
        return True, 'OK'

    def timer_callback(self):
        ok, reason = self.data_ok()
        if not ok:
            self.publish_motor_cmd('STOP', 0.0, 0.0, 0.0)
            self.publish_nav_status(state=reason)
            return

        distance = haversine_distance_m(self.gps.latitude, self.gps.longitude, self.goal.latitude, self.goal.longitude)
        target_bearing = bearing_deg(self.gps.latitude, self.gps.longitude, self.goal.latitude, self.goal.longitude)
        heading_error = wrap180(target_bearing - self.heading)
        radius = float(self.goal.acceptance_radius) if self.goal.acceptance_radius > 0.0 else float(self.get_parameter('default_acceptance_radius').value)

        if distance <= radius:
            self.publish_motor_cmd('STOP', 0.0, 0.0, 0.0)
            self.publish_nav_status(distance, target_bearing, heading_error, reached=True, state='TARGET_REACHED')
            return

        threshold = float(self.get_parameter('heading_threshold_deg').value)
        if abs(heading_error) > threshold:
            duration = float(self.get_parameter('turn_pulse_duration').value)
            turn_speed = float(self.get_parameter('turn_speed').value)

            # Around +/-180 degrees, the shortest turn direction is ambiguous.
            # Without this latch, the command can flip LEFT/RIGHT at every cycle.
            if abs(heading_error) > 160.0 and self.last_turn_direction is not None:
                turn_direction = self.last_turn_direction
            elif heading_error > 0.0:
                turn_direction = 'TURN_RIGHT'
            else:
                turn_direction = 'TURN_LEFT'

            self.last_turn_direction = turn_direction

            if turn_direction == 'TURN_RIGHT':
                self.publish_motor_cmd('TURN_RIGHT', duration, 0.0, -turn_speed)
                nav_state = 'TURN_RIGHT'
            else:
                self.publish_motor_cmd('TURN_LEFT', duration, 0.0, turn_speed)
                nav_state = 'TURN_LEFT'
        else:
            self.last_turn_direction = None
            duration = float(self.get_parameter('forward_pulse_duration').value)
            speed = float(self.get_parameter('forward_speed').value)
            self.publish_motor_cmd('FORWARD', duration, speed, 0.0)
            nav_state = 'FORWARD'

        self.publish_nav_status(distance, target_bearing, heading_error, reached=False, state=nav_state)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
