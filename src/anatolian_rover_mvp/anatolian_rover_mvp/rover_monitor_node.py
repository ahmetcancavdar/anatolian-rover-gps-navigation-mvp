import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from anatolian_rover_msgs.msg import (
    GeoGoal,
    MissionState,
    MotorPulseCommand,
    MotorStatus,
    NavStatus,
    GpsStatus,
    HeadingStatus,
)


class RoverMonitorNode(Node):
    def __init__(self):
        super().__init__('rover_monitor_node')

        self.declare_parameter('refresh_rate_hz', 2.0)

        self.goal = None
        self.mission_state = None
        self.gps = None
        self.gps_status = None
        self.heading = None
        self.heading_status = None
        self.nav_status = None
        self.motor_cmd = None
        self.motor_status = None

        self.create_subscription(GeoGoal, '/mission/current_goal', self.goal_cb, 10)
        self.create_subscription(MissionState, '/mission/state', self.mission_state_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_cb, 10)
        self.create_subscription(GpsStatus, '/gps/status', self.gps_status_cb, 10)
        self.create_subscription(Float32, '/heading/yaw', self.heading_cb, 10)
        self.create_subscription(HeadingStatus, '/heading/status', self.heading_status_cb, 10)
        self.create_subscription(NavStatus, '/nav/status', self.nav_status_cb, 10)
        self.create_subscription(MotorPulseCommand, '/motor/pulse_cmd', self.motor_cmd_cb, 10)
        self.create_subscription(MotorStatus, '/motor/status', self.motor_status_cb, 10)

        rate = float(self.get_parameter('refresh_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.print_dashboard)

        self.get_logger().info('rover_monitor_node started')

    def goal_cb(self, msg):
        self.goal = msg

    def mission_state_cb(self, msg):
        self.mission_state = msg

    def gps_cb(self, msg):
        self.gps = msg

    def gps_status_cb(self, msg):
        self.gps_status = msg

    def heading_cb(self, msg):
        self.heading = msg

    def heading_status_cb(self, msg):
        self.heading_status = msg

    def nav_status_cb(self, msg):
        self.nav_status = msg

    def motor_cmd_cb(self, msg):
        self.motor_cmd = msg

    def motor_status_cb(self, msg):
        self.motor_status = msg

    def fmt_float(self, value, digits=3):
        try:
            if value is None or math.isnan(value):
                return 'N/A'
            return f'{float(value):.{digits}f}'
        except Exception:
            return 'N/A'

    def fmt_bool(self, value):
        return 'true' if bool(value) else 'false'

    def print_dashboard(self):
        print('\033[2J\033[H', end='')

        print('============================================================')
        print('              ANATOLIAN ROVER LIVE MONITOR')
        print('============================================================')

        if self.mission_state is not None:
            print(f'Mission State     : {self.mission_state.state}')
            print(f'Active Goal ID    : {self.mission_state.active_goal_id}')
            print(f'Autonomous Enabled: {self.fmt_bool(self.mission_state.autonomous_enabled)}')
            print(f'Target Reached    : {self.fmt_bool(self.mission_state.target_reached)}')
        else:
            print('Mission State     : WAITING_FOR_MISSION_STATE')

        print('------------------------------------------------------------')

        if self.gps is not None:
            print(f'Current GPS       : {self.fmt_float(self.gps.latitude, 8)}, {self.fmt_float(self.gps.longitude, 8)}')
        else:
            print('Current GPS       : WAITING_FOR_GPS')

        if self.gps_status is not None:
            print(f'GPS Status        : {self.gps_status.fix_type} | gps_ok={self.fmt_bool(self.gps_status.gps_ok)} | acc={self.fmt_float(self.gps_status.horizontal_accuracy, 3)} m')
        else:
            print('GPS Status        : WAITING_FOR_GPS_STATUS')

        if self.goal is not None:
            print(f'Target GPS        : {self.fmt_float(self.goal.latitude, 8)}, {self.fmt_float(self.goal.longitude, 8)}')
            print(f'Acceptance Radius : {self.fmt_float(self.goal.acceptance_radius, 2)} m')
        else:
            print('Target GPS        : WAITING_FOR_GOAL')

        print('------------------------------------------------------------')

        if self.heading is not None:
            print(f'Current Heading   : {self.fmt_float(self.heading.data, 2)} deg')
        else:
            print('Current Heading   : WAITING_FOR_HEADING')

        if self.heading_status is not None:
            print(f'Heading OK        : {self.fmt_bool(self.heading_status.heading_ok)}')
        else:
            print('Heading OK        : WAITING_FOR_HEADING_STATUS')

        print('------------------------------------------------------------')

        if self.nav_status is not None:
            print(f'Distance to Target: {self.fmt_float(self.nav_status.distance_to_target, 2)} m')
            print(f'Target Bearing    : {self.fmt_float(self.nav_status.target_bearing_deg, 2)} deg')
            print(f'Heading Error     : {self.fmt_float(self.nav_status.heading_error_deg, 2)} deg')
            print(f'Navigation State  : {self.nav_status.nav_state}')
            print(f'Last Nav Command  : {self.nav_status.last_command}')
        else:
            print('Navigation State  : WAITING_FOR_NAV_STATUS')

        print('------------------------------------------------------------')

        if self.motor_cmd is not None:
            print(f'Motor Pulse Cmd   : {self.motor_cmd.command}')
            print(f'Pulse Duration    : {self.fmt_float(self.motor_cmd.duration_sec, 2)} s')
            print(f'Linear Speed      : {self.fmt_float(self.motor_cmd.linear_speed, 2)}')
            print(f'Angular Speed     : {self.fmt_float(self.motor_cmd.angular_speed, 2)}')
        else:
            print('Motor Pulse Cmd   : WAITING_FOR_MOTOR_CMD')

        if self.motor_status is not None:
            print(f'Motor Moving      : {self.fmt_bool(self.motor_status.is_moving)}')
            print(f'Motor Status      : {self.motor_status.status_text}')
        else:
            print('Motor Status      : WAITING_FOR_MOTOR_STATUS')

        print('============================================================')

        if self.nav_status is not None and self.nav_status.target_reached:
            print('INFO: Target reached. This MVP currently has only one target.')
            print('INFO: To move again, send a new target or implement waypoint list.')

        print('Press Ctrl+C to stop monitor.')


def main(args=None):
    rclpy.init(args=args)
    node = RoverMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
