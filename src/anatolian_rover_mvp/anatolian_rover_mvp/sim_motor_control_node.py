import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from anatolian_rover_msgs.msg import MissionState, MotorPulseCommand, MotorStatus


class SimMotorControlNode(Node):
    """Executes pulse commands by publishing /cmd_vel for Gazebo.

    Real deployment can keep the same /motor/pulse_cmd input but replace /cmd_vel
    output with serial/CAN/GPIO commands to the motor controller.
    """

    def __init__(self):
        super().__init__('motor_control_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('watchdog_timeout', 1.0)
        self.declare_parameter('status_rate_hz', 10.0)

        self.active_until = None
        self.last_cmd_time = None
        self.last_command = 'STOP'
        self.is_moving = False
        self.autonomous_enabled = True

        self.cmd_vel_pub = self.create_publisher(Twist, str(self.get_parameter('cmd_vel_topic').value), 10)
        self.status_pub = self.create_publisher(MotorStatus, '/motor/status', 10)

        self.create_subscription(MotorPulseCommand, '/motor/pulse_cmd', self.pulse_callback, 10)
        self.create_subscription(MissionState, '/mission/state', self.mission_callback, 10)

        rate = float(self.get_parameter('status_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.get_logger().info('sim motor_control_node started')

    def mission_callback(self, msg: MissionState):
        self.autonomous_enabled = bool(msg.autonomous_enabled)
        if not self.autonomous_enabled:
            self.stop('AUTONOMY_DISABLED')

    def pulse_callback(self, msg: MotorPulseCommand):
        self.last_cmd_time = self.get_clock().now()
        self.last_command = msg.command

        if not self.autonomous_enabled or msg.command == 'STOP' or msg.duration_sec <= 0.0:
            self.stop('STOP_COMMAND')
            return

        twist = Twist()
        twist.linear.x = float(msg.linear_speed)
        twist.angular.z = float(msg.angular_speed)
        self.cmd_vel_pub.publish(twist)
        self.is_moving = abs(twist.linear.x) > 1e-6 or abs(twist.angular.z) > 1e-6
        self.active_until = self.get_clock().now().nanoseconds + int(float(msg.duration_sec) * 1e9)

    def stop(self, reason='STOP'):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.active_until = None
        self.is_moving = False
        self.last_command = reason

    def timer_callback(self):
        now = self.get_clock().now()
        if self.active_until is not None and now.nanoseconds >= self.active_until:
            self.stop('PULSE_FINISHED')

        watchdog = float(self.get_parameter('watchdog_timeout').value)
        if self.last_cmd_time is not None:
            age = float((now - self.last_cmd_time).nanoseconds) / 1e9
            if age > watchdog and self.is_moving:
                self.stop('WATCHDOG_STOP')
        else:
            age = 999.0

        status = MotorStatus()
        status.header.stamp = now.to_msg()
        status.motor_ok = True
        status.last_command = self.last_command
        status.is_moving = bool(self.is_moving)
        status.command_age = float(age)
        status.status_text = 'MOVING' if self.is_moving else self.last_command
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SimMotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop('SHUTDOWN')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
