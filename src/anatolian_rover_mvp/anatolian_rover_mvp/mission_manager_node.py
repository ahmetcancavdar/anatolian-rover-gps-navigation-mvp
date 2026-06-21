import rclpy
from rclpy.node import Node

from anatolian_rover_msgs.msg import GeoGoal, MissionState, NavStatus, MotorStatus


class MissionManagerNode(Node):
    """Publishes the active GPS goal and mission state.

    This first MVP version supports a single target from ROS parameters.
    Later, this node can be extended to read goal lists, RSCP commands, or UI commands.
    """

    def __init__(self):
        super().__init__('mission_manager_node')
        self.declare_parameter('target_latitude', 39.7760)
        self.declare_parameter('target_longitude', 30.5200)
        self.declare_parameter('target_altitude', 0.0)
        self.declare_parameter('acceptance_radius', 2.0)
        self.declare_parameter('goal_id', 'goal_1')
        self.declare_parameter('autonomous_enabled', True)
        self.declare_parameter('publish_rate_hz', 1.0)

        self.target_reached = False
        self.last_nav_state = 'WAITING'
        self.last_motor_status = 'UNKNOWN'

        self.goal_pub = self.create_publisher(GeoGoal, '/mission/current_goal', 10)
        self.state_pub = self.create_publisher(MissionState, '/mission/state', 10)

        self.create_subscription(NavStatus, '/nav/status', self.nav_status_callback, 10)
        self.create_subscription(MotorStatus, '/motor/status', self.motor_status_callback, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.get_logger().info('mission_manager_node started')

    def nav_status_callback(self, msg: NavStatus):
        self.target_reached = bool(msg.target_reached)
        self.last_nav_state = msg.nav_state

    def motor_status_callback(self, msg: MotorStatus):
        self.last_motor_status = msg.status_text

    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        goal = GeoGoal()
        goal.header.stamp = now
        goal.header.frame_id = 'wgs84'
        goal.goal_id = str(self.get_parameter('goal_id').value)
        goal.latitude = float(self.get_parameter('target_latitude').value)
        goal.longitude = float(self.get_parameter('target_longitude').value)
        goal.altitude = float(self.get_parameter('target_altitude').value)
        goal.acceptance_radius = float(self.get_parameter('acceptance_radius').value)
        self.goal_pub.publish(goal)

        state = MissionState()
        state.header.stamp = now
        state.state = 'TARGET_REACHED' if self.target_reached else 'NAVIGATING'
        state.active_goal_id = goal.goal_id
        state.autonomous_enabled = bool(self.get_parameter('autonomous_enabled').value)
        state.target_reached = self.target_reached
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
