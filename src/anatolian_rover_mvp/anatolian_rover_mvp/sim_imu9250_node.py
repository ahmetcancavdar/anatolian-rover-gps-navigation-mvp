import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from anatolian_rover_msgs.msg import ImuStatus

from .math_utils import quaternion_to_yaw_deg


class SimImu9250Node(Node):
    """Publishes simulated IMU9250 data from Gazebo odometry.

    The output topics match the real rover interface: /imu/data and /imu/mag.
    """

    def __init__(self):
        super().__init__('imu9250_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('mag_field_strength', 50e-6)

        self.last_odom = None
        self.last_odom_stamp = None

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, '/imu/mag', 10)
        self.status_pub = self.create_publisher(ImuStatus, '/imu/status', 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self.odom_callback, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.get_logger().info('sim imu9250_node started; waiting for odometry')

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.last_odom_stamp = self.get_clock().now()

    def timer_callback(self):
        now_ros = self.get_clock().now()
        now = now_ros.to_msg()
        imu_ok = self.last_odom is not None

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'

        mag = MagneticField()
        mag.header.stamp = now
        mag.header.frame_id = 'imu_link'

        if imu_ok:
            q = self.last_odom.pose.pose.orientation
            imu.orientation = q
            imu.angular_velocity = self.last_odom.twist.twist.angular
            imu.linear_acceleration.x = 0.0
            imu.linear_acceleration.y = 0.0
            imu.linear_acceleration.z = 9.80665

            # Simple simulated magnetic field from yaw.
            # This is not a physics-accurate magnetometer; it is enough for MVP heading tests.
            yaw_deg = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)
            yaw = math.radians(yaw_deg)
            strength = float(self.get_parameter('mag_field_strength').value)
            mag.magnetic_field.x = strength * math.cos(yaw)
            mag.magnetic_field.y = strength * math.sin(yaw)
            mag.magnetic_field.z = 0.0
        else:
            imu.orientation.w = 1.0

        self.imu_pub.publish(imu)
        self.mag_pub.publish(mag)

        status = ImuStatus()
        status.header.stamp = now
        status.imu_ok = imu_ok
        status.status_text = 'OK' if imu_ok else 'NO_ODOM'
        status.data_age = 999.0 if self.last_odom_stamp is None else float((now_ros - self.last_odom_stamp).nanoseconds) / 1e9
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SimImu9250Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
