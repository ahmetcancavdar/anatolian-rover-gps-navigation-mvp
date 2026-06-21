import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from anatolian_rover_msgs.msg import GpsStatus

from .math_utils import xy_to_latlon


class SimGpsDataNode(Node):
    """Converts Gazebo /odom pose into a simulated RTK GPS fix.

    In the real rover, gps_data_node will read LC29HEA over serial/USB.
    This simulation adapter keeps the same output interface: /gps/fix and /gps/status.
    """

    def __init__(self):
        super().__init__('gps_data_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('reference_latitude', 39.7760)
        self.declare_parameter('reference_longitude', 30.5200)
        self.declare_parameter('reference_altitude', 0.0)
        self.declare_parameter('gps_noise_std_m', 0.0)
        self.declare_parameter('satellite_count', 18)
        self.declare_parameter('horizontal_accuracy', 0.02)
        self.declare_parameter('fix_type', 'RTK_FIXED')
        self.declare_parameter('publish_rate_hz', 5.0)

        self.last_odom = None
        self.last_odom_stamp = None

        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.status_pub = self.create_publisher(GpsStatus, '/gps/status', 10)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value), self.odom_callback, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.get_logger().info('sim gps_data_node started; waiting for odometry')

    def odom_callback(self, msg: Odometry):
        self.last_odom = msg
        self.last_odom_stamp = self.get_clock().now()

    def timer_callback(self):
        now_ros = self.get_clock().now()
        now = now_ros.to_msg()
        gps_ok = self.last_odom is not None

        fix = NavSatFix()
        fix.header.stamp = now
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX if gps_ok else NavSatStatus.STATUS_NO_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        if gps_ok:
            x = self.last_odom.pose.pose.position.x
            y = self.last_odom.pose.pose.position.y
            z = self.last_odom.pose.pose.position.z
            lat0 = float(self.get_parameter('reference_latitude').value)
            lon0 = float(self.get_parameter('reference_longitude').value)
            alt0 = float(self.get_parameter('reference_altitude').value)
            lat, lon = xy_to_latlon(x, y, lat0, lon0)
            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = alt0 + z
            acc = float(self.get_parameter('horizontal_accuracy').value)
            fix.position_covariance = [acc * acc, 0.0, 0.0,
                                       0.0, acc * acc, 0.0,
                                       0.0, 0.0, acc * acc]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        else:
            fix.latitude = float('nan')
            fix.longitude = float('nan')
            fix.altitude = float('nan')
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(fix)

        status = GpsStatus()
        status.header.stamp = now
        status.fix_type = str(self.get_parameter('fix_type').value) if gps_ok else 'NO_FIX'
        status.gps_ok = gps_ok
        status.horizontal_accuracy = float(self.get_parameter('horizontal_accuracy').value) if gps_ok else 999.0
        status.satellite_count = int(self.get_parameter('satellite_count').value) if gps_ok else 0
        if self.last_odom_stamp is None:
            status.data_age = 999.0
        else:
            status.data_age = float((now_ros - self.last_odom_stamp).nanoseconds) / 1e9
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = SimGpsDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
