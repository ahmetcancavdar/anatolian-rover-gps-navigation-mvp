import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, MagneticField
from anatolian_rover_msgs.msg import HeadingStatus, ImuStatus

from .math_utils import quaternion_to_yaw_deg, wrap360


class HeadingNode(Node):
    """Computes rover heading in degrees.

    In simulation, this node can use the orientation quaternion from /imu/data.
    In real deployment, it can use magnetometer heading after calibration.
    """

    def __init__(self):
        super().__init__('heading_node')
        self.declare_parameter('use_imu_orientation', True)
        self.declare_parameter('mag_mount_offset_deg', 0.0)
        self.declare_parameter('magnetic_declination_deg', 0.0)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('heading_timeout', 1.0)

        self.last_imu = None
        self.last_mag = None
        self.last_imu_status = None
        self.last_update_time = None
        self.prev_heading = None
        self.prev_heading_time = None
        self.heading_rate = 0.0

        self.heading_pub = self.create_publisher(Float32, '/heading/yaw', 10)
        self.status_pub = self.create_publisher(HeadingStatus, '/heading/status', 10)

        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(MagneticField, '/imu/mag', self.mag_callback, 10)
        self.create_subscription(ImuStatus, '/imu/status', self.imu_status_callback, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.get_logger().info('heading_node started')

    def imu_callback(self, msg: Imu):
        self.last_imu = msg
        self.last_update_time = self.get_clock().now()

    def mag_callback(self, msg: MagneticField):
        self.last_mag = msg
        self.last_update_time = self.get_clock().now()

    def imu_status_callback(self, msg: ImuStatus):
        self.last_imu_status = msg

    def compute_heading(self):
        use_orientation = bool(self.get_parameter('use_imu_orientation').value)
        mount_offset = float(self.get_parameter('mag_mount_offset_deg').value)
        declination = float(self.get_parameter('magnetic_declination_deg').value)

        if use_orientation and self.last_imu is not None:
            q = self.last_imu.orientation
            # Convert ROS yaw to compass heading convention.
            # ROS yaw: 0 along x-axis, CCW positive.
            # For a simple simulation where Gazebo x is East and y is North,
            # compass heading can be adapted later if needed. Here we use yaw directly as MVP.
            yaw_ros_deg = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)

            # ROS/Gazebo yaw convention:
            #   yaw = 0 deg   -> +X direction, usually East
            #   yaw = 90 deg  -> +Y direction, usually North
            #
            # Rover navigation heading convention:
            #   heading = 0 deg   -> North
            #   heading = 90 deg  -> East
            #   heading = 180 deg -> South
            #   heading = 270 deg -> West
            #
            # Therefore:
            #   compass_heading = 90 - ros_yaw
            heading = wrap360(90.0 - yaw_ros_deg)

            return wrap360(heading + mount_offset + declination)

        if self.last_mag is None:
            return None

        mx = self.last_mag.magnetic_field.x
        my = self.last_mag.magnetic_field.y
        # Simple 2D compass heading. Axis sign may need adjustment on the real rover.
        heading = math.degrees(math.atan2(my, mx))
        return wrap360(heading + mount_offset + declination)

    def timer_callback(self):
        now_ros = self.get_clock().now()
        heading = self.compute_heading()
        timeout = float(self.get_parameter('heading_timeout').value)
        data_age = 999.0 if self.last_update_time is None else float((now_ros - self.last_update_time).nanoseconds) / 1e9
        heading_ok = heading is not None and data_age <= timeout

        if heading is None:
            heading = 0.0

        if self.prev_heading is not None and self.prev_heading_time is not None:
            dt = float((now_ros - self.prev_heading_time).nanoseconds) / 1e9
            if dt > 1e-6:
                delta = (heading - self.prev_heading + 180.0) % 360.0 - 180.0
                self.heading_rate = delta / dt

        self.prev_heading = heading
        self.prev_heading_time = now_ros

        msg = Float32()
        msg.data = float(heading)
        self.heading_pub.publish(msg)

        status = HeadingStatus()
        status.header.stamp = now_ros.to_msg()
        status.heading_ok = bool(heading_ok)
        status.heading_deg = float(heading)
        status.heading_rate_deg_s = float(self.heading_rate)
        status.data_age = float(data_age)
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = HeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
