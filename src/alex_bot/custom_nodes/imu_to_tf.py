#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
import tf2_ros
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')

        # TF broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber for IMU data
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.imu_sub = self.create_subscription(
            Float32MultiArray,
            '/imu_data',
            self.imu_callback,
            qos_profile
)


    def imu_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 6:
            return

        # Extract roll, pitch, yaw (they appear to be in degrees, convert to radians)
        roll = math.radians(msg.data[3])
        pitch = math.radians(msg.data[4])
        yaw = math.radians(msg.data[5])

        # Convert roll, pitch, yaw → quaternion
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        # Build and publish transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.3

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.broadcaster.sendTransform(t)
        self.get_logger().info(
            f"IMU RPY=({msg.data[3]:.1f}°, {msg.data[4]:.1f}°, {msg.data[5]:.1f}°)"
        )


def main():
    rclpy.init()
    node = RobotMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
