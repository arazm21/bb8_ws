#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from sensor_msgs.msg import Joy

class JoystickMotorPublisher(Node):
    def __init__(self):
        super().__init__('joystick_motor_publisher')
        self.publisher = self.create_publisher(Int8MultiArray, 'motor_direction', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

    def joy_callback(self, msg: Joy):
        # Left stick Y = forward/backward, X = left/right
        y = msg.axes[1]   # forward = +1, backward = -1
        x = msg.axes[0]   # right = +1, left = -1

        # Arcade drive mixing
        left_power  = y + x
        right_power = y - x

        # Clip to [-1, 1]
        left_power  = max(min(left_power, 1.0), -1.0)
        right_power = max(min(right_power, 1.0), -1.0)

        # Convert to direction + speed (0–100)
        left_dir  = 1 if left_power >= 0 else -1
        right_dir = 1 if right_power >= 0 else -1
        left_speed  = int(abs(left_power)  * 100)
        right_speed = int(abs(right_power) * 100)

        # Publish as [left_dir, left_speed, right_dir, right_speed]
        output = Int8MultiArray()
        output.data = [left_dir, left_speed, right_dir, right_speed]
        self.publisher.publish(output)

        self.get_logger().info(
            f'Published: L(dir={left_dir}, speed={left_speed}), '
            f'R(dir={right_dir}, speed={right_speed})'
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
