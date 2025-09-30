#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher = self.create_publisher(Int8MultiArray, 'motor_direction', 10)
        self.timer = self.create_timer(0.3, self.publish_command)
        self.current_speed = 0
        self.target_speed = 100
        self.direction = 1  # 1 = forward, -1 = reverse

    def publish_command(self):
        if self.current_speed < self.target_speed:
            self.current_speed += 5
        elif self.current_speed > self.target_speed:
            self.current_speed -= 5

        # Create the message with direction and speed
        msg = Int8MultiArray()
        msg.data = [self.direction, self.current_speed]  # direction and speed as int8 values

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: dir={msg.data[0]}, speed={msg.data[1]}')

        # Flip direction once full speed reached in one direction and back to 0
        if self.current_speed == self.target_speed and self.target_speed == 100:
            self.target_speed = 0
        elif self.current_speed == self.target_speed and self.target_speed == 0:
            self.direction *= -1
            self.target_speed = 100

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String
# import time

# class MotorCommandPublisher(Node):
#     def __init__(self):
#         super().__init__('motor_command_publisher')
#         self.publisher = self.create_publisher(String, 'motor_direction', 10)
#         self.timer = self.create_timer(5.0, self.publish_command)
#         self.state = 0  # 0: forward, 1: reverse

#     def publish_command(self):
#         msg = String()
#         if self.state == 0:
#             msg.data = 'FORWARD'
#             self.state = 1
#         else:
#             msg.data = 'REVERSE'
#             self.state = 0
#         self.publisher.publish(msg)
#         self.get_logger().info(f'Published: {msg.data}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = MotorCommandPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
