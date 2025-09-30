#!/usr/bin/env python3
"""
UPS Status Publisher Node
Publishes UPS voltage, battery percentage, charging status, and power adapter status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils.hardware_interface import HardwareInterface
from utils.data_types import create_ups_status_dict
import json
import time
import traceback


class UPSPublisher(Node):
    def __init__(self):
        super().__init__('ups_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'ups_status', 10)
        
        # Initialize hardware interface
        try:
            self.hardware = HardwareInterface(logger=self.get_logger())
            self.get_logger().info('Hardware interface initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize hardware interface: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            raise
        
        # Create timer for publishing at 1Hz (every 1 second)
        self.timer = self.create_timer(1.0, self.publish_ups_status)
        
        self.get_logger().info('UPS Publisher Node has been started')

    def publish_ups_status(self):
        """Publish UPS status message"""
        try:
            # Read UPS data
            voltage, capacity = self.hardware.read_voltage_and_capacity()
            pld_state = self.hardware.get_pld_state()
            
            if voltage is None or capacity is None or pld_state is None:
                self.get_logger().warn('Failed to read UPS data, skipping publish')
                return
            
            # Get charging and power status
            charging_status = self.hardware.get_charging_status(capacity)
            ac_power_status, power_adapter_status = self.hardware.get_power_status(pld_state)
            
            # Create data dictionary
            ups_data = create_ups_status_dict(
                voltage=voltage,
                battery_percentage=capacity,
                charging_status=charging_status,
                ac_power_status=ac_power_status,
                power_adapter_status=power_adapter_status,
                pld_state=bool(pld_state)
            )
            
            # Add timestamp
            ups_data['timestamp'] = time.time()
            
            # Create and publish message
            msg = String()
            msg.data = json.dumps(ups_data)
            self.publisher_.publish(msg)
            
            self.get_logger().debug(
                f'Published UPS status - Voltage: {voltage:.3f}V, '
                f'Battery: {capacity:.2f}%, Charging: {charging_status}, '
                f'AC: {ac_power_status}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing UPS status: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        try:
            self.hardware.cleanup()
        except Exception as e:
            self.get_logger().error(f'Error during cleanup: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        ups_publisher = UPSPublisher()
        rclpy.spin(ups_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        print(f'Traceback: {traceback.format_exc()}')
    finally:
        if 'ups_publisher' in locals():
            ups_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()