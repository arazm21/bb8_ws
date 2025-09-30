#!/usr/bin/env python3
"""
Raspberry Pi System Status Publisher Node
Publishes RPi system statistics including voltage, current, temperature, and fan RPM
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from utils.hardware_interface import HardwareInterface
from utils.data_types import create_rpi_status_dict
import json
import time
import traceback


class RPiPublisher(Node):
    def __init__(self):
        super().__init__('rpi_publisher')
        
        # Create publisher
        self.publisher_ = self.create_publisher(String, 'rpi_status', 10)
        
        # Initialize hardware interface
        try:
            self.hardware = HardwareInterface(logger=self.get_logger())
            self.get_logger().info('Hardware interface initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize hardware interface: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            raise
        
        # Create timer for publishing at 1Hz (every 1 second)
        self.timer = self.create_timer(1.0, self.publish_rpi_status)
        
        self.get_logger().info('RPi Publisher Node has been started')

    def publish_rpi_status(self):
        """Publish Raspberry Pi system status message"""
        try:
            # Read system data
            input_voltage = self.hardware.read_input_voltage()
            cpu_volts = self.hardware.read_cpu_volts()
            cpu_amps = self.hardware.read_cpu_amps()
            cpu_temp = self.hardware.read_cpu_temp()
            fan_rpm = self.hardware.get_fan_rpm()
            system_watts = self.hardware.power_consumption_watts()
            
            # Handle None values
            if any(val is None for val in [input_voltage, cpu_volts, cpu_amps, cpu_temp, system_watts]):
                self.get_logger().warn('Some system metrics could not be read, using defaults')
            
            # Create data dictionary
            rpi_data = create_rpi_status_dict(
                input_voltage=input_voltage if input_voltage is not None else 0.0,
                cpu_volts=cpu_volts if cpu_volts is not None else 0.0,
                cpu_amps=cpu_amps if cpu_amps is not None else 0.0,
                system_watts=system_watts if system_watts is not None else 0.0,
                cpu_temp=cpu_temp if cpu_temp is not None else 0.0,
                fan_rpm=str(fan_rpm) if fan_rpm is not None else "Unknown"
            )
            
            # Add timestamp
            rpi_data['timestamp'] = time.time()
            
            # Create and publish message
            msg = String()
            msg.data = json.dumps(rpi_data)
            self.publisher_.publish(msg)
            
            self.get_logger().debug(
                f'Published RPi status - Input: {rpi_data["input_voltage"]:.3f}V, '
                f'CPU: {rpi_data["cpu_volts"]:.3f}V/{rpi_data["cpu_amps"]:.3f}A, '
                f'Temp: {rpi_data["cpu_temp"]:.1f}°C, Fan: {rpi_data["fan_rpm"]}, '
                f'Power: {rpi_data["system_watts"]:.3f}W'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing RPi status: {e}')
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
        rpi_publisher = RPiPublisher()
        rclpy.spin(rpi_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        print(f'Traceback: {traceback.format_exc()}')
    finally:
        if 'rpi_publisher' in locals():
            rpi_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()