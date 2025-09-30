#!/usr/bin/env python3
"""
Combined Power Monitor Publisher Node
Publishes both UPS and RPi system status from a single hardware interface
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os

# Add the package directory to the Python path
package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, package_dir)

from utils.hardware_interface import HardwareInterface
from utils.data_types import create_ups_status_dict, create_rpi_status_dict

import json
import time
import traceback


class CombinedPublisher(Node):
    def __init__(self):
        super().__init__('combined_publisher')
        
        # Create publishers
        self.ups_publisher = self.create_publisher(String, 'ups_status', 10)
        self.rpi_publisher = self.create_publisher(String, 'rpi_status', 10)
        
        # Initialize hardware interface (only once!)
        try:
            self.hardware = HardwareInterface(logger=self.get_logger())
            self.get_logger().info('Hardware interface initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize hardware interface: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            raise
        
        # Create timers for publishing at 1Hz (every 1 second)
        self.timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Combined Publisher Node has been started')

    def publish_status(self):
        """Publish both UPS and RPi status messages"""
        try:
            # Publish UPS status
            self.publish_ups_status()
            
            # Publish RPi status
            self.publish_rpi_status()
            
        except Exception as e:
            self.get_logger().error(f'Error in publish_status: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def publish_ups_status(self):
        """Publish UPS status message"""
        try:
            # Read UPS data
            voltage, capacity = self.hardware.read_voltage_and_capacity()
            pld_state = self.hardware.get_pld_state()
            
            if voltage is None or capacity is None or pld_state is None:
                self.get_logger().warn('Failed to read UPS data, skipping UPS publish')
                return
            
            # Get charging and power status
            charging_status = self.hardware.get_charging_status(capacity)
            ac_power_status, power_adapter_status, additional_info = self.hardware.get_power_status(pld_state)
            
            # Create data dictionary
            ups_data = create_ups_status_dict(
                voltage=voltage,
                battery_percentage=capacity,
                charging_status=charging_status,
                ac_power_status=ac_power_status,
                power_adapter_status=power_adapter_status,
                pld_state=bool(pld_state)
            )
            
            # Add additional fields for enhanced messaging
            ups_data['additional_info'] = additional_info
            
            # Add timestamp
            ups_data['timestamp'] = time.time()
            
            # Create and publish message
            msg = String()
            msg.data = json.dumps(ups_data)
            self.ups_publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published UPS status - Voltage: {voltage:.3f}V, '
                f'Battery: {capacity:.2f}%, Charging: {charging_status}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing UPS status: {e}')

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
            self.rpi_publisher.publish(msg)
            
            self.get_logger().debug(
                f'Published RPi status - Input: {rpi_data["input_voltage"]:.3f}V, '
                f'CPU: {rpi_data["cpu_volts"]:.3f}V/{rpi_data["cpu_amps"]:.3f}A, '
                f'Temp: {rpi_data["cpu_temp"]:.1f}°C, Power: {rpi_data["system_watts"]:.3f}W'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing RPi status: {e}')

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
        combined_publisher = CombinedPublisher()
        rclpy.spin(combined_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        print(f'Traceback: {traceback.format_exc()}')
    finally:
        if 'combined_publisher' in locals():
            combined_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()