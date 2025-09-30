#!/usr/bin/env python3
"""
Power Monitor Subscriber Node
Subscribes to both UPS and RPi status messages, monitors power conditions,
and provides warnings/shutdown logic based on power status
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os
import time

# Add the package directory to the Python path
package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, package_dir)

from utils.hardware_interface import PowerManager
from utils.data_types import validate_ups_data, validate_rpi_data, SYSTEM_THRESHOLDS

import json
import traceback


class PowerMonitorSubscriber(Node):
    def __init__(self):
        super().__init__('power_monitor_subscriber')
        
        # Initialize power manager
        self.power_manager = PowerManager(logger=self.get_logger())
        
        # Create subscribers
        self.ups_subscription = self.create_subscription(
            String,
            'ups_status',
            self.ups_status_callback,
            10
        )
        
        self.rpi_subscription = self.create_subscription(
            String,
            'rpi_status',
            self.rpi_status_callback,
            10
        )
        
        # Store latest messages
        self.latest_ups_status = None
        self.latest_rpi_status = None
        
        # Create timer for periodic status display (every 30 seconds)
        self.display_timer = self.create_timer(30.0, self.display_combined_status)
        
        self.get_logger().info('Power Monitor Subscriber Node has been started')

    def ups_status_callback(self, msg):
        """Callback for UPS status messages"""
        try:
            # Parse JSON data
            ups_data = json.loads(msg.data)
            
            if not validate_ups_data(ups_data):
                self.get_logger().warn('Invalid UPS status data received')
                return
                
            self.latest_ups_status = ups_data
            
            # Evaluate power status and check for critical conditions
            warning, critical = self.power_manager.evaluate_power_status(
                ups_data['pld_state'], ups_data['battery_percentage']
            )
            
            # Only log warnings occasionally to reduce spam (every 10 seconds instead of every second)
            current_time = time.time()
            if not hasattr(self, '_last_warning_time'):
                self._last_warning_time = 0
            
            if warning and (current_time - self._last_warning_time) > 20:
                self._last_warning_time = current_time
                # Enhanced warning message with power state info
                enhanced_warning = f"{ups_data['ac_power_status']}, Voltage: {ups_data['voltage']:.3f}V"
                if ups_data.get('additional_info'):
                    enhanced_warning += f"\n{ups_data['additional_info']}"
                enhanced_warning += f"\n{warning}"
                
                if critical:
                    self.get_logger().error(f'CRITICAL POWER STATUS:\n{enhanced_warning}')
                else:
                    self.get_logger().warn(f'POWER STATUS:\n{enhanced_warning}')
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse UPS status JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing UPS status: {e}')

    def rpi_status_callback(self, msg):
        """Callback for RPi status messages"""
        try:
            # Parse JSON data
            rpi_data = json.loads(msg.data)
            
            if not validate_rpi_data(rpi_data):
                self.get_logger().warn('Invalid RPi status data received')
                return
                
            self.latest_rpi_status = rpi_data
            
            # Check for system warnings
            if rpi_data['cpu_temp'] > SYSTEM_THRESHOLDS['cpu_temp_critical']:
                self.get_logger().error(f'CRITICAL CPU temperature: {rpi_data["cpu_temp"]:.1f}°C')
            elif rpi_data['cpu_temp'] > SYSTEM_THRESHOLDS['cpu_temp_warning']:
                self.get_logger().warn(f'High CPU temperature: {rpi_data["cpu_temp"]:.1f}°C')
            
            if rpi_data['input_voltage'] < SYSTEM_THRESHOLDS['input_voltage_min']:
                self.get_logger().warn(f'Low input voltage: {rpi_data["input_voltage"]:.3f}V')
            
            if rpi_data['system_watts'] > SYSTEM_THRESHOLDS['power_consumption_warning']:
                self.get_logger().warn(f'High power consumption: {rpi_data["system_watts"]:.3f}W')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse RPi status JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing RPi status: {e}')

    def display_combined_status(self):
        """Display combined status periodically"""
        try:
            if self.latest_ups_status is None or self.latest_rpi_status is None:
                self.get_logger().info('Waiting for status messages from both publishers...')
                return

            ups = self.latest_ups_status
            rpi = self.latest_rpi_status

            # Create formatted status display  
            status_msg = f"""
========== X120x UPS Status ==========
UPS Voltage: {ups['voltage']:.3f}V
Battery: {ups['battery_percentage']:.3f}%
Charging: {ups['charging_status']}

========== RPi5 System Stats ==========
Input Voltage: {rpi['input_voltage']:.3f}V
CPU Volts: {rpi['cpu_volts']:.3f}V
CPU Amps: {rpi['cpu_amps']:.3f}A
System Watts: {rpi['system_watts']:.3f}W
CPU Temp: {rpi['cpu_temp']:.1f}°C
Fan RPM: {rpi['fan_rpm']}

========== Power Status ==========
{ups['ac_power_status']}, Voltage: {ups['voltage']:.3f}V
{ups['power_adapter_status']}"""

            # Add additional info if available (power loss message)
            if ups.get('additional_info'):
                status_msg += f"\n{ups['additional_info']}"
            
            # Add current warning if on battery power
            if ups['pld_state'] != 1:  # Power loss detected
                if ups['battery_percentage'] >= 51:
                    warning_msg = f"WARNING: Running on UPS Backup Power | Batteries @ {ups['battery_percentage']:.2f}%"
                elif ups['battery_percentage'] <= 50 and ups['battery_percentage'] >= 25:
                    warning_msg = f"WARNING: UPS Power levels approaching critical | Batteries @ {ups['battery_percentage']:.2f}%"
                elif ups['battery_percentage'] <= 24:
                    warning_msg = f"CRITICAL: UPS Power levels critical | Batteries @ {ups['battery_percentage']:.2f}%"
                status_msg += f"\n{warning_msg}"
            
            status_msg += "\n======================================"

            self.get_logger().info(status_msg)
            
            # Check overall system health
            self._check_system_health(ups, rpi)

        except Exception as e:
            self.get_logger().error(f'Error displaying combined status: {e}')
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def _check_system_health(self, ups_data, rpi_data):
        """Check overall system health and provide recommendations"""
        issues = []
        
        # UPS-related checks - fix the pld_state logic to match original
        if ups_data['battery_percentage'] < 20 and ups_data['pld_state'] != 1:  # != 1 means on battery
            issues.append(f"Critical battery level ({ups_data['battery_percentage']:.1f}%) on backup power")
        
        if ups_data['voltage'] < 3.0:
            issues.append(f"Low UPS voltage ({ups_data['voltage']:.3f}V)")
        
        # System-related checks
        if rpi_data['cpu_temp'] > SYSTEM_THRESHOLDS['cpu_temp_warning']:
            issues.append(f"High CPU temperature ({rpi_data['cpu_temp']:.1f}°C)")
        
        if rpi_data['input_voltage'] < SYSTEM_THRESHOLDS['input_voltage_min']:
            issues.append(f"Low input voltage ({rpi_data['input_voltage']:.3f}V)")
        
        if rpi_data['system_watts'] > SYSTEM_THRESHOLDS['power_consumption_warning']:
            issues.append(f"High power consumption ({rpi_data['system_watts']:.3f}W)")
        
        # Log issues
        if issues:
            self.get_logger().warn("System Health Issues Detected:")
            for issue in issues:
                self.get_logger().warn(f"  - {issue}")
        else:
            self.get_logger().debug("System health: All parameters normal")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        power_monitor = PowerMonitorSubscriber()
        rclpy.spin(power_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
        print(f'Traceback: {traceback.format_exc()}')
    finally:
        if 'power_monitor' in locals():
            power_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()