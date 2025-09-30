#!/usr/bin/env python3
"""
Hardware interface utilities for X1200 UPS monitoring
Based on original qtx120xTerminal.py
Using singleton pattern to prevent GPIO conflicts
"""

import sys
import struct
from pathlib import Path
from subprocess import check_output, CalledProcessError, call
import smbus2
from gpiozero import InputDevice, Button
import time
import rclpy

# Constants
CHG_ONOFF_PIN = 16
PLD_BUTTON_PIN = 6
I2C_BUS = 1
UPS_I2C_ADDRESS = 0x36

class HardwareInterface:
    _instance = None
    _initialized = False
    
    def __new__(cls, logger=None):
        if cls._instance is None:
            cls._instance = super(HardwareInterface, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, logger=None):
        if self._initialized:
            return
            
        self.logger = logger
        try:
            self.bus = smbus2.SMBus(I2C_BUS)
            self.pld_button = Button(PLD_BUTTON_PIN)
            self._initialized = True
            if self.logger:
                self.logger.info("Hardware interface initialized (singleton)")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to initialize hardware interface: {e}")
            raise

    def read_voltage_and_capacity(self):
        """Read UPS voltage and battery capacity via I2C"""
        try:
            voltage_read = self.bus.read_word_data(UPS_I2C_ADDRESS, 2)
            capacity_read = self.bus.read_word_data(UPS_I2C_ADDRESS, 4)
            
            voltage_swapped = struct.unpack("<H", struct.pack(">H", voltage_read))[0]
            voltage = voltage_swapped * 1.25 / 1000 / 16
            
            capacity_swapped = struct.unpack("<H", struct.pack(">H", capacity_read))[0]
            capacity = capacity_swapped / 256
            
            return voltage, capacity
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error reading UPS voltage/capacity: {e}")
            return None, None

    def get_pld_state(self):
        """Get Power Loss Detection state"""
        try:
            # From original qtx120xTerminal.py: return 0 if PLD_BUTTON.is_pressed else 1
            # 0 = Power Loss (on battery), 1 = AC Power OK
            return 0 if self.pld_button.is_pressed else 1
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error reading PLD state: {e}")
            return None

    def read_hardware_metric(self, command_args, strip_chars):
        """Generic function to read hardware metrics using vcgencmd"""
        try:
            output = check_output(command_args).decode("utf-8")
            metric_str = output.split("=")[1].strip().rstrip(strip_chars)
            return float(metric_str)
        except (CalledProcessError, ValueError) as e:
            if self.logger:
                self.logger.error(f"Error reading hardware metric {command_args}: {e}")
            return None

    def read_cpu_volts(self):
        """Read CPU core voltage"""
        return self.read_hardware_metric(["vcgencmd", "pmic_read_adc", "VDD_CORE_V"], 'V')

    def read_cpu_amps(self):
        """Read CPU core current"""
        return self.read_hardware_metric(["vcgencmd", "pmic_read_adc", "VDD_CORE_A"], 'A')

    def read_cpu_temp(self):
        """Read CPU temperature"""
        return self.read_hardware_metric(["vcgencmd", "measure_temp"], "'C")

    def read_input_voltage(self):
        """Read external 5V input voltage"""
        return self.read_hardware_metric(["vcgencmd", "pmic_read_adc", "EXT5V_V"], 'V')

    def get_fan_rpm(self):
        """Read fan RPM from sysfs"""
        try:
            sys_devices_path = Path('/sys/devices/platform/cooling_fan')
            fan_input_files = list(sys_devices_path.rglob('fan1_input'))
            if not fan_input_files:
                return "No fan?"
            with open(fan_input_files[0], 'r') as file:
                rpm = file.read().strip()
            return f"{rpm} RPM"
        except FileNotFoundError:
            return "Fan RPM file not found"
        except PermissionError:
            return "Permission denied accessing the fan RPM file"
        except Exception as e:
            return f"Unexpected error: {e}"

    def power_consumption_watts(self):
        """Calculate total system power consumption"""
        try:
            output = check_output(['vcgencmd', 'pmic_read_adc']).decode("utf-8")
            lines = output.split('\n')
            amperages = {}
            voltages = {}
            
            for line in lines:
                cleaned_line = line.strip()
                if cleaned_line:
                    parts = cleaned_line.split(' ')
                    label, value = parts[0], parts[-1]
                    val = float(value.split('=')[1][:-1])
                    short_label = label[:-2]
                    if label.endswith('A'):
                        amperages[short_label] = val
                    else:
                        voltages[short_label] = val
                        
            wattage = sum(amperages[key] * voltages[key] for key in amperages if key in voltages)
            return wattage
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error calculating power consumption: {e}")
            return None

    def get_charging_status(self, capacity):
        """Determine charging status and control charging - matches original logic"""
        try:
            if capacity >= 90:
                charge_status = "disabled"
                InputDevice(CHG_ONOFF_PIN, pull_up=True)
            else:
                charge_status = "enabled"
                InputDevice(CHG_ONOFF_PIN, pull_up=False)
            return charge_status
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error controlling charging: {e}")
            return "unknown"

    def get_power_status(self, pld_state):
        """Get power adapter and AC power status - enhanced with merged.py style messages"""
        if pld_state == 1:
            ac_status = "AC Power State: Plugged"
            adapter_status = "Power Adapter: OK!"
            additional_info = ""
        else:
            ac_status = "AC Power State: Unplugged"  
            adapter_status = "Power Adapter: FAIL!"
            additional_info = "UPS is unplugged or AC power loss detected."
        
        return ac_status, adapter_status, additional_info

    def cleanup(self):
        """Cleanup hardware resources"""
        try:
            if hasattr(self, 'bus'):
                self.bus.close()
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error during cleanup: {e}")

    @classmethod
    def reset_instance(cls):
        """Reset singleton instance (for testing)"""
        cls._instance = None
        cls._initialized = False


class PowerManager:
    """Manages power-related operations and shutdown logic"""
    
    def __init__(self, logger=None):
        self.logger = logger
        self.shutdown_initiated = False
        
    def evaluate_power_status(self, pld_state, capacity):
        """Evaluate power status and return warning messages - matches original logic"""
        warning = ""
        critical = False
        
        # From original: pld_state == 1 means AC OK, pld_state == 0 means power loss
        if pld_state != 1 and capacity >= 51:
            warning = f"Running on UPS Backup Power | Batteries @ {capacity:.2f}%"
        elif pld_state != 1 and capacity <= 50 and capacity >= 25:
            warning = f"UPS Power levels approaching critical | Batteries @ {capacity:.2f}%"
            critical = False
        elif pld_state != 1 and capacity <= 24 and capacity >= 16:
            warning = f"UPS Power levels critical | Batteries @ {capacity:.2f}%"
            critical = True
        elif pld_state != 1 and capacity <= 15 and not self.shutdown_initiated:
            self.shutdown_initiated = True
            warning = "UPS Power failure imminent! Auto shutdown in 5 minutes!"
            critical = True
            self._initiate_shutdown()
        elif pld_state != 1 and self.shutdown_initiated:
            warning = "UPS Power failure imminent! Auto shutdown to occur within 5 minutes!"
            critical = True
        elif pld_state == 1 and self.shutdown_initiated:
            self._cancel_shutdown()
            warning = "AC Power has been restored. Auto shutdown has been cancelled!"
            self.shutdown_initiated = False
            critical = False
        
        return warning, critical
    
    def _initiate_shutdown(self):
        """Initiate system shutdown"""
        try:
            call("sudo shutdown -P +5 'Power failure, shutdown in 5 minutes.'", shell=True)
            if self.logger:
                self.logger.critical("System shutdown initiated due to power failure!")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to initiate shutdown: {e}")
    
    def _cancel_shutdown(self):
        """Cancel system shutdown"""
        try:
            call("sudo shutdown -c 'Shutdown is cancelled'", shell=True)
            if self.logger:
                self.logger.info("System shutdown cancelled - AC power restored")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to cancel shutdown: {e}")