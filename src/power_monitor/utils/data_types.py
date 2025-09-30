#!/usr/bin/env python3
"""
Data types and structures for power monitoring
Using dictionaries that can be serialized to JSON strings for ROS2 publishing
"""

def create_ups_status_dict(voltage=0.0, battery_percentage=0.0, charging_status="unknown", 
                          ac_power_status="unknown", power_adapter_status="unknown", pld_state=False):
    """
    Create UPS status dictionary
    
    Args:
        voltage (float): UPS voltage in V
        battery_percentage (float): Battery percentage (0-100)
        charging_status (str): "enabled", "disabled", or "unknown"
        ac_power_status (str): AC power status message
        power_adapter_status (str): Power adapter status message
        pld_state (bool): Power Loss Detection state (True = power OK, False = power loss)
    
    Returns:
        dict: UPS status data
    """
    return {
        'voltage': float(voltage),
        'battery_percentage': float(battery_percentage),
        'charging_status': str(charging_status),
        'ac_power_status': str(ac_power_status),
        'power_adapter_status': str(power_adapter_status),
        'pld_state': bool(pld_state),
        'additional_info': '',  # For extra messages like "UPS is unplugged..."
        'timestamp': None  # Will be set when publishing
    }


def create_rpi_status_dict(input_voltage=0.0, cpu_volts=0.0, cpu_amps=0.0, 
                          system_watts=0.0, cpu_temp=0.0, fan_rpm="unknown"):
    """
    Create RPi system status dictionary
    
    Args:
        input_voltage (float): External 5V input voltage
        cpu_volts (float): CPU core voltage
        cpu_amps (float): CPU core current
        system_watts (float): Total system power consumption
        cpu_temp (float): CPU temperature in °C
        fan_rpm (str): Fan RPM reading
    
    Returns:
        dict: RPi system status data
    """
    return {
        'input_voltage': float(input_voltage),
        'cpu_volts': float(cpu_volts),
        'cpu_amps': float(cpu_amps),
        'system_watts': float(system_watts),
        'cpu_temp': float(cpu_temp),
        'fan_rpm': str(fan_rpm),
        'timestamp': None  # Will be set when publishing
    }


def create_combined_status_dict(ups_data=None, rpi_data=None):
    """
    Create combined status dictionary for comprehensive monitoring
    
    Args:
        ups_data (dict): UPS status dictionary
        rpi_data (dict): RPi status dictionary
    
    Returns:
        dict: Combined status data
    """
    return {
        'ups_status': ups_data or create_ups_status_dict(),
        'rpi_status': rpi_data or create_rpi_status_dict(),
        'timestamp': None  # Will be set when publishing
    }


# UPS status keys including additional info
UPS_STATUS_KEYS = [
    'voltage', 'battery_percentage', 'charging_status', 
    'ac_power_status', 'power_adapter_status', 'pld_state', 
    'additional_info', 'timestamp'
]

RPI_STATUS_KEYS = [
    'input_voltage', 'cpu_volts', 'cpu_amps', 
    'system_watts', 'cpu_temp', 'fan_rpm', 'timestamp'
]

# Battery level thresholds (you can adjust these)
BATTERY_THRESHOLDS = {
    'critical_shutdown': 12.0,      # Reduced from 15% - only shutdown when very low
    'critical_warning': 22.0,       # Reduced from 24%
    'low_warning': 45.0,            # Reduced from 50%
    'charging_disable': 90.0
}

# System health thresholds
SYSTEM_THRESHOLDS = {
    'cpu_temp_warning': 70.0,
    'cpu_temp_critical': 80.0,
    'input_voltage_min': 4.8,
    'power_consumption_warning': 10.0
}


def validate_ups_data(data):
    """
    Validate UPS status data dictionary
    
    Args:
        data (dict): UPS status data
    
    Returns:
        bool: True if valid, False otherwise
    """
    if not isinstance(data, dict):
        return False
    
    required_keys = ['voltage', 'battery_percentage', 'pld_state']
    return all(key in data for key in required_keys)


def validate_rpi_data(data):
    """
    Validate RPi status data dictionary
    
    Args:
        data (dict): RPi status data
    
    Returns:
        bool: True if valid, False otherwise
    """
    if not isinstance(data, dict):
        return False
    
    required_keys = ['input_voltage', 'cpu_volts', 'cpu_temp']
    return all(key in data for key in required_keys)