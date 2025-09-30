from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'power_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'smbus2',
        'gpiozero',
    ],
    zip_safe=True,
    maintainer='bb8',
    maintainer_email='bb8@example.com',
    description='Power monitoring package for X1200 UPS and Raspberry Pi 5',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ups_publisher = power_monitor.ups_publisher:main',
            'rpi_publisher = power_monitor.rpi_publisher:main',
            'combined_publisher = power_monitor.combined_publisher:main',
            'power_monitor_subscriber = power_monitor.power_monitor_subscriber:main',
        ],
    },
    package_data={
        package_name: ['utils/*.py'],
    },
)