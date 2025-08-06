from launch import LaunchDescription
from launch_ros.actions import Node

# Path to the Python interpreter inside your virtualenv
VENV_PYTHON = '/home/bb8/ultralytics-venv/bin/python3'

def generate_launch_description():
    return LaunchDescription([
        # 1) Camera node (system ROS 2 Python)
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'camera': '/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a'},
                {'format' : 'XRGB8888'},
                {'width'  : 800},
                {'height' : 600},
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # 2) YOLOv8 NCNN detector under the venv’s Python
        Node(
            package='detections',
            executable=VENV_PYTHON,                # launch venv’s python
            name='yolo_ncnn_detector',
            output='screen',
            arguments=[
                '-u', '-m', 'detections.yolov8_object_detection_ncnn_node',  # run as module
                '--ros-args', '--log-level', 'warn',
                # explicit remappings
                '-r', '/camera/image_raw:=/camera_node/image_raw',
                '-r', '/camera/image_objects:=/camera_node/image_objects',
            ],
        ),
    ])
