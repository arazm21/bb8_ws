# from launch import LaunchDescription
# from launch_ros.actions import Node

# # Path to the Python interpreter inside your virtualenv
# VENV_PYTHON = '/home/bb8/ultralytics-venv/bin/python3'

# def generate_launch_description():
#     return LaunchDescription([
#         # 1) Camera node (system ROS 2 Python)
#         Node(
#             package='camera_ros',
#             executable='camera_node',
#             name='camera_node',
#             output='screen',
#             parameters=[
#                 {'camera': '/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a'},
#                 {'format' : 'XRGB8888'},
#                 {'width'  : 800},
#                 {'height' : 600},
#             ],
#             arguments=['--ros-args', '--log-level', 'warn'],
#         ),

#         # 2) YOLOv8 NCNN detector under the venv’s Python
#         Node(
#             package='detections',
#             executable=VENV_PYTHON,                # launch venv’s python
#             name='yolo_ncnn_detector',
#             output='screen',
#             arguments=[
#                 '-u', '-m', 'detections.yolov8_object_detection_ncnn_node',  # run as module
#                 '--ros-args', '--log-level', 'warn',
#                 # explicit remappings
#                 '-r', '/camera/image_raw:=/camera_node/image_raw',
#                 '-r', '/camera/image_objects:=/camera_node/image_objects',
#             ],
#         ),
#     ])




# yolo_ncnn.launch.py
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

# Path to your venv python (keep if you want to run detector inside that venv)
VENV_PYTHON = '/home/bb8/ultralytics-venv/bin/python3'

# Path to workspace src (so venv python can import detections package from source)
WORKSPACE_SRC = os.path.expanduser('/home/bb8/bb8_ws/src')

def generate_launch_description():
    camera_node = Node(
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
    )

    detector_cmd = [
        VENV_PYTHON,
        '-u',
        '-m', 'detections.yolov8_object_detection_ncnn_node',
        '--ros-args', '--log-level', 'warn',
        '-r', '/camera/image_raw:=/camera_node/image_raw',
        '-r', '/camera/image_objects:=/camera_node/image_objects',
    ]

    # COPY current environment and only modify PYTHONPATH so we don't lose LD_LIBRARY_PATH, etc.
    env = os.environ.copy()
    existing_py = env.get('PYTHONPATH', '')
    env['PYTHONPATH'] = WORKSPACE_SRC + (':' + existing_py if existing_py else '')

    detector_proc = ExecuteProcess(
        cmd=detector_cmd,
        output='screen',
        env=env,  # <<--- important: use the copied env, not a single-key dict
    )

    # small delay to let camera node initialize
    delayed_detector = TimerAction(period=1.0, actions=[detector_proc])

    return LaunchDescription([
        camera_node,
        delayed_detector,
    ])
