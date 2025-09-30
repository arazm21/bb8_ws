from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the camera driver
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                {'camera': '/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a'},
                {'format': 'XRGB8888'},
                {'width': 800},
                {'height': 600},
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # Start the DNN face detector
        Node(
            package='face_detector',
            executable='face_detector_dnn',
            name='dnn_face_detector',
            output='screen',
            remappings=[
                ('/camera/image_raw', '/camera_node/image_raw'),
                ('/camera/image_faces', '/camera_node/image_faces'),
            ],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ])
