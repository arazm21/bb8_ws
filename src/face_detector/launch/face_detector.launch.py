from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start the camera driver node with explicit parameters to silence startup warnings
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[
                # Select the camera to silence the "no camera selected" warning
                {'camera': '/base/axi/pcie@120000/rp1/i2c@80000/imx708@1a'},
                # Specify pixel format to silence the "no pixel format selected" warning
                {'format': 'XRGB8888'},
                # Define resolution to silence the "no dimensions selected" warning
                {'width': 800},
                {'height': 600},
            ],
            # Lower log verbosity to WARN to hide informational and debug messages
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # Start the face detector node, remapping to the camera_node topics
        Node(
            package='face_detector',
            executable='face_detector',
            name='face_detector',
            output='screen',
            remappings=[
                ('/camera/image_raw', '/camera_node/image_raw'),
                ('/camera/image_faces', '/camera_node/image_faces'),
            ],
            # Reduce logging verbosity here as well
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ])
