import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Use sim_time flag (keep this if you may run bagfiles or sim time later)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to your URDF/Xacro
    pkg_path = os.path.join(get_package_share_directory('alex_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'round_bot.urdf.xacro')

    # Process the xacro file - wrap with ParameterValue and specify value_type=str
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Robot state publisher
    params = {
        'robot_description': robot_description_config,
        'use_sim_time': use_sim_time
    }
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_gui': False}]
    )

    # RViz2 node (so you can see it right away)
    rviz_config_file = os.path.join(pkg_path, 'config', 'round_bot.rviz')  # optional custom config
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=None  # remove this if you don't have a custom config
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        joint_state_publisher,
        node_robot_state_publisher,
        node_rviz
    ])