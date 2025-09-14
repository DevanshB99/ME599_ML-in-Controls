from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='commander',
            executable='optimize_pid.py',
            name='commander_node',
            output='screen'
        )
    ])