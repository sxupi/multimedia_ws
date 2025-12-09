from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='multimedia_python', executable='start_multimedia'),
    ])
