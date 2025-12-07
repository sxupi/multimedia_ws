from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='multimedia_python', executable='controll_node'),
        Node(package='multimedia_python', executable='lcd_1602_node'),
        Node(package='multimedia_python', executable='si470x_node'),
        Node(package='multimedia_python', executable='ir_receiver_node'),
        Node(package='multimedia_python', executable='volume_system_node'),
    ])
