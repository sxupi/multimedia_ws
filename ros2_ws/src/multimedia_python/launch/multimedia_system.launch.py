from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multimedia_python',
            executable='radio_node',
            name='radio_node'
        ),
        Node(
            package='multimedia_python',
            executable='volume_controller_node',
            name='volume_controller_node'
        ),
        Node(
            package='multimedia_python',
            executable='frequency_controller_node',
            name='frequency_controller_node'
        ),
        Node(
            package='multimedia_python',
            executable='ir_receiver_node',
            name='ir_receiver_node'
        ),
        Node(
            package='multimedia_python',
            executable='display_node',
            name='display_node'
        ),
    ])
