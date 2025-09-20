from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talker_relay_lab',
            executable='talker',
            name='talker',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'v': 7.0, 'd': 7.0}
            ]
        ),
        Node(
            package='talker_relay_lab',
            executable='relay',
            name='relay',
            output='screen',
            emulate_tty=True
        )
    ])