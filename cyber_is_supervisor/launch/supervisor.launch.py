from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='cyber_is_supervisor',
            executable='supervisor_node',
            name='supervisor',
            output='screen',
            parameters={
                'initial_mode': 'MANUAL',
                'nav_type': 'n',
                'enable_echo': True,
                'heartbeat_period': 1.0,
            },
        )
    ])

