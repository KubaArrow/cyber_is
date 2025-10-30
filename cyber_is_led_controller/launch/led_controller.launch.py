#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    leds_topic_arg = DeclareLaunchArgument('leds_topic', default_value='/leds')
    state_topic_arg = DeclareLaunchArgument('state_topic', default_value='/leds_mode')
    side_leds_arg = DeclareLaunchArgument('side_leds', default_value='9')
    front_leds_arg = DeclareLaunchArgument('front_leds', default_value='5')
    frequency_arg = DeclareLaunchArgument('frequency', default_value='12')

    node = Node(
        package='cyber_is_led_controller',
        executable='leds_node',
        name='leds_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'leds_topic': LaunchConfiguration('leds_topic'),
            'state_topic': LaunchConfiguration('state_topic'),
            'side_leds': ParameterValue(LaunchConfiguration('side_leds'), value_type=int),
            'front_leds': ParameterValue(LaunchConfiguration('front_leds'), value_type=int),
            'frequency': ParameterValue(LaunchConfiguration('frequency'), value_type=int),
        }],
    )

    return LaunchDescription([
        leds_topic_arg,
        state_topic_arg,
        side_leds_arg,
        front_leds_arg,
        frequency_arg,
        node,
    ])

