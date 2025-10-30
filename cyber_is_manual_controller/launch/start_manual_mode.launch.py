from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Arguments for joystick mapping and scale
    axis_linear = DeclareLaunchArgument('axis_linear', default_value='1')
    axis_angular = DeclareLaunchArgument('axis_angular', default_value='0')
    scale_linear = DeclareLaunchArgument('scale_linear', default_value='0.5')
    scale_angular = DeclareLaunchArgument('scale_angular', default_value='1.0')
    deadman_button = DeclareLaunchArgument('deadman_button', default_value='-1')
    output_topic = DeclareLaunchArgument('output_topic', default_value='/cmd_vel')
    joy_topic = DeclareLaunchArgument('joy_topic', default_value='/joy')
    start_streamer = DeclareLaunchArgument('start_streamer', default_value='false')

    joystick = Node(
        package='cyber_is_manual_controller',
        executable='joystick_teleop',
        name='joystick_teleop',
        output='screen',
        parameters=[{
            'axis_linear': LaunchConfiguration('axis_linear'),
            'axis_angular': LaunchConfiguration('axis_angular'),
            'scale_linear': LaunchConfiguration('scale_linear'),
            'scale_angular': LaunchConfiguration('scale_angular'),
            'deadman_button': LaunchConfiguration('deadman_button'),
            'output_topic': LaunchConfiguration('output_topic'),
            'joy_topic': LaunchConfiguration('joy_topic'),
        }]
    )

    # Optional MJPG streamer execution
    streamer = ExecuteProcess(
        cmd=['ros2', 'run', 'cyber_is_manual_controller', 'start_streamer'],
        shell=False,
        condition=None  # Controlled by launch context in opaque function if needed
    )

    # We control conditional execution via an OpaqueFunction replacement
    from launch.actions import OpaqueFunction
    def add_streamer(context):
        ld = [axis_linear, axis_angular, scale_linear, scale_angular,
              deadman_button, output_topic, joy_topic, joystick]
        if LaunchConfiguration('start_streamer').perform(context).lower() in ('1', 'true', 'yes', 'on'):
            ld.append(streamer)
        return ld

    return LaunchDescription([
        axis_linear,
        axis_angular,
        scale_linear,
        scale_angular,
        deadman_button,
        output_topic,
        joy_topic,
        start_streamer,
        OpaqueFunction(function=add_streamer),
    ])

