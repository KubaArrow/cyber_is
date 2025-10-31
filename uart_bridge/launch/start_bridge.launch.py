from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    uart_port = LaunchConfiguration('uart_port')
    baud_rate = LaunchConfiguration('baud_rate')

    return LaunchDescription([
        DeclareLaunchArgument('uart_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        Node(
            package='uart_bridge',
            executable='uart_bridge',
            name='uart_bridge',
            output='screen',
            parameters=[
                {'uart_port': uart_port},
                {'baud_rate': baud_rate},
                {'frequency': 200},
                {'twist_topic': '/cmd_vel'},
                {'pose_topic': '/slam_out_pose'},
                {'odom_topic': '/low_level_odom'},
                {'odom_frame': 'odom'},
                {'imu_topic': '/imu'},
                {'imu_frame': 'imu'},
                {'magnet_topic': '/magnet'},
                {'line_detector_topic': '/line_detector'},
                {'leds_topic': '/leds'},
                {'battery_topic': '/battery'},
                {'status_topic': '/status_topic'},
            ],
        )
    ])
