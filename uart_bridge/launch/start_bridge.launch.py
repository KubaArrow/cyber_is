from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='uart_bridge',
            executable='uart_bridge',
            name='uart_bridge',
            output='screen',
            parameters=[
                {'uart_port': '/dev/ttyACM0'},
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

