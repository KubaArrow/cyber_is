from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    line_filter = Node(
        package='cyber_is_filters',
        executable='line_filter',
        name='line_filter',
        output='screen',
        parameters=[
            {
                'line_detector_topic': '/line_detector',
                'max_limit': False,
                'threshold0': 3800,
                'threshold1': 3800,
                'threshold2': 3850,
                'threshold3': 3800,
                'threshold4': 3800,
            }
        ],
    )

    magnet_filter = Node(
        package='cyber_is_filters',
        executable='magnet_filter',
        name='magnet_filter',
        output='screen',
        parameters=[
            {
                'magnet_detector_topic': '/magnet',
                'max_limit': False,
                'min_value': 2000,
                'max_value': 3500,
            }
        ],
    )

    return LaunchDescription([line_filter, magnet_filter])

