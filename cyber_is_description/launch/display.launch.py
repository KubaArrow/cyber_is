#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_gui_arg = DeclareLaunchArgument(
        'use_gui', default_value='false',
        description='Launch joint_state_publisher_gui when true'
    )

    xacro_file = PathJoinSubstitution([FindPackageShare('cyber_is_description'), 'urdf', 'cyber_is.xacro'])
    rviz_config = PathJoinSubstitution([FindPackageShare('cyber_is_description'), 'rviz', 'cyber_is.rviz'])

    # Use xacro to generate the robot_description
    robot_description_content = Command([FindExecutable(name='xacro'), ' ', xacro_file])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)
    }

    # Optional GUI joint state publisher (off by default to avoid missing package errors)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[robot_description]
    )

    return LaunchDescription([
        use_gui_arg,
        joint_state_publisher,
        robot_state_publisher,
        rviz,
    ])
