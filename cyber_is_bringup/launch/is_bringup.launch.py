#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Basic flags
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_description = LaunchConfiguration('start_description')
    use_gui = LaunchConfiguration('use_gui')
    start_navigation = LaunchConfiguration('start_navigation')
    start_uart_bridge = LaunchConfiguration('start_uart_bridge')
    start_led_controller = LaunchConfiguration('start_led_controller')
    start_supervisor = LaunchConfiguration('start_supervisor')
    start_lidar = LaunchConfiguration('start_lidar')
    start_rosbridge = LaunchConfiguration('start_rosbridge')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    # Navigation args
    nav_params_file = LaunchConfiguration('nav_params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Local params for nodes
    uart_bridge_params_file = LaunchConfiguration('uart_bridge_params_file')
    leds_params_file = LaunchConfiguration('leds_params_file')
    supervisor_params_file = LaunchConfiguration('supervisor_params_file')

    # Defaults
    default_nav_params = PathJoinSubstitution([
        FindPackageShare('cyber_is_navigation'), 'config', 'nav2_params.yaml'
    ])
    default_uart_params = PathJoinSubstitution([
        FindPackageShare('cyber_is_bringup'), 'config', 'uart_bridge.yaml'
    ])
    default_leds_params = PathJoinSubstitution([
        FindPackageShare('cyber_is_bringup'), 'config', 'leds_controller.yaml'
    ])
    default_supervisor_params = PathJoinSubstitution([
        FindPackageShare('cyber_is_bringup'), 'config', 'supervisor.yaml'
    ])

    # Nodes
    uart_bridge = Node(
        package='uart_bridge',
        executable='uart_bridge',
        name='uart_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[uart_bridge_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_uart_bridge),
    )

    leds_controller = Node(
        package='cyber_is_led_controller',
        executable='leds_node',
        name='leds_controller',
        output='screen',
        emulate_tty=True,
        parameters=[leds_params_file],
        condition=IfCondition(start_led_controller),
    )

    supervisor = Node(
        package='cyber_is_supervisor',
        executable='supervisor_node',
        name='supervisor',
        output='screen',
        parameters=[supervisor_params_file],
        condition=IfCondition(start_supervisor),
    )

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cyber_is_description'), 'launch', 'display.launch.py'
        ])),
        launch_arguments={'use_gui': use_gui}.items(),
        condition=IfCondition(start_description),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cyber_is_navigation'), 'launch', 'start_navigation.launch.py'
        ])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'map': map_yaml,
            'autostart': autostart,
        }.items(),
        condition=IfCondition(start_navigation),
    )

    # LIDAR (ld14p.launch.py)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('ldlidar_sl_ros2'), 'launch', 'ld14p.launch.py'
        ])),
        # If you need to pass serial/frame/topic later, add launch_arguments={ ... }.items()
        condition=IfCondition(start_lidar),
    )

    # Rosbridge WebSocket (XML launch)
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'
        ])),
        launch_arguments={
            'port': rosbridge_port,
            # 'address': '0.0.0.0',  # uncomment to bind explicitly
        }.items(),
        condition=IfCondition(start_rosbridge),
    )

    return LaunchDescription([
        # Core flags
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('start_description', default_value='false',
                              description='Start robot description + RViz'),
        DeclareLaunchArgument('use_gui', default_value='false',
                              description='Use joint_state_publisher_gui if true'),
        DeclareLaunchArgument('start_navigation', default_value='true',
                              description='Start Nav2 stack'),
        DeclareLaunchArgument('start_uart_bridge', default_value='true',
                              description='Start UART bridge node'),
        DeclareLaunchArgument('start_led_controller', default_value='true',
                              description='Start LED controller node'),
        DeclareLaunchArgument('start_supervisor', default_value='true',
                              description='Start supervisor node'),
        DeclareLaunchArgument('start_lidar', default_value='true',
                              description='Start LDLIDAR SL (ld14p) launcher'),
        DeclareLaunchArgument('start_rosbridge', default_value='true',
                              description='Start rosbridge websocket server'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090',
                              description='rosbridge websocket port'),

        # Navigation args
        DeclareLaunchArgument('nav_params_file', default_value=default_nav_params,
                              description='Full path to Nav2 parameters YAML'),
        DeclareLaunchArgument('map', default_value='',
                              description='Full path to the map YAML file'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Automatically startup the Nav2 stack'),

        # Per-node parameter files
        DeclareLaunchArgument('uart_bridge_params_file', default_value=default_uart_params,
                              description='UART bridge parameters YAML'),
        DeclareLaunchArgument('leds_params_file', default_value=default_leds_params,
                              description='LED controller parameters YAML'),
        DeclareLaunchArgument('supervisor_params_file', default_value=default_supervisor_params,
                              description='Supervisor node parameters YAML'),

        # Launch inclusions
        description_launch,
        uart_bridge,
        leds_controller,
        supervisor,
        lidar_launch,
        rosbridge_launch,
        navigation_launch,
    ])
