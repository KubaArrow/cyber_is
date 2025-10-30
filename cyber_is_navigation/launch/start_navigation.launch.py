from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Default params file from this package
    default_params = os.path.join(
        get_package_share_directory('cyber_is_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # Default Nav2 BT (short) tree
    default_bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    )

    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'info'],
        intra_process_comms=True,
        composable_node_descriptions=[
            # Map Server
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[
                    params_file,
                    {'yaml_filename': map_yaml}
                ],
            ),
            # AMCL
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[params_file],
            ),
            # Planner Server (Smac 2D)
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file],
            ),
            # Controller Server (RPP)
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file],
            ),
            # Behavior Server
            ComposableNode(
                package='nav2_behaviors',
                plugin='nav2_behaviors::BehaviorServer',
                name='behavior_server',
                parameters=[params_file],
            ),
            # BT Navigator
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[
                    params_file,
                    {'default_bt_xml_filename': default_bt_xml}
                ],
            ),
        ]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['map_server', 'amcl', 'controller_server', 'planner_server', 'bt_navigator', 'behavior_server']}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Full path to the map YAML file'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        container,
        lifecycle_manager,
    ])

