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
    slam_params_file = LaunchConfiguration('slam_params_file')
    autostart = LaunchConfiguration('autostart')
    behavior_plugin = LaunchConfiguration('behavior_plugin')

    # Default params file from this package
    default_params = os.path.join(
        get_package_share_directory('cyber_is_navigation'),
        'config',
        'nav2_params.yaml'
    )

    # Default SLAM toolbox params
    default_slam_params = os.path.join(
        get_package_share_directory('cyber_is_navigation'),
        'config',
        'slam_toolbox_online_async.yaml'
    )

    # Default Nav2 BT (short) tree
    default_bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        'navigate_w_replanning_and_recovery.xml'
    )

    # SLAM toolbox (async) publishes /map and map->odom TF
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, slam_params_file],
    )

    # Nav2 container without map_server and amcl (we rely on SLAM's /map)
    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', 'info'],
        composable_node_descriptions=[
            # Planner Server (Smac 2D)
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Controller Server (RPP)
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Behavior Server
            ComposableNode(
                package='nav2_behaviors',
                plugin=behavior_plugin,
                name='behavior_server',
                parameters=[params_file],
                extra_arguments=[{'use_intra_process_comms': True}],
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
                extra_arguments=[{'use_intra_process_comms': True}],
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
            {'node_names': ['controller_server', 'planner_server', 'bt_navigator', 'behavior_server']}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Full path to the Nav2 parameters YAML'
        ),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_slam_params,
            description='Full path to the slam_toolbox parameters YAML'
        ),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'
        ),
        DeclareLaunchArgument(
            'behavior_plugin',
            default_value='behavior_server::BehaviorServer',
            description='Fully-qualified Behavior Server component plugin name'
        ),
        slam_toolbox,
        container,
        lifecycle_manager,
    ])

