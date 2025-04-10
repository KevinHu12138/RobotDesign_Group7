from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import SetParameter, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os


def generate_launch_description():
    ld = LaunchDescription()
    # Parameters, Nodes and Launch files go here
    # Declare package directory
    pkg_nav_demos = get_package_share_directory('navigation_demos')
    
    # Define nav_to_pose behaviour tree
    bt_xml_navtopose_file = PathJoinSubstitution([pkg_nav_demos, 'behavior_tree_xml', 'bt_simple_nav.xml'])

    # Declare the argument for use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Whether to run in simulation mode'
    )

    # Necessary fixes
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behaviour_server',
        'bt_navigator',
        #'map_server',
        #'amcl',
        #'waypoint_follower',
    ]

    # LOAD PARAMETERS FROM YAML FILES
    config_controller = PathJoinSubstitution([pkg_nav_demos, 'config', 'controller.yaml'])
    config_planner    = PathJoinSubstitution([pkg_nav_demos, 'config', 'planner.yaml'])
    config_bt_nav     = PathJoinSubstitution([pkg_nav_demos, 'config', 'bt_nav.yaml'])
    config_params = PathJoinSubstitution([pkg_nav_demos, 'config', 'params.yaml'])

    use_sim_time_arg = LaunchConfiguration('use_sim_time')

    launch_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('leo_rover'), '/launch', '/rover.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time_arg
        }.items(),
    )

    # # Include Gazebo Simulation
    # launch_gazebo = IncludeLaunchDescription(
    # PythonLaunchDescriptionSource([get_package_share_directory('gz_example_robot_description'), '/launch', '/sim_robot.launch.py']),
    # launch_arguments={}.items(),
    # )

    # Include SLAM Toolbox standard launch file
    launch_slamtoolbox = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('slam_toolbox'), '/launch', '/online_async_launch.py']),
    launch_arguments={
        'use_sim_time': use_sim_time_arg,
    }.items(),
    )

    # Behaviour Tree Navigator
    node_bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            config_bt_nav,
            {'use_sim_time': use_sim_time_arg},
            ],
        remappings=remappings,
    )

    # Behaviour Tree Server
    node_behaviour = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behaviour_server',
        output='screen',
        parameters=[
            config_bt_nav,
            {'use_sim_time': use_sim_time_arg},
            ],
        remappings=remappings,
    )

    # Planner Server Node
    node_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            config_planner,
            {'use_sim_time': use_sim_time_arg},
        ], 
        remappings=remappings,
    )

    # Controller Server Node
    node_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            config_controller,
            {'use_sim_time': use_sim_time_arg},
        ],
        remappings=remappings,
    )

    # Map Server Node
    node_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[config_params],
        # remappings=remappings,
    )

    # AMCL Node
    node_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[config_params],
        # remappings=remappings,
    )

    # Waypoint Follower Node
    node_waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[config_params],
        # remappings=remappings,
    )

    # Lifecycle Node Manager to automatically start lifecycles nodes (from list)
    node_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
    )

    node_explore = Node(
        package='navigation_demos',
        executable='explore',
        name='explore',
        output='screen',
    )

    node_simple_nav = Node(
        package='navigation_demos',
        executable='simple_nav',
        name='simple_nav',
        output='screen',
    )

    node_ui_nav = Node(
        package='navigation_demos',
        executable='ui_nav',
        name='ui_nav',
        output='screen',
    )

    # Add actions to LaunchDescription
    ld.add_action(use_sim_time_arg)
    # ld.add_action(launch_gazebo)
    ld.add_action(launch_rover)
    ld.add_action(launch_slamtoolbox)
    ld.add_action(node_bt_nav)
    ld.add_action(node_behaviour)
    ld.add_action(node_planner)
    ld.add_action(node_controller)
    ld.add_action(node_lifecycle_manager)
    ld.add_action(node_ui_nav)
    # ld.add_action(node_map_server)
    # ld.add_action(node_amcl)
    # ld.add_action(node_waypoint_follower)
    # ld.add_action(node_explore)
    # ld.add_action(node_simple_nav)

    return ld