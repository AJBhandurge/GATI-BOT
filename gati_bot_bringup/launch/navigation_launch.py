import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('gati_bot_bringup')

    # Parameter and map file locations
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')

    return LaunchDescription([

        # 1. Declare Arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_path, 'config', 'nav2_params.yaml'),
            description='Full path to the nav2_params.yaml configuration file'
        ),

        DeclareLaunchArgument(
            'map',
            # This remains the default if you don't provide one in the terminal
            default_value=os.path.join(pkg_path, 'maps', 'svr100.yaml'),
            description='Full path to the map YAML file'
        ),

        # 2. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': map_yaml_file # This will now use the arg passed via CLI
            }]
        ),

        # 3. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[params_file]
        ),

        # 4. Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),

        # 5. Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),

        # 6. Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),

        # 7. Waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),
        
        # 8. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'waypoint_follower'
                ]
            }]
        ),
    ])
