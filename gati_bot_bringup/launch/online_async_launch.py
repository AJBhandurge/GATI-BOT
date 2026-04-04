import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    slam_params_file = os.path.join(
        get_package_share_directory("gati_bot_bringup"),
        'config',
        'mapper_params_online_async.yaml'
    )

    start_async_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False}   # 🔥 VERY IMPORTANT FOR REAL ROBOT
        ],
    )

    return LaunchDescription([
        start_async_slam_toolbox_node
    ])
