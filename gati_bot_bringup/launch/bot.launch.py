import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # 1. Define Paths
    # Replace 'gati_bot_model_description' and 'gati_bot_bringup' with your actual pkg names
    description_pkg_share = get_package_share_directory('gati_bot_model_description')
    bringup_pkg_share = get_package_share_directory('gati_bot_bringup')

    #RPLIDAR 
    rplidar_pkg_share = get_package_share_directory('rplidar_ros')

    
    urdf_path = os.path.join(description_pkg_share, 'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(bringup_pkg_share, 'config', 'rviz2.rviz')

    # 2. Robot State Publisher Node
    # This uses 'xacro' command to process the file into raw URDF string
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    # 3. Joint State Publisher Node
    # If you want the GUI version, use 'joint_state_publisher_gui'
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # 4. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    # 5. Gati Bot Node to operate the motors using teleop
    gati_bot_node = Node(
        package='gati_bot_node',
        executable='motor2.py',
        output='screen'
    )
    rplidar_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(rplidar_pkg_share, 'launch', 'rplidar_a1_launch.py')
         )
    )
    # static_tf_footprint_to_base = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0','0','0','0','0','0','base_footprint','base_link']
    # )

    # static_tf_base_to_laser = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['0.20','0','0.15','0','0','0','base_link','laser_frame']  # ← also fix frame name (see Bug 2)
    #)
    # Create Launch Description and add actions
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        gati_bot_node,
        rplidar_launch,
        #static_tf_footprint_to_base,   # ← ADD THESE
       # static_tf_base_to_laser,  
    ])