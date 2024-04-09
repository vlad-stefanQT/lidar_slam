# Launches slam tool box with simple_bot config
import launch
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import launch_ros
import os


def generate_launch_description():
    # Get the rviz configuration
    pkg_share = FindPackageShare(package='simple_bot').find('simple_bot')
    default_rviz_config_path = os.path.join(pkg_share, 'config/slam_toolbox_default.rviz')

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path ],
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')]),
        launch_arguments={'slam_params_file':os.path.join(pkg_share, 'config/mapper_params_online_async.yaml')}.items()
    )

    return launch.LaunchDescription([
        rviz_node,
        slam_toolbox
    ])
