import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def include_conditional_lidar_launch_file(context):
    # Special thanks to 吉海さん
    # LaunchConfiguration の値を取得 
    lidar = context.launch_configurations.get('lidar', 'sick551')
    launch_path = os.path.join(get_package_share_directory('simple_bot'), 'launch', 
                         'simple_robot_multiple.launch.py')
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=[('lidar', lidar)],
    )]

def generate_launch_description():
    # The user selects which lidar driver to use by assigning a value to the 'lidar' launch argument
    argument = DeclareLaunchArgument('lidar', default_value='sick551',
                                     choices=['sick551', 'sick571', 'ust10lx', 'ust20lx', 'pacecat', 'slamtech'])

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    simple_bot_prefix = get_package_share_directory('simple_bot')

    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  simple_bot_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='simple_bot.lua')

    resolution = LaunchConfiguration('resolution', default='0.01')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')  # Not sure what this does

    # rviz_config_dir = os.path.join(simple_bot_prefix, 'rviz', 'simple_bot_slam_config.rviz')
    rviz_config_dir = os.path.join(simple_bot_prefix, 'rviz', 'tb3_cartographer.rviz')
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')

    return LaunchDescription([
        argument,
        rviz_node,
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,  # default also set in LaunchConfiguration (why?)
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
        OpaqueFunction(function=include_conditional_lidar_launch_file),
    ])