# Launches specified LIDAR driver with frame_id laser

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


lidar_drivers = {
    'sick551':{'path':[os.path.join(get_package_share_directory('simple_bot'), 'launch', 'sick_tim_551.launch.py')],
               'launch_arg':{}},
    'sick571':{'path':[os.path.join(get_package_share_directory('simple_bot'), 'launch','sick_tim_571.launch.py')],
               'launch_arg':{}},
    'ust10lx':{'path':[os.path.join(get_package_share_directory('simple_bot'), 'launch','urg_node2.launch.py')],
               'launch_arg':{}},
    'ust20lx':{'path':[os.path.join(get_package_share_directory('simple_bot'), 'launch','urg_node2.launch.py')],
               'launch_arg':{'params_file':os.path.join(get_package_share_directory('simple_bot'), 'config','params_ether.yaml')}},  #FIXME pass param?
    'pacecat':{'path':[os.path.join(get_package_share_directory('bluesea2'), 'launch','LDS-50C-E.launch')],
               'launch_arg':{'params_file': os.path.join(get_package_share_directory('simple_bot'), 'config', 'LDS-50C-E.yaml')}},
    'slamtech':{'path':[os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_t1_launch.py')],
                'launch_arg':{}},
}

def include_conditional_lidar_launch_file(context):
    # Special thanks to 吉海さん
    # LaunchConfiguration の値を取得 
    lidar_type = context.launch_configurations.get('lidar', 'sick551')
    print(lidar_type)

    lidar_driver = lidar_drivers[lidar_type]
    launch_path = lidar_driver['path']
    launch_args = lidar_driver['launch_arg']
    print(launch_path)
    print(launch_args)

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments=launch_args.items(),
    )]

def generate_launch_description():
    # The user selects which lidar driver to use by assigning a value to the 'lidar' launch argument
    # argument = DeclareLaunchArgument('lidar', default_value='sick551',
    #                                  choices=['sick551', 'sick571', 'ust10lx', 'ust20lx', 'pacecat', 'slamtech'])

    # Get the path of the URDF file and the rviz configuration
    pkg_share = FindPackageShare(package='simple_bot').find('simple_bot')
    default_model_path = os.path.join(pkg_share, 'urdf/simple_bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'config/slam_toolbox_default.rviz')

    with open(default_model_path, 'r') as urdf:
        robot_desc = urdf.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )

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
        # argument,
        # joint_state_publisher_node,
        # robot_state_publisher_node,
        rviz_node,
        # OpaqueFunction(function=include_conditional_lidar_launch_file),
        slam_toolbox
    ])
