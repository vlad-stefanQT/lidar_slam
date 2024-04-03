import launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='simple_bot').find('simple_bot')
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

    # launch rviz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
    )

    # launch lidar driver
    # lidar_driver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('sick_scan_xd'), 'launch', 'sick_tim_5xx.launch.py')]),
    #      launch_arguments={'':'hostname:=192.168.0.10'}.items()  # Does not work (why?)
    # )

    lidar_driver = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('urg_node2'), 'launch', 'urg_node2.launch.py')]),
         launch_arguments={'ip_address':'192.168.0.20'}.items()
    )
    # launch the laser scan matcher
    laser_scan_matcher = Node(
        package='ros2_laser_scan_matcher', 
        executable='laser_scan_matcher',
        name='scan_matcher'
    )
    # launch the slam toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')]),
        launch_arguments={'slam_params_file':os.path.join(pkg_share, 'config/config.yaml')}.items()
    )

    return launch.LaunchDescription([
        lidar_driver,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        laser_scan_matcher,
        slam_toolbox
    ])