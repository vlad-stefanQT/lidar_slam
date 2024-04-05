import launch
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import launch_ros
import os

# # Each lidar driver has a corresponding boolean value showing if it is used or not
# lidar_drivers = {'sick551':False, 'sick571':False, 'ust10lx':False,
#                   'ust20lx':False, 'pacecat': False, 'slamtech': False}
## TODO
lidar_drivers = {
    'sick551':{'path':[os.path.join(get_package_share_directory('sick_scan_xd'), 'launch', 'sick_tim_5xx.launch')],
               'launch_arg':{'model':'sick551'}},
    'sick571':{'path':[os.path.join(get_package_share_directory('sick_scan_xd'), 'launch','sick_tim_5xx.launch')],
               'launch_arg':{'model':'sick571'}}, 
    'ust':{'path':[os.path.join(get_package_share_directory('simple_bot'), 'launch','urg_node2.launch.py')],
               'launch_arg':{}},
    'pacecat':{'path':[os.path.join(get_package_share_directory('bluesea2'), 'launch','LDS-50C-E.launch')],
               'launch_arg':{'params_file': os.path.join(get_package_share_directory('simple_bot'), 'config', 'LDS-50C-E.yaml')}},
    'slamtech':{'path':[os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'),'sllidar_t1_launch.py'],
                'launch_arg':{}},
}

## END TODO
def generate_launch_description():
    # The user selects which lidar driver to use by assigning a value to the 'lidar' launch argument
    argument = DeclareLaunchArgument('lidar', default_value='sick551', 
                                     choices=['sick551', 'sick571', 'ust10lx', 'ust20lx', 'pacecat', 'slamtech'])
    # Switch the boolean corresponding to the required lidar driver to True
    lidar = LaunchConfiguration('lidar')

    try:
        lidar_driver = lidar_drivers[lidar]
    except KeyError:
        print(f"Lidar driver for {lidar} not found")

    # Get the path of the URDF file and the rviz configuration
    pkg_share = launch_ros.substitutions.FindPackageShare(package='simple_bot').find('simple_bot')
    default_model_path = os.path.join(pkg_share, 'urdf/simple_bot.urdf')

    # TODO: select the correct rviz config
    default_rviz_config_path = os.path.join(pkg_share, 'config/slam_toolbox_default.rviz')

    # FIXME not adapted for lidar frame_ids
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

    
    #TODO: add all drivers
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_driver['path']),
        launch_arguments=lidar_driver['launch_arg'].items(),  # FIXME doesn't work
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
        argument,
        driver_launch,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        laser_scan_matcher,
        slam_toolbox
    ])
