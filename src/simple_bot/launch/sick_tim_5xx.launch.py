import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    argument = DeclareLaunchArgument('model', default_value='sick571', choices=['sick_tim_571', 'sick_tim_551'] )
    model = LaunchConfiguration('model')

    ld = LaunchDescription()
    sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    launchfile = os.path.basename(__file__)[:-3] # convert "<lidar_name>.launch.py" to "<lidar_name>.launch"
    launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) # 'launch/sick_tim_5xx.launch')
    node_arguments=[launch_file_path]
    
    host_names = {'sick551':'192.168.0.1',
                  'sick571':'192.168.0.10'}
    node_arguments.append(f'hostname:={host_names[model]}')
    node_arguments.append('frame_id:=laser')
    # append optional commandline arguments in name:=value syntax
    for arg in sys.argv:
        if len(arg.split(":=")) == 2:
            node_arguments.append(arg)
    
    ROS_DISTRO = os.environ.get('ROS_DISTRO') # i.e. 'eloquent', 'foxy', etc.
    if ROS_DISTRO[0] <= "e": # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable"
        node = Node(
            package='sick_scan_xd',
            node_executable='sick_generic_caller',
            output='screen',
            arguments=node_arguments
        )
    else: # ROS versions eloquent and earlier require "node_executable", ROS foxy and later use "executable"
        node = Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            output='screen',
            arguments=node_arguments
        )
    ld.add_action(argument)
    ld.add_action(node)
    return ld
