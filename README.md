# lidar_slam

## 基本機能 - Basic Functions ##
#TODO

## 事前準備 - Advance Preparations ##
Init submodules and clone LIDAR driver packages.
`git submodule init`
`git submodule update --init --recursive`

`sudo apt install ros-humble-urg-node` (may not be required)

`colcon build --packages-select libsick_ldmrs --event-handlers console_direct+`

`colcon build --packages-select msgpack11 --cmake-args " -DMSGPACK11_BUILD_TESTS=0" --event-handlers console_direct+`

`source ./install/setup.bash`

Build scik lidar driver packages (~2mins)
`colcon build --packages-select sick_scan_xd --cmake-args " -DROS_VERSION=2" --event-handlers console_direct+`

`colcon build colcon build --symlink-install`
`source ./install/setup.bash`

# In the case of any packages needing to be rebuilt after editing
`colcon build colcon build --packages-select <package_name>`
`source ./install/setup.bash`

## 実行方法 - How to Run ##
# Without robot
Launch Lidar with robot state publisher
`ros2 launch simple_bot simple_robot_multiple.launch.py lidar:='<lidar_name>'`

Standalone LiDAR slam with Cartographer
`ros2 launch simple_bot simple_bot_slam.launch lidar:='<lidar_name>'`

# With robot
Launch slamtoolbox for mapping
`ros2 launch simple_bot slam_toolbox.launch`

# Running drivers individually
<<SLAMTEC_LPX_T1>>
IP on computer= 192.168.11.5
ros2 launch sllidar_ros2 view_sllidar_t1_launch.py
rviz opens by itself
frame id = laser

<<PaceCat 2D LDS-50C-E>>
IP on computer = 192.168.158.5
ros2 launch bluesea2 LDS-50C-E.launch
ros2 run rviz2 rviz2
frame id = map

<<SICK TIM571>>
IP on computer= 192.168.0.5
ros2 launch sick_scan_xd sick_tim_5xx.launch.py hostname:=192.168.0.10
ros2 run rviz2 rviz2
frame id = cloud

<<SICK TIM551>>
IP on computer= 192.168.0.5
ros2 launch sick_scan_xd sick_tim_5xx.launch.py hostname:=192.168.0.1
ros2 run rviz2 rviz2
frame id = cloud

<<UST-10LX/UST-20LXSH>>
IP on computer= 192.168.0.5
ros2 launch urg_node2 urg_node2.launch.py
ros2 run rviz2 rviz2
frame id = laser


Since this transform is not updated, the map will not be updated either.
