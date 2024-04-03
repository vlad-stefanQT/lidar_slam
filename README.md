# lidar_slam
a valid transform from odom to base_link is necessary.
Right now there is no odometry used so it is necessary to at least publish a static transform.

```bash
ros2 run tf2_ros static_transform_publisher 0.1 0 0 0 0 0 odom base_link
```

Since this transform is not updated, the map will not be updated either.
