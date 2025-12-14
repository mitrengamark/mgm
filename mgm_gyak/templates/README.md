# Templates Examples

This folder contains runnable examples for common ROS 2 tasks:

- examples_tf2_pose.py: `tf2_ros` Buffer/TransformListener and `do_transform_pose_stamped` to transform `PoseStamped` from `base_link` to `map`.
- examples_pointcloud2.py: Convert `LaserScan` to `PointCloud2` using `sensor_msgs_py.point_cloud2`.
- examples_markers.py: Publish a single `Marker` and a `MarkerArray` of spheres.
- examples_odometry_quat.py: Create `Odometry` with `quaternion_from_euler` and read yaw via `euler_from_quaternion`.

Run any example:

```bash
source install/setup.bash
ros2 run templates examples_tf2_pose
ros2 run templates examples_pointcloud2
ros2 run templates examples_markers
ros2 run templates examples_odometry_quat
```

Adapt topics/frame_ids as needed for your environment.
