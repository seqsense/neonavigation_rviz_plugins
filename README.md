# neonavigation_rviz_plugins
RViz plugins for neonavigation_msgs

These are ROS 2 (rviz2) plugins, built with `ament_cmake` against `rviz_common` /
`rviz_rendering` / `rviz_default_plugins`.

| Plugin | Message |
| --- | --- |
| `costmap_cspace_rviz_plugins/CSpace3D` | `costmap_cspace_msgs/msg/CSpace3D` (+ `CSpace3DUpdate`) |
| `trajectory_tracker_rviz_plugins/PathWithVelocity` | `trajectory_tracker_msgs/msg/PathWithVelocity` |

`CSpace3D` draws one yaw layer at a time; pick it with the `Yaw` property. The
`Update Topic` property is independent of `Topic` and has to be set as well
(`costmap_cspace` publishes `costmap` and `costmap_update`).

`costmap_cspace` publishes both with `transient_local` durability, which was
latching under ROS 1. rviz2 subscribes as `volatile` by default, so set
**Durability Policy** to **Transient Local** on both topics to pick up a costmap
that was published before the display was added.

## License
The source codes in this package are available under BSD license.
All icons and graphics in this package are in the Public Domain.

This meta-package contains derived codes and icons from [rviz](https://github.com/ros-visualization/rviz)
and [rviz2](https://github.com/ros2/rviz).

See [LICENSE](LICENSE) for detail.
