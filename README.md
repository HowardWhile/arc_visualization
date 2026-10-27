# arc_visualization

RViz2 displays copied from `jsk_rviz_plugins`, with package and C++ namespace renamed to `arc_visualization`. Display behavior is unchanged.

- `arc_visualization/CameraInfo`
- `arc_visualization/TFTrajectory`

Build in a ROS 2 workspace with `colcon build --packages-select arc_visualization`, source the workspace, then select these displays under `arc_visualization` in RViz2. In existing RViz configurations, replace the corresponding `Class: jsk_rviz_plugins/...` values with `Class: arc_visualization/...`.

The imported display sources retain their original JSK BSD license notices. The original `jsk_rviz_plugins` package is unchanged.
