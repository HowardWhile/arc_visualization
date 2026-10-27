# arc_visualization

ROS 2 visualization packages. The RViz display package lives in `arc_rviz_plugins/`.

## arc_rviz_plugins

RViz2 displays copied from `jsk_rviz_plugins`, with package and C++ namespace renamed to `arc_rviz_plugins`. Theora is omitted from the image transport options; other display behavior is unchanged.

- `arc_rviz_plugins/CameraInfo`
- `arc_rviz_plugins/TFTrajectory`

Build in a ROS 2 workspace with `colcon build --packages-select arc_rviz_plugins`, source the workspace, then select these displays under `arc_rviz_plugins` in RViz2. In existing RViz configurations, replace the corresponding `Class: jsk_rviz_plugins/...` or `Class: arc_visualization/...` values with `Class: arc_rviz_plugins/...`.

The imported display sources retain their original JSK BSD license notices. The original `jsk_rviz_plugins` package is unchanged.
