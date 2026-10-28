# arc_visualization

ROS 2 visualization packages. The RViz display package lives in `arc_rviz_plugins/`.

## arc_rviz_plugins

RViz2 displays copied from `jsk_rviz_plugins`, with package and C++ namespace renamed to `arc_rviz_plugins`. Image subscriptions use the message type discovered from DDS; no transport hints are required.

- `arc_rviz_plugins/CameraInfo`
- `arc_rviz_plugins/TFTrajectory`
- `arc_rviz_plugins/DepthView` — rectified depth to Points, optional aligned RGB, frame skipping, pixel stride and maximum Z filtering. See [DepthView usage and validation](docs/depth-view.md).

Build in a ROS 2 workspace with `colcon build --packages-select arc_rviz_plugins`, source the workspace, then select these displays under `arc_rviz_plugins` in RViz2. In existing RViz configurations, replace the corresponding `Class: jsk_rviz_plugins/...` or `Class: arc_visualization/...` values with `Class: arc_rviz_plugins/...`.

The imported display sources retain their original JSK BSD license notices. The original `jsk_rviz_plugins` package is unchanged.

CameraInfo topic reception frequency uses a fixed rolling five-second window (messages received in the last five seconds divided by five), measured with a monotonic clock. The initial five seconds are a warm-up period. The status updates even without incoming messages and reaches 0 Hz after five seconds of silence. Resetting the display or changing the topic clears the window.

Image Topic supports `sensor_msgs/msg/Image` and `sensor_msgs/msg/CompressedImage` regardless of the topic name suffix. Unknown topics are retried once per second without creating a subscription until their type is discovered. Unsupported or ambiguous types are reported in the display status.
