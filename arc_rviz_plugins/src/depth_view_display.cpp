#include "depth_view_display.h"
#include "camera_info_display.h"

#include <OgreSceneNode.h>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>

namespace arc_rviz_plugins
{
namespace properties = rviz_common::properties;
DepthViewDisplay::DepthViewDisplay()
: processor_(std::make_shared<DepthViewProcessor>())
{
  depth_topic_ = new ImageTopicProperty("Depth Topic", "", "Rectified depth Image or CompressedImage; ROS graph determines type.", this, SLOT(updateSubscriptions()));
  use_color_ = new properties::BoolProperty("Use Color Image", true, "Use aligned RGB when available; otherwise use depth colors.", this, SLOT(updateSubscriptions()));
  color_topic_ = new ImageTopicProperty("Color Topic", "", "RGB image already aligned to the depth image.", this, SLOT(updateSubscriptions()));
  info_topic_ = new properties::RosTopicProperty("CameraInfo Topic", "", "sensor_msgs/msg/CameraInfo", "Calibration for rectified depth; P, ROI and binning determine projection.", this, SLOT(updateSubscriptions()));
  every_n_ = new properties::IntProperty("Process Every N Frames", 1, "Process frame N, 2N, ...; skipped depth frames are not decoded.", this, SLOT(updateSettings()));
  stride_ = new properties::IntProperty("Pixel Stride", 1, "Sample every Nth pixel in both dimensions.", this, SLOT(updateSettings()));
  max_depth_ = new properties::FloatProperty("Max Depth (m)", 0, "Maximum optical Z; 0 disables filtering.", this, SLOT(updateSettings()));
  point_size_ = new properties::FloatProperty("Point Size (px)", 3, "Points size in screen pixels, subject to GPU support.", this, SLOT(updatePointSize()));
  tolerance_ = new properties::FloatProperty("Color Time Tolerance (s)", 0.1, "Maximum timestamp difference; never wait for RGB.", this, SLOT(updateSettings()));
  color_min_ = new properties::FloatProperty("Color Min Depth (m)", 0, "Blue endpoint; does not filter points.", this, SLOT(updateSettings()));
  color_max_ = new properties::FloatProperty("Color Max Depth (m)", 5, "Red endpoint; does not filter points.", this, SLOT(updateSettings()));
  every_n_->setMin(1); stride_->setMin(1);
  for (auto * property : {max_depth_, tolerance_, color_min_, color_max_}) { property->setMin(0); }
  point_size_->setMin(1);
  std::array<properties::Property *, 3> topics{depth_topic_, color_topic_, info_topic_};
  for (size_t i = 0; i < inputs_.size(); ++i) {
    auto & input = inputs_[i];
    input.reliability = new properties::EnumProperty("Reliability", "Best Effort", "Subscription reliability.", topics[i], SLOT(updateSubscriptions()), this);
    input.reliability->addOption("Best Effort", 0); input.reliability->addOption("Reliable", 1);
    input.durability = new properties::EnumProperty("Durability", "Volatile", "Subscription durability.", topics[i], SLOT(updateSubscriptions()), this);
    input.durability->addOption("Volatile", 0); input.durability->addOption("Transient Local", 1);
  }
}
DepthViewDisplay::~DepthViewDisplay()
{
  stopWorker();
  for (auto & input : inputs_) { input.subscription.reset(); }
  if (cloud_) { scene_node_->detachObject(cloud_.get()); }
}
void DepthViewDisplay::onInitialize()
{
  const auto node = context_->getRosNodeAbstraction();
  depth_topic_->initialize(node); color_topic_->initialize(node); info_topic_->initialize(node);
  cloud_ = std::make_unique<rviz_rendering::PointCloud>();
  cloud_->setRenderMode(rviz_rendering::PointCloud::RM_POINTS);
  scene_node_->attachObject(cloud_.get());
  updateSettings();
  updatePointSize();
}
rclcpp::QoS DepthViewDisplay::qos(size_t index) const
{
  // DDS retains at most one waiting message per input. RGB pairing has its own bounded cache.
  rclcpp::QoS value(rclcpp::KeepLast(1));
  value.reliability(inputs_[index].reliability->getOptionInt() ? RMW_QOS_POLICY_RELIABILITY_RELIABLE : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  value.durability(inputs_[index].durability->getOptionInt() ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL : RMW_QOS_POLICY_DURABILITY_VOLATILE);
  return value;
}
void DepthViewDisplay::clearCloud()
{
  waiting_transform_.reset();
  if (cloud_) { cloud_->clearAndRemoveAllPoints(); }
  if (context_) { context_->queueRender(); }
}
void DepthViewDisplay::stopWorker()
{
  processor_->stop();
  if (worker_.joinable()) { worker_.join(); }
  std::lock_guard<std::mutex> lock(completed_mutex_);
  completed_.reset();
}
void DepthViewDisplay::onEnable() { updateSubscriptions(); }
void DepthViewDisplay::onDisable()
{
  stopWorker();
  for (auto & input : inputs_) { input.subscription.reset(); input.type.clear(); }
  processor_->reset(); clearCloud();
}
void DepthViewDisplay::reset()
{
  rviz_common::Display::reset();
  updateSubscriptions();
}
void DepthViewDisplay::fixedFrameChanged() { reset(); }
void DepthViewDisplay::updateSettings()
{
  stopWorker();
  DepthViewProcessor::Settings settings;
  settings.every_n = every_n_->getInt(); settings.stride = stride_->getInt();
  settings.max_depth = max_depth_->getFloat(); settings.use_color = use_color_->getBool();
  settings.color_tolerance = tolerance_->getFloat();
  settings.color_min = color_min_->getFloat(); settings.color_max = color_max_->getFloat();
  try {
    settings_valid_ = false;
    processor_->configure(settings);
    settings_valid_ = true;
    deleteStatus("Settings");
  } catch (const std::exception & e) {
    setStatus(properties::StatusProperty::Error, "Settings", e.what());
  }
  clearCloud();
  if (cloud_ && isEnabled() && settings_valid_) {
  const auto processor = processor_;
  worker_ = std::thread([this, processor] {
    while (auto result = processor->waitAndProcess()) {
      std::lock_guard<std::mutex> lock(completed_mutex_);
      completed_ = std::move(result);
    }
  });
  }
}
void DepthViewDisplay::updatePointSize()
{
  const auto size = point_size_->getFloat();
  if (!std::isfinite(size) || size < 1) {
    setStatus(properties::StatusProperty::Error, "Point Size", "Point size must be finite and at least 1 px");
    return;
  }
  if (cloud_) { cloud_->setDimensions(size, size, size); }
  deleteStatus("Point Size");
  queueRender();
}
void DepthViewDisplay::updateSubscriptions()
{
  stopWorker();
  for (auto & input : inputs_) { input.subscription.reset(); input.type.clear(); }
  // Old callbacks retain only their old processor, never the display or the new stream.
  processor_ = std::make_shared<DepthViewProcessor>();
  updateSettings();
  color_topic_->setHidden(!use_color_->getBool());
  deleteStatus("Depth Topic"); deleteStatus("Color Topic"); deleteStatus("CameraInfo Topic");
  if (!context_ || !isEnabled()) { return; }
  discover();
}
void DepthViewDisplay::subscribeImage(size_t index, ImageTopicProperty * topic)
{
  auto & input = inputs_[index];
  const auto name = topic->getTopicStd();
  const auto key = index == 0 ? "Depth Topic" : "Color Topic";
  auto node_interface = context_->getRosNodeAbstraction().lock();
  if (!node_interface) { setStatus(properties::StatusProperty::Error, key, "ROS node unavailable"); return; }
  const auto node = node_interface->get_raw_node();
  try {
    if (name.empty()) { setStatus(properties::StatusProperty::Warn, key, "No topic selected"); return; }
    const auto resolved = node->get_node_topics_interface()->resolve_topic_name(name);
    const auto topics = node->get_topic_names_and_types();
    const auto entry = topics.find(resolved);
    if (entry == topics.end() || entry->second.empty()) {
      setStatus(properties::StatusProperty::Warn, key, "Waiting for topic type from ROS graph"); return;
    }
    if (entry->second.size() != 1) {
      setStatus(properties::StatusProperty::Error, key, "Topic has multiple message types"); return;
    }
    const auto type = entry->second.front();
    if (type != "sensor_msgs/msg/Image" && type != "sensor_msgs/msg/CompressedImage") {
      setStatus(properties::StatusProperty::Error, key, "Unsupported image topic type"); return;
    }
    const auto processor = processor_;
    if (type == "sensor_msgs/msg/Image") {
      input.subscription = node->create_subscription<sensor_msgs::msg::Image>(name, qos(index),
        [processor, index](sensor_msgs::msg::Image::ConstSharedPtr image) {
          if (index == 0) { processor->depth(image); } else { processor->color(image); }
        });
    } else {
      input.subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(name, qos(index),
        [processor, index](sensor_msgs::msg::CompressedImage::ConstSharedPtr image) {
          if (index == 0) { processor->depth(image); } else { processor->color(image); }
        });
    }
    input.type = type;
    setStatus(properties::StatusProperty::Ok, key, QString::fromStdString("Subscribed: " + type));
  } catch (const std::exception & e) { setStatus(properties::StatusProperty::Error, key, e.what()); }
}
void DepthViewDisplay::discover()
{
  next_discovery_ = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  if (!inputs_[0].subscription) { subscribeImage(0, depth_topic_); }
  if (use_color_->getBool() && !inputs_[1].subscription) { subscribeImage(1, color_topic_); }
  if (inputs_[2].subscription) { return; }
  try {
    auto node_interface = context_->getRosNodeAbstraction().lock();
    if (!node_interface) { throw std::runtime_error("ROS node unavailable"); }
    const auto node = node_interface->get_raw_node();
    const auto name = info_topic_->getTopicStd();
    if (name.empty()) { throw std::runtime_error("No CameraInfo topic selected"); }
    const auto resolved = node->get_node_topics_interface()->resolve_topic_name(name);
    const auto topics = node->get_topic_names_and_types();
    const auto entry = topics.find(resolved);
    if (entry == topics.end() || entry->second.empty()) {
      setStatus(properties::StatusProperty::Warn, "CameraInfo Topic", "Waiting for topic type from ROS graph"); return;
    }
    if (entry->second.size() != 1 || entry->second.front() != "sensor_msgs/msg/CameraInfo") {
      throw std::runtime_error("CameraInfo topic must have exactly one CameraInfo type");
    }
    const auto processor = processor_;
    inputs_[2].subscription = node->create_subscription<sensor_msgs::msg::CameraInfo>(name, qos(2),
      [processor](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) { processor->cameraInfo(info); });
    setStatus(properties::StatusProperty::Ok, "CameraInfo Topic", "Subscribed");
  } catch (const std::exception & e) { setStatus(properties::StatusProperty::Error, "CameraInfo Topic", e.what()); }
}
void DepthViewDisplay::update(float, float)
{
  if (!isEnabled() || !cloud_) { return; }
  if (std::chrono::steady_clock::now() >= next_discovery_) { discover(); }
  if (!settings_valid_) { return; }
  {
    std::lock_guard<std::mutex> lock(completed_mutex_);
    if (completed_) { waiting_transform_ = std::move(completed_); completed_.reset(); }
  }
  if (!waiting_transform_) { return; }
  auto & result = *waiting_transform_;
  if (!result.error.empty()) {
    setStatus(properties::StatusProperty::Error, "Depth", QString::fromStdString(result.error));
    clearCloud(); return;
  }
  Ogre::Vector3 position; Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(result.header, position, orientation)) {
    setStatus(properties::StatusProperty::Error, "Transform", "Waiting for depth frame transform to Fixed Frame");
    cloud_->clearAndRemoveAllPoints(); context_->queueRender(); return;
  }
  std::vector<rviz_rendering::PointCloud::Point> points;
  points.reserve(result.points.size());
  for (const auto & source : result.points) {
    rviz_rendering::PointCloud::Point point;
    point.position = Ogre::Vector3(source.x, source.y, source.z);
    point.setColor(source.r, source.g, source.b); points.push_back(point);
  }
  cloud_->clearAndRemoveAllPoints();
  cloud_->addPoints(points.begin(), points.end());
  scene_node_->setPosition(position); scene_node_->setOrientation(orientation);
  setStatus(properties::StatusProperty::Ok, "Transform", "OK");
  setStatus(properties::StatusProperty::Ok, "Depth", QString("Showing %1 points").arg(points.size()));
  setStatus(result.color_status.empty() || !use_color_->getBool() ? properties::StatusProperty::Ok : properties::StatusProperty::Warn,
    "Color", result.color_status.empty() ? "Using RGB" : QString::fromStdString(result.color_status));
  waiting_transform_.reset(); context_->queueRender();
}
}  // namespace arc_rviz_plugins
PLUGINLIB_EXPORT_CLASS(arc_rviz_plugins::DepthViewDisplay, rviz_common::Display)
