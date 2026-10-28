#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include "depth_view_processor.h"

namespace arc_rviz_plugins
{
class ImageTopicProperty;
class DepthViewDisplay : public rviz_common::Display
{
  Q_OBJECT
public:
  DepthViewDisplay();
  ~DepthViewDisplay() override;
  void reset() override;
protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;
  void fixedFrameChanged() override;
private Q_SLOTS:
  void updateSubscriptions();
  void updateSettings();
  void updatePointSize();
private:
  struct Input
  {
    rviz_common::properties::EnumProperty * reliability = nullptr;
    rviz_common::properties::EnumProperty * durability = nullptr;
    rclcpp::SubscriptionBase::SharedPtr subscription;
    std::string type;
  };
  void discover();
  void stopWorker();
  void clearCloud();
  void subscribeImage(size_t index, ImageTopicProperty * topic);
  rclcpp::QoS qos(size_t index) const;
  std::shared_ptr<DepthViewProcessor> processor_;
  std::thread worker_;
  std::mutex completed_mutex_;
  std::optional<DepthViewProcessor::Result> completed_;
  bool settings_valid_ = true;
  std::unique_ptr<rviz_rendering::PointCloud> cloud_;
  std::array<Input, 3> inputs_;
  ImageTopicProperty * depth_topic_;
  ImageTopicProperty * color_topic_;
  rviz_common::properties::RosTopicProperty * info_topic_;
  rviz_common::properties::BoolProperty * use_color_;
  rviz_common::properties::IntProperty * every_n_;
  rviz_common::properties::IntProperty * stride_;
  rviz_common::properties::FloatProperty * max_depth_;
  rviz_common::properties::FloatProperty * point_size_;
  rviz_common::properties::FloatProperty * tolerance_;
  rviz_common::properties::FloatProperty * color_min_;
  rviz_common::properties::FloatProperty * color_max_;
  std::chrono::steady_clock::time_point next_discovery_;
  std::optional<DepthViewProcessor::Result> waiting_transform_;
};
}  // namespace arc_rviz_plugins
