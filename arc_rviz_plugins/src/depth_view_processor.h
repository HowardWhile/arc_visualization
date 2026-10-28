#pragma once

#include <deque>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>
#include <variant>
#include <vector>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace arc_rviz_plugins
{
// The complete image-to-cloud boundary. It owns no Qt or rendering resources.
class DepthViewProcessor
{
public:
  using Image = std::variant<sensor_msgs::msg::Image::ConstSharedPtr,
      sensor_msgs::msg::CompressedImage::ConstSharedPtr>;
  struct Settings
  {
    int every_n = 1;
    int stride = 1;
    double max_depth = 0;
    bool use_color = true;
    double color_tolerance = 0.1;
    double color_min = 0;
    double color_max = 5;
  };
  struct Point { float x, y, z, r, g, b; };
  struct Result
  {
    std_msgs::msg::Header header;
    std::vector<Point> points;
    std::string error;
    std::string color_status;
  };
  void configure(const Settings & settings);
  void cameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
  void depth(Image image);
  void color(Image image);
  std::optional<Result> processLatest();
  void reset();
  std::optional<Result> waitAndProcess();
  void stop();
private:
  std::mutex mutex_;
  std::condition_variable available_;
  bool stopped_ = false;
  Settings settings_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info_;
  std::optional<Image> pending_;
  std::deque<Image> colors_;
  int received_ = 0;
};
}  // namespace arc_rviz_plugins
