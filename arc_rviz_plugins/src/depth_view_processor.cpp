#include "depth_view_processor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <cstring>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <compressed_depth_image_transport/compression_common.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_geometry/pinhole_camera_model.hpp>

namespace arc_rviz_plugins
{
namespace
{
const std_msgs::msg::Header & header(const DepthViewProcessor::Image & image)
{
  return std::visit([](const auto & msg) -> const std_msgs::msg::Header & { return msg->header; }, image);
}
int64_t stamp(const std_msgs::msg::Header & h)
{
  return static_cast<int64_t>(h.stamp.sec) * 1000000000LL + h.stamp.nanosec;
}
// RVL stores variable-length integers as high-to-low nibbles in native uint32 words.
// Unlike the transport decoder, this reader checks every input and output boundary.
class RvlReader
{
public:
  RvlReader(const uint8_t * data, size_t size) : data_(data), size_(size) {}
  uint32_t integer()
  {
    uint32_t value = 0;
    for (unsigned shift = 0; shift <= 30; shift += 3) {
      if (remaining_ == 0) {
        if (size_ < 4) { throw std::runtime_error("Truncated RVL payload"); }
        std::memcpy(&word_, data_, 4); data_ += 4; size_ -= 4; remaining_ = 8;
      }
      const uint32_t nibble = word_ >> 28;
      word_ <<= 4; --remaining_;
      if (shift == 30 && (nibble & 7) > 3) { throw std::runtime_error("RVL integer overflow"); }
      value |= (nibble & 7) << shift;
      if (!(nibble & 8)) { return value; }
    }
    throw std::runtime_error("Invalid RVL integer");
  }
private:
  const uint8_t * data_;
  size_t size_;
  uint32_t word_ = 0;
  unsigned remaining_ = 0;
};
cv::Mat compressedDepth(const sensor_msgs::msg::CompressedImage & message)
{
  using compressed_depth_image_transport::ConfigHeader;
  std::istringstream format(message.format);
  std::string encoding, transport, codec, extra;
  format >> encoding >> transport >> codec >> extra;
  if ((encoding != "16UC1;" && encoding != "32FC1;") || transport != "compressedDepth" ||
    (!codec.empty() && codec != "png" && codec != "rvl") || !extra.empty() ||
    message.data.size() <= sizeof(ConfigHeader))
  {
    throw std::runtime_error("Unsupported or malformed compressedDepth format/payload");
  }
  ConfigHeader config;
  std::memcpy(&config, message.data.data(), sizeof(config));
  const auto * payload = message.data.data() + sizeof(config);
  const size_t length = message.data.size() - sizeof(config);
  cv::Mat decoded;
  if (codec != "rvl") {
    static const uint8_t signature[] = {137, 80, 78, 71, 13, 10, 26, 10};
    if (length < sizeof(signature) || std::memcmp(payload, signature, sizeof(signature)) != 0 ||
      length > static_cast<size_t>(std::numeric_limits<int>::max()))
    { throw std::runtime_error("compressedDepth payload is not PNG"); }
    const cv::Mat bytes(1, static_cast<int>(length), CV_8UC1, const_cast<uint8_t *>(payload));
    decoded = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
  } else {
    if (length < 12) { throw std::runtime_error("Truncated RVL header"); }
    uint32_t width, height;
    std::memcpy(&width, payload, 4); std::memcpy(&height, payload + 4, 4);
    const uint64_t count = static_cast<uint64_t>(width) * height;
    if (!width || !height || width > 16384 || height > 16384 || count > 16777216) {
      throw std::runtime_error("Invalid RVL dimensions");
    }
    decoded = cv::Mat::zeros(height, width, CV_16UC1);
    RvlReader reader(payload + 8, length - 8);
    auto * values = decoded.ptr<uint16_t>();
    size_t position = 0;
    int64_t previous = 0;
    while (position < count) {
      const auto zeros = reader.integer();
      if (zeros > count - position) { throw std::runtime_error("RVL zero run exceeds image"); }
      position += zeros;
      const auto nonzeros = reader.integer();
      if (nonzeros > count - position || (!zeros && !nonzeros)) {
        throw std::runtime_error("Invalid RVL nonzero run");
      }
      for (uint32_t i = 0; i < nonzeros; ++i) {
        const auto value = reader.integer();
        previous += (value & 1) ? -static_cast<int64_t>((static_cast<uint64_t>(value) + 1) / 2) : value / 2;
        if (previous <= 0 || previous > 65535) { throw std::runtime_error("Invalid RVL depth sample"); }
        values[position++] = static_cast<uint16_t>(previous);
      }
    }
  }
  if (decoded.empty() || decoded.type() != CV_16UC1) {
    throw std::runtime_error("compressedDepth must decode to a 16-bit single-channel image");
  }
  if (encoding == "16UC1;") { return decoded; }
  if (config.format != compressed_depth_image_transport::INV_DEPTH ||
    !std::isfinite(config.depthParam[0]) || config.depthParam[0] <= 0 ||
    !std::isfinite(config.depthParam[1]))
  { throw std::runtime_error("Invalid compressedDepth inverse-depth parameters"); }
  cv::Mat meters(decoded.size(), CV_32FC1);
  for (int v = 0; v < decoded.rows; ++v) {
    for (int u = 0; u < decoded.cols; ++u) {
      const auto value = decoded.at<uint16_t>(v, u);
      const float denominator = value - config.depthParam[1];
      meters.at<float>(v, u) = value && denominator > 0 ? config.depthParam[0] / denominator :
        std::numeric_limits<float>::quiet_NaN();
    }
  }
  return meters;
}

cv::Mat decode(const DepthViewProcessor::Image & image, bool depth)
{
  if (const auto raw = std::get_if<sensor_msgs::msg::Image::ConstSharedPtr>(&image)) {
    if (depth && (*raw)->encoding != "16UC1" && (*raw)->encoding != "32FC1") {
      throw std::runtime_error("Unsupported depth encoding: " + (*raw)->encoding);
    }
    if (!depth) { return cv_bridge::toCvCopy(*raw, "rgb8")->image; }
    const auto & message = **raw;
    const size_t bytes = message.encoding == "16UC1" ? 2 : 4;
    if (!message.width || !message.height || message.width > std::numeric_limits<int>::max() ||
      message.height > std::numeric_limits<int>::max() ||
      message.step < static_cast<uint64_t>(message.width) * bytes ||
      static_cast<uint64_t>(message.height) * message.step != message.data.size())
    { throw std::runtime_error("Malformed raw depth dimensions/step/payload"); }
    const int type = bytes == 2 ? CV_16UC1 : CV_32FC1;
    const uint16_t endian_probe = 1;
    const bool native_big = *reinterpret_cast<const uint8_t *>(&endian_probe) == 0;
    if (native_big == static_cast<bool>(message.is_bigendian) && message.step % bytes == 0) {
      return cv::Mat(message.height, message.width, type, const_cast<uint8_t *>(message.data.data()), message.step);
    }
    // Preserve the scalar type when swapping; some cv_bridge versions return byte channels.
    cv::Mat converted(message.height, message.width, type);
    for (uint32_t v = 0; v < message.height; ++v) {
      auto * output = converted.ptr<uint8_t>(v);
      const auto * input = message.data.data() + static_cast<size_t>(v) * message.step;
      if (native_big == static_cast<bool>(message.is_bigendian)) {
        std::memcpy(output, input, static_cast<size_t>(message.width) * bytes);
      } else {
        for (size_t u = 0; u < message.width; ++u) {
          for (size_t b = 0; b < bytes; ++b) { output[u * bytes + b] = input[u * bytes + bytes - 1 - b]; }
        }
      }
    }
    return converted;
  }
  const auto compressed = std::get<sensor_msgs::msg::CompressedImage::ConstSharedPtr>(image);
  if (depth) { return compressedDepth(*compressed); }
  if (compressed->format.find("compressedDepth") != std::string::npos) {
    throw std::runtime_error("RGB input cannot use compressedDepth");
  }
  const auto & data = compressed->data;
  const bool png = data.size() >= 8 && data[0] == 137 && data[1] == 80 && data[2] == 78 && data[3] == 71;
  const bool jpeg = data.size() >= 3 && data[0] == 255 && data[1] == 216 && data[2] == 255;
  const bool declares_png = compressed->format == "png" || compressed->format.find("; png compressed") != std::string::npos;
  const bool declares_jpeg = compressed->format == "jpeg" || compressed->format.find("; jpeg compressed") != std::string::npos;
  if ((!png || !declares_png) && (!jpeg || !declares_jpeg)) {
    throw std::runtime_error("Unsupported or mismatched RGB compressed format/payload");
  }
  return cv_bridge::toCvCopy(compressed, "rgb8")->image;
}
}
void DepthViewProcessor::configure(const Settings & settings)
{
  if (settings.every_n < 1 || settings.stride < 1 || !std::isfinite(settings.max_depth) ||
    settings.max_depth < 0 || !std::isfinite(settings.color_tolerance) || settings.color_tolerance < 0 ||
    !std::isfinite(settings.color_min) || !std::isfinite(settings.color_max) ||
    settings.color_min < 0 || settings.color_max <= settings.color_min)
  {
    throw std::invalid_argument("Invalid DepthView settings");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  settings_ = settings;
  stopped_ = false;
  received_ = 0;
  pending_.reset();
  if (!settings.use_color) { colors_.clear(); }
}
void DepthViewProcessor::cameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
  std::lock_guard<std::mutex> lock(mutex_);
  info_ = std::move(info);
}
void DepthViewProcessor::depth(Image image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (++received_ < settings_.every_n) { return; }
  received_ = 0;
  pending_ = std::move(image);
  available_.notify_one();
}
void DepthViewProcessor::color(Image image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!settings_.use_color) { return; }
  colors_.push_back(std::move(image));
  if (colors_.size() > 30) { colors_.pop_front(); }
}
void DepthViewProcessor::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  info_.reset(); pending_.reset(); colors_.clear(); received_ = 0;
}
void DepthViewProcessor::stop()
{
  std::lock_guard<std::mutex> lock(mutex_);
  stopped_ = true;
  available_.notify_all();
}
std::optional<DepthViewProcessor::Result> DepthViewProcessor::waitAndProcess()
{
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(mutex_);
      available_.wait(lock, [this] { return stopped_ || pending_.has_value(); });
      if (stopped_) { return {}; }
    }
    if (auto result = processLatest()) { return result; }
  }
}
std::optional<DepthViewProcessor::Result> DepthViewProcessor::processLatest()
{
  Image image;
  std::optional<Image> rgb;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  Settings settings;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_) { return {}; }
    image = std::move(*pending_); pending_.reset();
    info = info_; settings = settings_;
    double nearest = std::numeric_limits<double>::infinity();
    if (settings.use_color) {
      for (const auto & candidate : colors_) {
        const double delta = std::abs(static_cast<double>(stamp(header(candidate)) - stamp(header(image)))) / 1e9;
        if (delta <= settings.color_tolerance && delta < nearest) { nearest = delta; rgb = candidate; }
      }
    }
  }
  Result result;
  result.header = header(image);
  try {
    if (!info || info->width == 0 || info->height == 0 ||
      !std::isfinite(info->k[0]) || !std::isfinite(info->k[4]) ||
      !std::isfinite(info->k[2]) || !std::isfinite(info->k[5]) ||
      info->k[0] <= 0 || info->k[4] <= 0)
    {
      throw std::runtime_error("Missing or invalid depth CameraInfo");
    }
    image_geometry::PinholeCameraModel camera;
    camera.fromCameraInfo(*info);
    if (!std::isfinite(camera.fx()) || !std::isfinite(camera.fy()) ||
      !std::isfinite(camera.cx()) || !std::isfinite(camera.cy()) || camera.fx() <= 0 || camera.fy() <= 0)
    { throw std::runtime_error("Invalid rectified CameraInfo projection"); }
    const cv::Mat mat = decode(image, true);
    if (mat.empty() || mat.size() != camera.reducedResolution())
    {
      throw std::runtime_error("Depth image dimensions do not match CameraInfo");
    }
    cv::Mat color;
    if (!settings.use_color) { result.color_status = "Color disabled; using depth colors"; }
    else if (!rgb) { result.color_status = "No RGB within time tolerance; using depth colors"; }
    else {
      try {
        color = decode(*rgb, false);
        if (color.size() != mat.size()) {
          color.release(); result.color_status = "RGB dimensions differ; using depth colors";
        }
      } catch (const std::exception & e) {
        result.color_status = std::string("RGB decode failed; using depth colors: ") + e.what();
      }
    }
    const size_t columns = (static_cast<size_t>(mat.cols) - 1) / settings.stride + 1;
    const size_t rows = (static_cast<size_t>(mat.rows) - 1) / settings.stride + 1;
    result.points.reserve(columns * rows);
    for (int64_t v = 0; v < mat.rows; v += settings.stride) {
      for (int64_t u = 0; u < mat.cols; u += settings.stride) {
        const float z = mat.type() == CV_16UC1 ? mat.at<uint16_t>(v, u) * 0.001f : mat.at<float>(v, u);
        if (!std::isfinite(z) || z <= 0 || (settings.max_depth > 0 && z > settings.max_depth)) { continue; }
        const float t = std::clamp((z - settings.color_min) / (settings.color_max - settings.color_min), 0.0, 1.0);
        Point point{static_cast<float>((u - camera.cx()) * z / camera.fx()),
          static_cast<float>((v - camera.cy()) * z / camera.fy()), z, t, 0, 1 - t};
        if (!color.empty()) {
          const auto pixel = color.at<cv::Vec3b>(v, u);
          point.r = pixel[0] / 255.0f; point.g = pixel[1] / 255.0f; point.b = pixel[2] / 255.0f;
        }
        if (std::isfinite(point.x) && std::isfinite(point.y)) { result.points.push_back(point); }
      }
    }
  } catch (const std::exception & e) { result.error = e.what(); }
  return result;
}
}  // namespace arc_rviz_plugins
