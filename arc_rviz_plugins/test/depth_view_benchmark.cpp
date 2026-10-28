#include "depth_view_processor.h"
#include <chrono>
#include <iostream>
#include <cv_bridge/cv_bridge.hpp>

int main()
{
  using arc_rviz_plugins::DepthViewProcessor;
  auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
  info->width = 640; info->height = 480;
  info->k = {525, 0, 319.5, 0, 525, 239.5, 0, 0, 1};
  info->p = {525, 0, 319.5, 0, 0, 525, 239.5, 0, 0, 0, 1, 0};
  std_msgs::msg::Header header; header.frame_id = "camera";
  auto depth = cv_bridge::CvImage(header, "16UC1", cv::Mat(480, 640, CV_16UC1, cv::Scalar(1500))).toImageMsg();
  auto color = cv_bridge::CvImage(header, "rgb8", cv::Mat(480, 640, CV_8UC3, cv::Scalar(80, 160, 240))).toImageMsg();
  std::cout << "every_n,stride,rgb,input_frames,output_frames,total_points,elapsed_ms\n";
  for (int n : {1, 3}) {
    for (int stride : {1, 2}) {
      for (bool rgb : {false, true}) {
        DepthViewProcessor processor;
        DepthViewProcessor::Settings settings;
        settings.every_n = n; settings.stride = stride; settings.use_color = rgb;
        processor.configure(settings); processor.cameraInfo(info);
        size_t frames = 0, points = 0;
        const auto begin = std::chrono::steady_clock::now();
        for (int i = 0; i < 180; ++i) {
          if (rgb) { processor.color(color); }
          processor.depth(depth);
          if (auto result = processor.processLatest()) {
            if (!result->error.empty()) { std::cerr << result->error; return 1; }
            ++frames; points += result->points.size();
          }
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
        std::cout << n << ',' << stride << ',' << rgb << ",180," << frames << ',' << points << ',' << ms << '\n';
      }
    }
  }
}
