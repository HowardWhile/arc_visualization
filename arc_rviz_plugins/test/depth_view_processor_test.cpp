#include "depth_view_processor.h"
#include <gtest/gtest.h>
#include <future>
#include <compressed_depth_image_transport/codec.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.hpp>

using arc_rviz_plugins::DepthViewProcessor;
namespace
{
auto info(unsigned width = 2, unsigned height = 1)
{
  auto msg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  msg->width = width; msg->height = height;
  msg->k = {2, 0, 0, 0, 2, 0, 0, 0, 1};
  msg->p = {2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1, 0};
  return msg;
}
auto image(const cv::Mat & data, const std::string & encoding, int seconds = 1)
{
  std_msgs::msg::Header header;
  header.frame_id = "camera"; header.stamp.sec = seconds;
  return cv_bridge::CvImage(header, encoding, data).toImageMsg();
}
}
TEST(DepthView, ProjectsMillimetersIntoMeters)
{
  DepthViewProcessor processor;
  processor.cameraInfo(info());
  processor.depth(image((cv::Mat_<uint16_t>(1, 2) << 1000, 2000), "16UC1"));
  auto result = processor.processLatest();
  ASSERT_TRUE(result); ASSERT_TRUE(result->error.empty()) << result->error;
  ASSERT_EQ(result->points.size(), 2u);
  EXPECT_FLOAT_EQ(result->points[0].z, 1);
  EXPECT_FLOAT_EQ(result->points[1].x, 1);
  EXPECT_FLOAT_EQ(result->points[1].z, 2);
}
TEST(DepthView, FiltersInvalidAndFarDepthBeforeSampling)
{
  DepthViewProcessor processor;
  DepthViewProcessor::Settings settings;
  settings.max_depth = 2; settings.stride = 2;
  processor.configure(settings);
  processor.cameraInfo(info(7));
  processor.depth(image((cv::Mat_<float>(1, 7) << 1, 9, 2, 9, 2.001, 9,
      std::numeric_limits<float>::quiet_NaN()), "32FC1"));
  auto result = processor.processLatest();
  ASSERT_TRUE(result); ASSERT_TRUE(result->error.empty()) << result->error;
  ASSERT_EQ(result->points.size(), 2u);
  EXPECT_FLOAT_EQ(result->points[1].x, 2);
  EXPECT_FLOAT_EQ(result->points[1].z, 2);
}
TEST(DepthView, SkipsFramesAndKeepsOnlyNewestEligibleFrame)
{
  DepthViewProcessor processor;
  DepthViewProcessor::Settings settings; settings.every_n = 3;
  processor.configure(settings); processor.cameraInfo(info());
  auto frame = image((cv::Mat_<uint16_t>(1, 2) << 1000, 1000), "16UC1");
  processor.depth(frame); EXPECT_FALSE(processor.processLatest());
  processor.depth(frame); EXPECT_FALSE(processor.processLatest());
  processor.depth(frame);
  processor.depth(frame); processor.depth(frame);
  processor.depth(image((cv::Mat_<uint16_t>(1, 2) << 2000, 2000), "16UC1", 6));
  auto result = processor.processLatest();
  ASSERT_TRUE(result); EXPECT_EQ(result->header.stamp.sec, 6);
  EXPECT_FLOAT_EQ(result->points[0].z, 2);
  EXPECT_FALSE(processor.processLatest());
}
TEST(DepthView, UsesNearestRgbAndFallsBackOutsideTolerance)
{
  DepthViewProcessor processor;
  processor.cameraInfo(info());
  processor.color(image(cv::Mat(1, 2, CV_8UC3, cv::Scalar(255, 0, 0)), "rgb8", 1));
  processor.color(image(cv::Mat(1, 2, CV_8UC3, cv::Scalar(0, 255, 0)), "rgb8", 2));
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(5)), "32FC1", 1));
  auto result = processor.processLatest();
  ASSERT_TRUE(result); ASSERT_EQ(result->points.size(), 2u);
  EXPECT_FLOAT_EQ(result->points[0].r, 1); EXPECT_FLOAT_EQ(result->points[0].g, 0);
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(0.5)), "32FC1", 4));
  result = processor.processLatest();
  EXPECT_FALSE(result->color_status.empty());
  EXPECT_GT(result->points[0].b, result->points[0].r);
}

TEST(DepthView, DecodesOfficialCompressedDepthWithoutTopicHints)
{
  for (const auto & codec : {"png", "rvl"}) {
    for (const auto & encoding : {"16UC1", "32FC1"}) {
      DepthViewProcessor processor; processor.cameraInfo(info());
      auto raw = std::string(encoding) == "16UC1" ?
        image((cv::Mat_<uint16_t>(1, 2) << 1000, 2000), encoding) :
        image((cv::Mat_<float>(1, 2) << 1, 2), encoding);
      auto compressed = compressed_depth_image_transport::encodeCompressedDepthImage(*raw, codec);
      ASSERT_TRUE(compressed);
      processor.depth(compressed);
      const auto result = processor.processLatest();
      ASSERT_TRUE(result); ASSERT_TRUE(result->error.empty()) << codec << encoding << result->error;
      ASSERT_EQ(result->points.size(), 2u);
      EXPECT_NEAR(result->points[0].z, 1, 0.002);
      EXPECT_NEAR(result->points[1].x, 1, 0.002);
      EXPECT_NEAR(result->points[1].z, 2, 0.002);
    }
  }
}
TEST(DepthView, UsesRectifiedCameraProjection)
{
  DepthViewProcessor processor;
  auto calibration = info();
  calibration->p[0] = 4;
  processor.cameraInfo(calibration);
  processor.depth(image((cv::Mat_<uint16_t>(1, 2) << 2000, 2000), "16UC1"));
  const auto result = processor.processLatest();
  ASSERT_TRUE(result); ASSERT_EQ(result->points.size(), 2u);
  EXPECT_FLOAT_EQ(result->points[1].x, 0.5);
}
TEST(DepthView, InvalidInputsReportErrorsInsteadOfProducingPoints)
{
  DepthViewProcessor processor;
  auto raw = image(cv::Mat(1, 2, CV_16UC1, cv::Scalar(1000)), "16UC1");
  processor.depth(raw);
  ASSERT_FALSE(processor.processLatest()->error.empty());
  auto invalid = info(); invalid->p[0] = 0;
  processor.cameraInfo(invalid); processor.depth(raw);
  ASSERT_FALSE(processor.processLatest()->error.empty());
  processor.cameraInfo(info());
  for (const auto & encoding : {"mono16", "rgb8"}) {
    processor.depth(image(cv::Mat(1, 2, CV_16UC1, cv::Scalar(1000)), encoding));
    auto result = processor.processLatest();
    ASSERT_TRUE(result); EXPECT_FALSE(result->error.empty()); EXPECT_TRUE(result->points.empty());
  }
}
TEST(DepthView, IgnoresAllNonpositiveOrNonfiniteDepths)
{
  DepthViewProcessor processor; processor.cameraInfo(info(5));
  processor.depth(image((cv::Mat_<float>(1, 5) << 0, -1,
      std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity(), 3), "32FC1"));
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  ASSERT_EQ(result->points.size(), 1u); EXPECT_FLOAT_EQ(result->points[0].z, 3);
}
TEST(DepthView, ColorToggleAndSizeMismatchFallBackToFixedDepthColors)
{
  DepthViewProcessor processor;
  processor.cameraInfo(info());
  processor.color(image(cv::Mat(1, 1, CV_8UC3, cv::Scalar(0, 255, 0)), "rgb8"));
  processor.depth(image((cv::Mat_<float>(1, 2) << 0.5, 6), "32FC1"));
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  EXPECT_NE(result->color_status.find("dimensions"), std::string::npos);
  EXPECT_FLOAT_EQ(result->points[1].r, 1); EXPECT_FLOAT_EQ(result->points[1].b, 0);
  DepthViewProcessor::Settings settings; settings.use_color = false; settings.color_min = 1; settings.color_max = 2;
  processor.configure(settings);
  processor.color(image(cv::Mat(1, 2, CV_8UC3, cv::Scalar(0, 255, 0)), "rgb8"));
  processor.depth(image((cv::Mat_<float>(1, 2) << 0.5, 6), "32FC1"));
  result = processor.processLatest();
  ASSERT_EQ(result->points.size(), 2u);
  EXPECT_FLOAT_EQ(result->points[0].b, 1); EXPECT_FLOAT_EQ(result->points[1].r, 1);
}
TEST(DepthView, CompressedRgbUsesActualPixelsAndRejectsMismatchedCodec)
{
  DepthViewProcessor processor; processor.cameraInfo(info());
  auto compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
  compressed->header.stamp.sec = 1; compressed->format = "rgb8; png compressed bgr8";
  cv::imencode(".png", cv::Mat(1, 2, CV_8UC3, cv::Scalar(0, 0, 255)), compressed->data);
  processor.color(compressed);
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(1)), "32FC1"));
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  ASSERT_TRUE(result->color_status.empty()) << result->color_status;
  EXPECT_FLOAT_EQ(result->points[0].r, 1); EXPECT_FLOAT_EQ(result->points[0].b, 0);
  processor.reset(); processor.cameraInfo(info());
  compressed->format = "rgb8; jpeg compressed bgr8"; processor.color(compressed);
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(1)), "32FC1"));
  result = processor.processLatest(); EXPECT_FALSE(result->color_status.empty());
  EXPECT_GT(result->points[0].b, result->points[0].r);
}
TEST(DepthView, MalformedCompressedDepthIsRejectedAndSkippedFramesAreNotDecoded)
{
  DepthViewProcessor processor; processor.cameraInfo(info());
  for (auto codec : {"png", "rvl", "unknown"}) {
    auto compressed = std::make_shared<sensor_msgs::msg::CompressedImage>();
    compressed->format = std::string("16UC1; compressedDepth ") + codec;
    for (size_t size : {0u, 4u, 12u, 16u, 32u}) {
      compressed->data.assign(size, 255);
      processor.depth(compressed);
      auto result = processor.processLatest(); ASSERT_TRUE(result);
      EXPECT_FALSE(result->error.empty()); EXPECT_TRUE(result->points.empty());
    }
  }
  DepthViewProcessor::Settings settings; settings.every_n = 2; processor.configure(settings);
  processor.depth(std::make_shared<sensor_msgs::msg::CompressedImage>());
  EXPECT_FALSE(processor.processLatest());
  processor.depth(image(cv::Mat(1, 2, CV_16UC1, cv::Scalar(1000)), "16UC1"));
  EXPECT_TRUE(processor.processLatest()->error.empty());
}
TEST(DepthView, BoundedRgbCacheDropsOldFramesAndResetClearsCalibration)
{
  DepthViewProcessor processor; processor.cameraInfo(info());
  for (int second = 1; second <= 31; ++second) {
    processor.color(image(cv::Mat(1, 2, CV_8UC3, cv::Scalar(0, 255, 0)), "rgb8", second));
  }
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(1)), "32FC1", 1));
  auto result = processor.processLatest(); EXPECT_FALSE(result->color_status.empty());
  processor.reset();
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(1)), "32FC1"));
  EXPECT_FALSE(processor.processLatest()->error.empty());
}
TEST(DepthView, RejectsInvalidSettings)
{
  DepthViewProcessor processor;
  DepthViewProcessor::Settings settings;
  settings.every_n = 0; EXPECT_THROW(processor.configure(settings), std::invalid_argument);
  settings.every_n = 1; settings.stride = 0; EXPECT_THROW(processor.configure(settings), std::invalid_argument);
  settings.stride = 1; settings.color_max = 0; EXPECT_THROW(processor.configure(settings), std::invalid_argument);
  settings.color_max = 5; settings.color_tolerance = -1; EXPECT_THROW(processor.configure(settings), std::invalid_argument);
}
TEST(DepthView, CameraBinningUsesReducedResolutionAndFocalLength)
{
  DepthViewProcessor processor;
  auto calibration = info(4, 2); calibration->binning_x = 2; calibration->binning_y = 2;
  processor.cameraInfo(calibration);
  processor.depth(image((cv::Mat_<uint16_t>(1, 2) << 2000, 2000), "16UC1"));
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  ASSERT_TRUE(result->error.empty()) << result->error;
  ASSERT_EQ(result->points.size(), 2u); EXPECT_FLOAT_EQ(result->points[1].x, 2);
}
TEST(DepthView, ColorToleranceIncludesBoundaryAndSelectsClosestTimestamp)
{
  DepthViewProcessor processor; processor.cameraInfo(info());
  auto red = image(cv::Mat(1, 2, CV_8UC3, cv::Scalar(255, 0, 0)), "rgb8");
  auto green = image(cv::Mat(1, 2, CV_8UC3, cv::Scalar(0, 255, 0)), "rgb8");
  red->header.stamp.nanosec = 100000000; green->header.stamp.nanosec = 20000000;
  processor.color(red); processor.color(green);
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(1)), "32FC1"));
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  EXPECT_FLOAT_EQ(result->points[0].g, 1);
  processor.reset(); processor.cameraInfo(info()); processor.color(red);
  processor.depth(image(cv::Mat(1, 2, CV_32FC1, cv::Scalar(1)), "32FC1"));
  result = processor.processLatest(); EXPECT_TRUE(result->color_status.empty());
  EXPECT_FLOAT_EQ(result->points[0].r, 1);
}
TEST(DepthView, BigEndianDepthMaintainsMetricScale)
{
  DepthViewProcessor processor; processor.cameraInfo(info());
  auto raw = image((cv::Mat_<uint16_t>(1, 2) << 1000, 2000), "16UC1");
  for (size_t i = 0; i < raw->data.size(); i += 2) { std::swap(raw->data[i], raw->data[i + 1]); }
  raw->is_bigendian = !raw->is_bigendian;
  processor.depth(raw);
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  ASSERT_TRUE(result->error.empty()) << result->error;
  EXPECT_FLOAT_EQ(result->points[1].z, 2);
}
TEST(DepthView, WaitingWorkerReceivesFramesAndStopsCleanly)
{
  DepthViewProcessor processor; processor.cameraInfo(info());
  auto future = std::async(std::launch::async, [&] { return processor.waitAndProcess(); });
  processor.depth(image(cv::Mat(1, 2, CV_16UC1, cv::Scalar(1000)), "16UC1"));
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_TRUE(future.get());
  auto stopped = std::async(std::launch::async, [&] { return processor.waitAndProcess(); });
  processor.stop();
  ASSERT_EQ(stopped.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_FALSE(stopped.get());
}
TEST(DepthView, SupportsRowPaddingThatIsNotScalarAligned)
{
  DepthViewProcessor processor; processor.cameraInfo(info(2, 2));
  auto raw = image((cv::Mat_<float>(2, 2) << 1, 2, 3, 4), "32FC1");
  auto original = raw->data;
  raw->step = 9; raw->data.assign(18, 0);
  std::copy_n(original.begin(), 8, raw->data.begin());
  std::copy_n(original.begin() + 8, 8, raw->data.begin() + 9);
  processor.depth(raw);
  auto result = processor.processLatest(); ASSERT_TRUE(result);
  ASSERT_TRUE(result->error.empty()) << result->error;
  ASSERT_EQ(result->points.size(), 4u);
  EXPECT_FLOAT_EQ(result->points[3].x, 2); EXPECT_FLOAT_EQ(result->points[3].z, 4);
}
