#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rviz_common/display.hpp>
#include <rviz_rendering/objects/point_cloud.hpp>
#include <OgreSceneNode.h>
#include <gtest/gtest.h>
#include <QApplication>
#include <QThread>
#include <QFileInfo>
#include <QVBoxLayout>
#include <cv_bridge/cv_bridge.hpp>
#include <compressed_depth_image_transport/codec.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/status_list.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

namespace
{
bool waitUntil(const std::function<bool()> & predicate)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(6);
  while (std::chrono::steady_clock::now() < end) {
    QCoreApplication::processEvents();
    if (predicate()) { return true; }
    QThread::msleep(10);
  }
  return false;
}
}
TEST(DepthViewIntegration, DiscoversActualTypesRendersAndManagesSubscriptions)
{
  if (!std::getenv("DISPLAY")) { GTEST_SKIP() << "Requires X server/OpenGL; run with DISPLAY set."; }
  int argc = 1; char name[] = "depth_view_integration"; char * argv[] = {name, nullptr};
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  auto abstraction = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("depth_view_test");
  auto node = abstraction->get_raw_node();
  auto publisher = std::make_shared<rclcpp::Node>("depth_view_source");
  QWidget window;
  QVBoxLayout layout(&window);
  rviz_common::RenderPanel panel;
  layout.addWidget(&panel);
  window.resize(640, 480);
  window.show();
  QCoreApplication::processEvents();
  panel.getRenderWindow()->initialize();
  rviz_common::VisualizationManager manager(&panel, abstraction, nullptr, node->get_clock());
  panel.initialize(&manager);
  manager.initialize(); manager.setFixedFrame("camera"); manager.startUpdate();

  auto * display = manager.createDisplay("arc_rviz_plugins/DepthView", "DepthView", true);
  ASSERT_NE(display, nullptr);
  ASSERT_EQ(display->getClassId(), "arc_rviz_plugins/DepthView");
  EXPECT_EQ(display->subProp("Point Size (px)")->getValue().toFloat(), 3);
  EXPECT_EQ(display->subProp("Process Every N Frames")->getValue().toInt(), 1);
  EXPECT_EQ(display->subProp("Pixel Stride")->getValue().toInt(), 1);
  display->subProp("Depth Topic")->setValue("/depth_view_test/arbitrary");
  display->subProp("Color Topic")->setValue("/depth_view_test/colors");
  display->subProp("CameraInfo Topic")->setValue("/depth_view_test/calibration");
  auto depth = publisher->create_publisher<sensor_msgs::msg::Image>("/depth_view_test/arbitrary", rclcpp::SensorDataQoS());
  auto rgb = publisher->create_publisher<sensor_msgs::msg::Image>("/depth_view_test/colors", rclcpp::SensorDataQoS());
  auto calibration = publisher->create_publisher<sensor_msgs::msg::CameraInfo>("/depth_view_test/calibration", rclcpp::SensorDataQoS());
  ASSERT_TRUE(waitUntil([&] { return depth->get_subscription_count() && rgb->get_subscription_count() && calibration->get_subscription_count(); }));
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = "camera"; info.width = 2; info.height = 1;
  info.k = {2, 0, 0, 0, 2, 0, 0, 0, 1}; info.p = {2, 0, 0, 0, 0, 2, 0, 0, 0, 0, 1, 0};
  auto raw = cv_bridge::CvImage(info.header, "16UC1", (cv::Mat_<uint16_t>(1, 2) << 1000, 2000)).toImageMsg();
  auto publish = [&] { info.header.stamp = node->now(); raw->header.stamp = info.header.stamp; calibration->publish(info); depth->publish(*raw); };
  auto status = [&](const char * key) {
    for (int i = 0; i < display->numChildren(); ++i) {
      auto * list = dynamic_cast<rviz_common::properties::StatusList *>(display->childAt(i));
      if (!list) { continue; }
      for (int j = 0; j < list->numChildren(); ++j) {
        if (list->childAt(j)->getName() == key) { return list->childAt(j)->getValue().toString(); }
      }
    }
    return QString();
  };
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Depth").contains("Showing 2 points"); }));
  auto * cloud = dynamic_cast<rviz_rendering::PointCloud *>(display->getSceneNode()->getAttachedObject(0));
  ASSERT_NE(cloud, nullptr);
  ASSERT_EQ(cloud->getPoints().size(), 2u);
  EXPECT_FLOAT_EQ(cloud->getPoints()[1].position.x, 1);
  EXPECT_FLOAT_EQ(cloud->getPoints()[1].position.z, 2);
  raw->header.frame_id = "unknown_depth_frame";
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Transform").contains("Waiting"); }));
  raw->header.frame_id = "camera";
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Depth").contains("Showing 2 points") && status("Transform") == "OK"; }));
  info.k[0] = 0;
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Depth").contains("invalid"); }));
  info.k[0] = 2;
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Depth").contains("Showing 2 points"); }));
  display->subProp("Point Size (px)")->setValue(5);
  EXPECT_EQ(cloud->getPoints().size(), 2u) << "Resizing must preserve the displayed cloud";
  display->subProp("Max Depth (m)")->setValue(1.5);
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Depth").contains("Showing 1 points"); }));
  display->subProp("Use Color Image")->setValue(false);
  ASSERT_TRUE(waitUntil([&] { return rgb->get_subscription_count() == 0 && depth->get_subscription_count() > 0; }));
  ASSERT_TRUE(waitUntil([&] { publish(); return status("Color").contains("Color disabled"); }));
  const auto qos_info = node->get_subscriptions_info_by_topic("/depth_view_test/arbitrary");
  ASSERT_EQ(qos_info.size(), 1u);
  EXPECT_EQ(qos_info.front().qos_profile().reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(qos_info.front().qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  display->subProp("Depth Topic")->subProp("Reliability")->setValue("Reliable");
  display->subProp("Depth Topic")->subProp("Durability")->setValue("Transient Local");
  ASSERT_TRUE(waitUntil([&] {
    const auto endpoints = node->get_subscriptions_info_by_topic("/depth_view_test/arbitrary");
    return endpoints.size() == 1 && endpoints.front().qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable &&
      endpoints.front().qos_profile().durability() == rclcpp::DurabilityPolicy::TransientLocal;
  }));
  display->subProp("Depth Topic")->subProp("Reliability")->setValue("Best Effort");
  display->subProp("Depth Topic")->subProp("Durability")->setValue("Volatile");
  auto compressed = publisher->create_publisher<sensor_msgs::msg::CompressedImage>("/depth_view_test/no_transport_suffix", rclcpp::SensorDataQoS());
  display->subProp("Depth Topic")->setValue("/depth_view_test/no_transport_suffix");
  ASSERT_TRUE(waitUntil([&] { return compressed->get_subscription_count() > 0 && depth->get_subscription_count() == 0; }));
  ASSERT_TRUE(waitUntil([&] {
    calibration->publish(info);
    compressed->publish(*compressed_depth_image_transport::encodeCompressedDepthImage(*raw, "png"));
    return status("Depth").contains("Showing 1 points");
  }));
  // Render a denser surface for visual acceptance; the processor tests cover numeric colors.
  display->subProp("Max Depth (m)")->setValue(0);
  info.width = 64; info.height = 48;
  info.k = {50, 0, 31.5, 0, 50, 23.5, 0, 0, 1};
  info.p = {50, 0, 31.5, 0, 0, 50, 23.5, 0, 0, 0, 1, 0};
  auto surface = cv_bridge::CvImage(info.header, "32FC1", cv::Mat(48, 64, CV_32FC1, cv::Scalar(2))).toImageMsg();
  auto packed_surface = compressed_depth_image_transport::encodeCompressedDepthImage(*surface, "png");
  ASSERT_TRUE(waitUntil([&] {
    calibration->publish(info); compressed->publish(*packed_surface);
    return status("Depth").contains("Showing 3072 points");
  }));
  EXPECT_EQ(cloud->getPoints().size(), 3072u);
  if (const char * path = std::getenv("DEPTH_VIEW_SCREENSHOT")) {
    const auto frame = manager.getFrameCount();
    manager.queueRender();
    ASSERT_TRUE(waitUntil([&] { return manager.getFrameCount() > frame; }));
    panel.getRenderWindow()->captureScreenShot(path);
    EXPECT_TRUE(QFileInfo(path).exists());
  }
  display->subProp("Depth Topic")->setValue("/depth_view_test/absent");
  ASSERT_TRUE(waitUntil([&] { return status("Depth Topic").contains("Waiting for topic type"); }));
  auto wrong = publisher->create_publisher<sensor_msgs::msg::CameraInfo>("/depth_view_test/wrong", rclcpp::SensorDataQoS());
  ASSERT_TRUE(waitUntil([&] { return node->get_topic_names_and_types().count("/depth_view_test/wrong"); }));
  display->subProp("Depth Topic")->setValue("/depth_view_test/wrong");
  ASSERT_TRUE(waitUntil([&] { return status("Depth Topic").contains("Unsupported"); }));
  auto ambiguous_raw = publisher->create_publisher<sensor_msgs::msg::Image>("/depth_view_test/ambiguous", rclcpp::SensorDataQoS());
  auto alternate_context = std::make_shared<rclcpp::Context>();
  alternate_context->init(0, nullptr);
  auto alternate_node = std::make_shared<rclcpp::Node>("depth_view_alternate_source", rclcpp::NodeOptions().context(alternate_context));
  auto ambiguous_packed = alternate_node->create_publisher<sensor_msgs::msg::CompressedImage>("/depth_view_test/ambiguous", rclcpp::SensorDataQoS());
  ASSERT_TRUE(waitUntil([&] { const auto topics = node->get_topic_names_and_types(); const auto it = topics.find("/depth_view_test/ambiguous"); return it != topics.end() && it->second.size() == 2; }));
  display->subProp("Depth Topic")->setValue("/depth_view_test/ambiguous");
  ASSERT_TRUE(waitUntil([&] { return status("Depth Topic").contains("multiple message types"); }));
  display->setEnabled(false);
  ASSERT_TRUE(waitUntil([&] { return compressed->get_subscription_count() == 0 && calibration->get_subscription_count() == 0; }));
  manager.stopUpdate(); manager.removeAllDisplays();
}
