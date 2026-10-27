// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_common/properties/parse_color.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <image_transport/image_transport.hpp>
#include "camera_info_display.h"
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreBlendMode.h>
#include <QImage>
#include <OGRE/OgreHardwarePixelBuffer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <algorithm>
#include <cmath>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

namespace enc = sensor_msgs::image_encodings;

namespace arc_rviz_plugins
{
  ImageTopicProperty::ImageTopicProperty(
    const QString& name,
    const QString& default_value,
    const QString& description,
    rviz_common::properties::Property* parent,
    const char* changed_slot)
    : rviz_common::properties::EditableEnumProperty(
        name, default_value, description, parent, changed_slot)
  {
    QObject::connect(
      this,
      &rviz_common::properties::EditableEnumProperty::requestOptions,
      this,
      [this](rviz_common::properties::EditableEnumProperty*) {
        fillTopicList();
      });
  }

  void ImageTopicProperty::initialize(
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node)
  {
    rviz_ros_node_ = rviz_ros_node;
  }

  std::string ImageTopicProperty::getTopicStd() const
  {
    return getStdString();
  }

  void ImageTopicProperty::fillTopicList()
  {
    clearOptions();
    auto node_interface = rviz_ros_node_.lock();
    if (!node_interface) {
      return;
    }

    const auto node = node_interface->get_raw_node();
    const auto topic_names_and_types = node->get_topic_names_and_types();
    std::set<std::string> image_topics;
    for (const auto& topic_and_types : topic_names_and_types) {
      for (const auto& type : topic_and_types.second) {
        if (type == rosidl_generator_traits::name<sensor_msgs::msg::Image>() ||
            type == rosidl_generator_traits::name<sensor_msgs::msg::CompressedImage>()) {
          image_topics.insert(topic_and_types.first);
        }
      }
    }

    for (const auto& topic : image_topics) {
      addOptionStd(topic);
    }
  }

  TrianglePolygon::TrianglePolygon(
    Ogre::SceneManager* manager,
    Ogre::SceneNode* node,
    const cv::Point3d& O,
    const cv::Point3d& A,
    const cv::Point3d& B,
    const std::string& name,
    const Ogre::ColourValue& color,
    bool use_color,
    bool upper_triangle)
  {
    // uniq string is requred for name

    manual_ = manager->createManualObject();
    manual_->clear();
    manual_->begin(name,
                   Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    manual_->position(O.x, O.y, O.z);
    if (upper_triangle) {
      manual_->textureCoord(0, 0);
    }
    else {
      manual_->textureCoord(1, 0);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->position(A.x, A.y, A.z);
    if (upper_triangle) {
      manual_->textureCoord(1, 0);
    }
    else {
      manual_->textureCoord(1, 1);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->position(B.x, B.y, B.z);
    if (upper_triangle) {
      manual_->textureCoord(0, 1);
    }
    else {
      manual_->textureCoord(0, 1);
    }
    if (use_color) {
      manual_->colour(color);
    }
    manual_->end();
    node->attachObject(manual_);
  }

  TrianglePolygon::~TrianglePolygon()
  {
    manual_->detachFromParent();
    //manager_->destroyManualObject(manual_); // this crashes rviz
  }

  CameraInfoDisplay::CameraInfoDisplay()
    : alpha_(0.5),
      far_clip_distance_(1.0),
      show_polygons_(true),
      show_edges_(true),
      use_image_(false),
      image_updated_(true),
      not_show_side_polygons_(true),
      depth_range_initialized_(false),
      depth_min_(0.0),
      depth_max_(0.0)
  {
    ////////////////////////////////////////////////////////
    // initialize properties
    ////////////////////////////////////////////////////////
    far_clip_distance_property_ = new rviz_common::properties::FloatProperty(
      "far clip",
      1.0,
      "far clip distance from the origin of camera info",
      this, SLOT(updateFarClipDistance()));
    show_edges_property_ = new rviz_common::properties::BoolProperty(
      "show edges",
      true,
      "show edges of the region of the camera info",
      this, SLOT(updateShowEdges()));
    show_polygons_property_ = new rviz_common::properties::BoolProperty(
      "show polygons",
      true,
      "show polygons of the region of the camera info",
      this, SLOT(updateShowPolygons()));
    not_show_side_polygons_property_ = new rviz_common::properties::BoolProperty(
      "not show side polygons",
      true,
      "do not show polygons of the region of the camera info",
      this, SLOT(updateNotShowSidePolygons()));
    use_image_property_ = new rviz_common::properties::BoolProperty(
      "use image",
      false,
      "use image as texture",
      this, SLOT(updateUseImage()));
    image_topic_property_ = new ImageTopicProperty(
      "Image Topic", "",
      "sensor_msgs::msg::Image or sensor_msgs::msg::CompressedImage topic to subscribe to.",
      this, SLOT( updateImageTopic() ));
    image_topic_property_->hide();
    image_transport_hints_property_ = new ImageTransportHintsProperty(
      "transport hints",
      "transport hint for image subscription",
      this, SLOT( updateImageTopic() ));
    image_transport_hints_property_->hide();

    color_property_ = new rviz_common::properties::ColorProperty(
      "color",
      QColor(85, 255, 255),
      "color of CameraInfo",
      this, SLOT(updateColor()));
    edge_color_property_ = new rviz_common::properties::ColorProperty(
      "edge color",
      QColor(125, 125, 125),
      "edge color of CameraInfo",
      this, SLOT(updateEdgeColor()));
    alpha_property_ = new rviz_common::properties::FloatProperty(
      "alpha",
      0.5,
      "alpha blending value",
      this, SLOT(updateAlpha()));
  }

  CameraInfoDisplay::~CameraInfoDisplay()
  {
    if (edges_) {
      edges_->clear();
    }
    polygons_.clear();
    delete far_clip_distance_property_;
    delete color_property_;
    delete alpha_property_;
    delete show_polygons_property_;
    delete edge_color_property_;
  }

  void CameraInfoDisplay::reset()
  {
    RTDClass::reset();
    {
      std::lock_guard<std::mutex> lock(frequency_mutex_);
      reception_times_.clear();
    }
    if (edges_) {
      edges_->clear();
    }
    polygons_.clear();
    camera_info_.reset();
  }

  void CameraInfoDisplay::onInitialize()
  {
    RTDClass::onInitialize();
    image_topic_property_->initialize(context_->getRosNodeAbstraction());
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    updateColor();
    updateAlpha();
    updateFarClipDistance();
    updateShowPolygons();
    updateNotShowSidePolygons();
    updateShowEdges();
    updateUseImage();
    updateEdgeColor();
  }

  void CameraInfoDisplay::processMessage(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(frequency_mutex_);
      reception_times_.push_back(std::chrono::steady_clock::now());
    }
    updateTopicFrequency();
    if (!isSameCameraInfo(msg)) {
      createCameraInfoShapes(msg);
    }
    // move scene_node according to tf
     Ogre::Vector3 position;
     Ogre::Quaternion quaternion;
     std::string frame_id = msg->header.frame_id;
     if (frame_id.empty()) {
       setStatus(rviz_common::properties::StatusProperty::Error,
                 "Transform",
                 "Camera info frame_id is empty");
       return;
     }
     if (frame_id[0] == '/') {
       frame_id = frame_id.substr(1, frame_id.size());
     }
     if(!context_->getFrameManager()->getTransform(frame_id,
                                                   rclcpp::Time(msg->header.stamp, RCL_ROS_TIME),
                                                   position,
                                                   quaternion) &&
        !context_->getFrameManager()->getTransform(frame_id,
                                                   position,
                                                   quaternion)) {
       setStatus(rviz_common::properties::StatusProperty::Error,
                 "Transform",
                 QString("Error transforming camera info from frame '%1' to frame '%2'")
                   .arg(QString::fromStdString(msg->header.frame_id))
                   .arg(fixed_frame_));
       return;
     }
     setStatus(rviz_common::properties::StatusProperty::Ok, "Transform", "OK");
     scene_node_->setPosition(position);
     scene_node_->setOrientation(quaternion);
     camera_info_ = msg;        // store for caching
  }

  void CameraInfoDisplay::updateTopicFrequency()
  {
    const auto cutoff = std::chrono::steady_clock::now() - std::chrono::seconds(5);
    std::lock_guard<std::mutex> lock(frequency_mutex_);
    while (!reception_times_.empty() && reception_times_.front() <= cutoff) {
      reception_times_.pop_front();
    }
    // Fixed five-second window, including silence and the initial warm-up period.
    const double frequency = static_cast<double>(reception_times_.size()) / 5.0;
    setStatus(rviz_common::properties::StatusProperty::Ok, "Topic",
              QString("%1 messages in last 5 s at %2 hz.")
                .arg(static_cast<qulonglong>(reception_times_.size()))
                .arg(frequency, 0, 'f', 1));
  }

  void CameraInfoDisplay::update(float wall_dt, float ros_dt)
  {
    if (subscription_) {
      updateTopicFrequency();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (image_updated_) {
      if (bottom_texture_) {
        drawImageTexture();
        image_updated_ = false;
      }
    }
  }

  bool CameraInfoDisplay::isSameCameraInfo(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    if (camera_info_) {
      bool meta_same_p =
        msg->header.frame_id == camera_info_->header.frame_id &&
        msg->height == camera_info_->height &&
        msg->width == camera_info_->width &&
        msg->distortion_model == camera_info_->distortion_model &&
        msg->roi.x_offset == camera_info_->roi.x_offset &&
        msg->roi.y_offset == camera_info_->roi.y_offset &&
        msg->roi.height == camera_info_->roi.height &&
        msg->roi.width == camera_info_->roi.width;
      if (meta_same_p) {
        for (size_t i = 0; i < msg->p.size(); i++) {
          if (msg->p[i] != camera_info_->p[i]) {
            return false;
          }
        }
        return true;
      }
      else {
        return false;
      }
    }
    else {
      return false;
    }
  }

  void CameraInfoDisplay::addPointToEdge(
    const cv::Point3d& point)
  {
    Ogre::Vector3 p;
    p[0] = point.x;
    p[1] = point.y;
    p[2] = point.z;
    edges_->addPoint(p);
  }

  void CameraInfoDisplay::addPolygon(
    const cv::Point3d& O, const cv::Point3d& A, const cv::Point3d& B, std::string name, bool use_color, bool upper_triangle)
  {
    Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
    color.a = alpha_;
    TrianglePolygon::Ptr triangle (new TrianglePolygon(
                                     scene_manager_,
                                     scene_node_,
                                     O, A, B, name,
                                     color,
                                     use_color,
                                     upper_triangle));
    polygons_.push_back(triangle);
  }

  void CameraInfoDisplay::createTextureForBottom(int width, int height)
  {
    if (!bottom_texture_
        || bottom_texture_->getWidth() != width
        || bottom_texture_->getHeight() != height) {
      static uint32_t count = 0;
      rviz_common::UniformStringStream ss;
      ss << "CameraInfoDisplayPolygonBottom" << count++;
      material_bottom_
        = Ogre::MaterialManager::getSingleton().create(
          ss.str(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      bottom_texture_ = Ogre::TextureManager::getSingleton().createManual(
        material_bottom_->getName() + "Texture",        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);
      material_bottom_->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
      Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
      color.a = alpha_;
      material_bottom_->getTechnique(0)->getPass(0)->setAmbient(color);
      material_bottom_->setReceiveShadows(false);
      material_bottom_->getTechnique(0)->setLightingEnabled(true);
      material_bottom_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
      material_bottom_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      material_bottom_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
      material_bottom_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);

      material_bottom_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      material_bottom_->getTechnique(0)->getPass(0)->createTextureUnitState(bottom_texture_->getName());
      material_bottom_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    }
  }

  void CameraInfoDisplay::prepareMaterial()
  {
    if (!texture_) {
      // material
      static uint32_t count = 0;
      rviz_common::UniformStringStream ss;
      ss << "CameraInfoDisplayPolygon" << count++;
      material_
        = Ogre::MaterialManager::getSingleton().create(
          ss.str(),
          Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
      texture_ = Ogre::TextureManager::getSingleton().createManual(
        material_->getName() + "Texture",        // name
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D, 1, 1, 0, Ogre::PF_A8R8G8B8, Ogre::TU_DEFAULT);
      material_->getTechnique(0)->getPass(0)->setColourWriteEnabled(true);
      Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
      color.a = alpha_;
      material_->getTechnique(0)->getPass(0)->setAmbient(color);
      material_->setReceiveShadows(false);
      material_->getTechnique(0)->setLightingEnabled(true);
      material_->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
      material_->getTechnique(0)->getPass(0)->setLightingEnabled(false);
      material_->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
      material_->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);

      material_->getTechnique(0)->getPass(0)->setVertexColourTracking(Ogre::TVC_DIFFUSE);
      material_->getTechnique(0)->getPass(0)->createTextureUnitState(texture_->getName());
      material_->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
      createTextureForBottom(640, 480);
    }
  }

  void CameraInfoDisplay::subscribeImage(std::string topic)
  {

    image_sub_.shutdown();
    compressed_image_sub_.reset();
    if (topic.empty()) {
      setStatus(rviz_common::properties::StatusProperty::Warn, "Image Topic", "Topic name is empty");
      return;
    }
    auto node_interface = rviz_ros_node_.lock();
    if (!node_interface) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Image Topic", "RViz ROS node is unavailable");
      return;
    }
    auto node = node_interface->get_raw_node();
    depth_range_initialized_ = false;
    if (topic.rfind("/compressed") == topic.size() - std::string("/compressed").size() ||
        topic.rfind("/compressedDepth") == topic.size() - std::string("/compressedDepth").size()) {
      compressed_image_sub_ = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&CameraInfoDisplay::compressedImageCallback, this, std::placeholders::_1));
      setStatus(rviz_common::properties::StatusProperty::Ok, "Image Topic", "OK");
      return;
    }
    image_sub_ = image_transport::create_subscription(
      node.get(),
      topic,
      std::bind(&CameraInfoDisplay::imageCallback, this, std::placeholders::_1),
      image_transport_hints_property_->getTransport(),
      rmw_qos_profile_sensor_data);
    setStatus(rviz_common::properties::StatusProperty::Ok, "Image Topic", "OK");
  }

  void CameraInfoDisplay::drawImageTexture()
  {
    bottom_texture_->getBuffer()->lock( Ogre::HardwareBuffer::HBL_NORMAL );
    const Ogre::PixelBox& pixelBox
      = bottom_texture_->getBuffer()->getCurrentLock();
    Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);

    // Don't copy pixel-by-pixel image matrices.
    // Just split matrix into channels, add needed alpha channel and merge back directly into buffer.
    if (use_image_ && !image_.empty() &&
        bottom_texture_->getHeight() == image_.rows &&
        bottom_texture_->getWidth() == image_.cols) {
      std::vector<cv::Mat> splitted;
      cv::split(image_, splitted);
      // Swap channels RGB -> BGR for cv::merge.
      std::swap(splitted[0], splitted[2]);
      cv::Mat alpha(image_.rows, image_.cols, CV_8U, cv::Scalar(alpha_ * 255.0));
      splitted.push_back(alpha);
      cv::Mat boxMat(image_.rows, image_.cols, CV_8UC4, pDest);
      cv::merge(splitted, boxMat);
    } else {
      memset(pDest, 0, bottom_texture_->getWidth() * bottom_texture_->getHeight());
      QImage Hud(pDest, bottom_texture_->getWidth(), bottom_texture_->getHeight(), QImage::Format_ARGB32);
      for (size_t j = 0; j < bottom_texture_->getHeight(); j++) {
        for (size_t i = 0; i < bottom_texture_->getWidth(); i++) {
          Hud.setPixel(i, j, color_.rgba());
        }
      }
    }
    bottom_texture_->getBuffer()->unlock();
  }

  bool CameraInfoDisplay::convertImageToRGB(
    const cv::Mat& input, const std::string& encoding, cv::Mat& output)
  {
    if (input.empty()) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Image", "Decoded image is empty");
      return false;
    }

    cv::Mat im = input;
    const bool has_depth_encoding =
      encoding == enc::TYPE_16UC1 || encoding == enc::TYPE_32FC1 ||
      encoding == "16UC1" || encoding == "32FC1";

    if (encoding == enc::BGRA8) {
      cv::cvtColor(im, output, cv::COLOR_BGRA2RGB);
    } else if (encoding == enc::BGRA16) {
      im.convertTo(output, CV_8U, 1 / 256.0);
      cv::cvtColor(output, output, cv::COLOR_BGRA2RGB);
    } else if (encoding == enc::BGR8) {
      cv::cvtColor(im, output, cv::COLOR_BGR2RGB);
    } else if (encoding == enc::BGR16) {
      im.convertTo(output, CV_8U, 1 / 256.0);
      cv::cvtColor(output, output, cv::COLOR_BGR2RGB);
    } else if (encoding == enc::RGBA8) {
      cv::cvtColor(im, output, cv::COLOR_RGBA2RGB);
    } else if (encoding == enc::RGBA16) {
      im.convertTo(output, CV_8U, 1 / 256.0);
      cv::cvtColor(output, output, cv::COLOR_RGBA2RGB);
    } else if (encoding == enc::RGB8) {
      output = im.clone();
    } else if (encoding == enc::RGB16) {
      im.convertTo(output, CV_8U, 1 / 256.0);
    } else if (encoding == enc::MONO8) {
      cv::cvtColor(im, output, cv::COLOR_GRAY2RGB);
    } else if (encoding == enc::MONO16) {
      im.convertTo(output, CV_8U, 1 / 256.0);
      cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);
    } else if (has_depth_encoding || im.channels() == 1) {
      cv::Mat depth_float;
      im.convertTo(depth_float, CV_32F);
      cv::Mat finite_mask;
      if (depth_float.depth() == CV_32F || depth_float.depth() == CV_64F) {
        finite_mask = depth_float == depth_float;
      } else {
        finite_mask = cv::Mat(depth_float.rows, depth_float.cols, CV_8U, cv::Scalar(255));
      }
      cv::Mat nonzero_mask = depth_float != 0;
      cv::Mat valid_mask;
      cv::bitwise_and(finite_mask, nonzero_mask, valid_mask);
      double min_value = 0.0;
      double max_value = 0.0;
      std::vector<float> valid_depths;
      valid_depths.reserve(cv::countNonZero(valid_mask));
      for (int y = 0; y < im.rows; ++y) {
        for (int x = 0; x < im.cols; ++x) {
          if (!valid_mask.at<uint8_t>(y, x)) {
            continue;
          }
          valid_depths.push_back(depth_float.at<float>(y, x));
        }
      }
      if (valid_depths.size() >= 20) {
        const size_t low_index = valid_depths.size() * 2 / 100;
        const size_t high_index = valid_depths.size() * 98 / 100;
        std::nth_element(valid_depths.begin(), valid_depths.begin() + low_index, valid_depths.end());
        min_value = valid_depths[low_index];
        std::nth_element(valid_depths.begin(), valid_depths.begin() + high_index, valid_depths.end());
        max_value = valid_depths[high_index];
      } else {
        cv::minMaxLoc(depth_float, &min_value, &max_value, nullptr, nullptr, valid_mask);
      }
      if (max_value > min_value) {
        if (!depth_range_initialized_) {
          depth_min_ = min_value;
          depth_max_ = max_value;
          depth_range_initialized_ = true;
        } else {
          const double smoothing = 0.9;
          depth_min_ = depth_min_ * smoothing + min_value * (1.0 - smoothing);
          depth_max_ = depth_max_ * smoothing + max_value * (1.0 - smoothing);
        }
        min_value = depth_min_;
        max_value = depth_max_;
      }
      cv::Mat normalized_depth;
      if (max_value > min_value) {
        depth_float.convertTo(normalized_depth, CV_8U, -255.0 / (max_value - min_value),
                              max_value * 255.0 / (max_value - min_value));
      } else {
        normalized_depth = cv::Mat(im.rows, im.cols, CV_8U, cv::Scalar(0));
      }
      normalized_depth.setTo(0, ~valid_mask);
      cv::Mat colorized_depth;
      cv::applyColorMap(normalized_depth, colorized_depth, cv::COLORMAP_TURBO);
      colorized_depth.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
      cv::cvtColor(colorized_depth, output, cv::COLOR_BGR2RGB);
    } else if (im.channels() == 3) {
      cv::cvtColor(im, output, cv::COLOR_BGR2RGB);
    } else if (im.channels() == 4) {
      cv::cvtColor(im, output, cv::COLOR_BGRA2RGB);
    } else {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Image", QString("Unsupported image encoding '%1'")
                  .arg(QString::fromStdString(encoding)));
      return false;
    }
    return true;
  }

  bool CameraInfoDisplay::decodeCompressedImage(
    sensor_msgs::msg::CompressedImage::ConstSharedPtr msg, cv::Mat& output)
  {
    std::vector<uint8_t> data = msg->data;
    if (msg->format.find("compressedDepth") != std::string::npos) {
      const std::vector<uint8_t> png_signature = {0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
      auto png_begin = std::search(data.begin(), data.end(), png_signature.begin(), png_signature.end());
      if (png_begin == data.end()) {
        setStatus(rviz_common::properties::StatusProperty::Error,
                  "Image", "Failed to find PNG payload in compressedDepth image");
        return false;
      }
      data.erase(data.begin(), png_begin);
    }

    cv::Mat encoded(1, static_cast<int>(data.size()), CV_8UC1, data.data());
    cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_UNCHANGED);
    if (decoded.empty()) {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Image", "Failed to decode compressed image");
      return false;
    }

    std::string encoding = msg->format;
    const size_t separator = encoding.find(';');
    if (separator != std::string::npos) {
      encoding = encoding.substr(0, separator);
    }
    const size_t compressed_pixel_format = msg->format.find("compressed ");
    if (compressed_pixel_format != std::string::npos &&
        msg->format.find("compressedDepth") == std::string::npos) {
      encoding = msg->format.substr(compressed_pixel_format + std::string("compressed ").size());
    }
    if (encoding.empty() || encoding.find("jpeg") != std::string::npos ||
        encoding.find("png") != std::string::npos) {
      encoding = decoded.channels() == 1 ? enc::MONO8 : enc::BGR8;
    }
    return convertImageToRGB(decoded, encoding, output);
  }

  void CameraInfoDisplay::updateImage(const cv::Mat& image)
  {
    int roi_height = camera_info_->roi.height ? camera_info_->roi.height : camera_info_->height;
    int roi_width = camera_info_->roi.width ? camera_info_->roi.width : camera_info_->width;
    if (camera_info_->binning_y > 0) {
      roi_height /= camera_info_->binning_y;
    }
    if (camera_info_->binning_x > 0) {
      roi_width /= camera_info_->binning_x;
    }

    if (image.cols == static_cast<int>(camera_info_->width) &&
        image.rows == static_cast<int>(camera_info_->height)) {
      cv::Rect roi(camera_info_->roi.x_offset, camera_info_->roi.y_offset,
                   camera_info_->roi.width ? camera_info_->roi.width : camera_info_->width,
                   camera_info_->roi.height ? camera_info_->roi.height : camera_info_->height);
      image_ = cv::Mat(image, roi).clone();
    } else if (image.cols == roi_width && image.rows == roi_height) {
      image_ = image.clone();
    } else {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Image", QString("Invalid image size (%1, %2), expected (%3, %4) or ROI size (%5, %6)")
                  .arg(image.cols).arg(image.rows)
                  .arg(camera_info_->width).arg(camera_info_->height)
                  .arg(roi_width).arg(roi_height));
      return;
    }

    if (!bottom_texture_
        || bottom_texture_->getWidth() != image_.cols
        || bottom_texture_->getHeight() != image_.rows) {
      createTextureForBottom(image_.cols, image_.rows);
      if (camera_info_) {
        createCameraInfoShapes(camera_info_);
      }
    }
    image_updated_ = true;
  }

  // convert sensor_msgs::Image into cv::Mat
  void CameraInfoDisplay::imageCallback(
      sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cv_bridge::CvImageConstPtr cv_ptr;
    if (!camera_info_) {
      return;
    }
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg);
      cv::Mat rgb_image;
      if (!convertImageToRGB(cv_ptr->image, msg->encoding, rgb_image)) {
        return;
      }
      updateImage(rgb_image);
    }
    catch (cv_bridge::Exception& e)
    {
      setStatus(rviz_common::properties::StatusProperty::Error,
                "Image", QString("cv_bridge exception: ") + e.what());
    }
  }

  void CameraInfoDisplay::compressedImageCallback(
      sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!camera_info_) {
      return;
    }
    cv::Mat rgb_image;
    if (decodeCompressedImage(msg, rgb_image)) {
      updateImage(rgb_image);
    }
  }

  void CameraInfoDisplay::createCameraInfoShapes(
    sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    polygons_.clear();
    if (edges_) {
      edges_->clear();
    }
    image_geometry::PinholeCameraModel model;
    bool model_success_p = model.fromCameraInfo(msg);
    if (!model_success_p) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", "Failed to create camera model from msg");
      return;
    }
    // fx and fy should not be equal 0.
    if (model.fx() == 0.0 || model.fy() == 0.0) {
      setStatus(rviz_common::properties::StatusProperty::Error, "Camera Info", "Invalid intrinsic matrix");
      return;
    }
    setStatus(rviz_common::properties::StatusProperty::Ok, "Camera Info", "OK");

    ////////////////////////////////////////////////////////
    // initialize BillboardLine
    ////////////////////////////////////////////////////////
    if (!edges_) {
      edges_.reset(new rviz_rendering::BillboardLine(context_->getSceneManager(),
                                           scene_node_));
      edges_->setLineWidth(0.01);
    }

    int height = msg->roi.height ? msg->roi.height : msg->height;
    int width = msg->roi.width ? msg->roi.width : msg->width;
    if (msg->binning_y > 0) {
      height /= msg->binning_y;
    }
    if (msg->binning_x > 0) {
      width /= msg->binning_x;
    }

    cv::Point2d a(0, 0), b(width, 0),
      c(width, height), d(0, height);
    // all the z = 1.0
    cv::Point3d A = model.projectPixelTo3dRay(a);
    cv::Point3d B = model.projectPixelTo3dRay(b);
    cv::Point3d C = model.projectPixelTo3dRay(c);
    cv::Point3d D = model.projectPixelTo3dRay(d);

    cv::Point3d scaled_A = A * far_clip_distance_;
    cv::Point3d scaled_B = B * far_clip_distance_;
    cv::Point3d scaled_C = C * far_clip_distance_;
    cv::Point3d scaled_D = D * far_clip_distance_;

    cv::Point3d O(0, 0, 0);

    ////////////////////////////////////////////////////////
    // build polygons
    ////////////////////////////////////////////////////////
    if (show_polygons_) {
      ////////////////////////////////////////////////////////
      // setup color for polygons
      ////////////////////////////////////////////////////////
      Ogre::ColourValue color = rviz_common::properties::qtToOgre(color_);
      color.a = alpha_;
      prepareMaterial();
      if (!not_show_side_polygons_) {
        material_->getTechnique(0)->getPass(0)->setAmbient(color);
        {
          texture_->getBuffer()->lock( Ogre::HardwareBuffer::HBL_NORMAL );
          const Ogre::PixelBox& pixelBox
            = texture_->getBuffer()->getCurrentLock();
          Ogre::uint8* pDest = static_cast<Ogre::uint8*> (pixelBox.data);
          memset(pDest, 0, 1);
          QImage Hud(pDest, 1, 1, QImage::Format_ARGB32 );
          Hud.setPixel(0, 0, color_.rgba());
          texture_->getBuffer()->unlock();
        }
        addPolygon(O, scaled_B, scaled_A, material_->getName(), true, true);
        addPolygon(O, scaled_C, scaled_B, material_->getName(), true, true);
        addPolygon(O, scaled_D, scaled_C, material_->getName(), true, true);
        addPolygon(O, scaled_A, scaled_D, material_->getName(), true, true);
      }
      // bottom
      drawImageTexture();

      addPolygon(scaled_A, scaled_B, scaled_D, material_bottom_->getName(), false, true);
      addPolygon(scaled_B, scaled_C, scaled_D, material_bottom_->getName(), false, false);
    }
    ////////////////////////////////////////////////////////
    // build edges
    ////////////////////////////////////////////////////////
    if (show_edges_) {
      edges_->clear();
      edges_->setMaxPointsPerLine(2);
      edges_->setNumLines(8);
      edges_->setColor(edge_color_.red() / 255.0,
                       edge_color_.green() / 255.0,
                       edge_color_.blue() / 255.0,
                       alpha_);
      addPointToEdge(O); addPointToEdge(scaled_A); edges_->finishLine();
      addPointToEdge(O); addPointToEdge(scaled_B); edges_->finishLine();
      addPointToEdge(O); addPointToEdge(scaled_C); edges_->finishLine();
      addPointToEdge(O); addPointToEdge(scaled_D); edges_->finishLine();
      addPointToEdge(scaled_A); addPointToEdge(scaled_B); edges_->finishLine();
      addPointToEdge(scaled_B); addPointToEdge(scaled_C); edges_->finishLine();
      addPointToEdge(scaled_C); addPointToEdge(scaled_D); edges_->finishLine();
      addPointToEdge(scaled_D); addPointToEdge(scaled_A);
    }
  }

  ////////////////////////////////////////////////////////
  // Properties updating functions
  ////////////////////////////////////////////////////////
  void CameraInfoDisplay::updateColor()
  {
    color_ = color_property_->getColor();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateEdgeColor()
  {
    edge_color_ = edge_color_property_->getColor();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateAlpha()
  {
    alpha_ = alpha_property_->getFloat();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateFarClipDistance()
  {
    far_clip_distance_ = far_clip_distance_property_->getFloat();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateShowPolygons()
  {
    show_polygons_ = show_polygons_property_->getBool();
    if (show_polygons_) {
      not_show_side_polygons_property_->show();
    }
    else {
      not_show_side_polygons_property_->hide();
    }
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateShowEdges()
  {
    show_edges_ = show_edges_property_->getBool();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }

  void CameraInfoDisplay::updateImageTopic()
  {
    if (use_image_) {
      std::string topic = image_topic_property_->getTopicStd();
      subscribeImage(topic);
    } else {
      image_sub_.shutdown();
      compressed_image_sub_.reset();
      deleteStatus("Image Topic");
      // Set image_updated_ true in order to clear the bottom texture in update() method.
      image_updated_ = true;
    }
  }

  void CameraInfoDisplay::updateUseImage()
  {
    use_image_ = use_image_property_->getBool();
    if (use_image_) {
      image_topic_property_->show();
      image_transport_hints_property_->show();
    }
    else {
      image_topic_property_->hide();
      image_transport_hints_property_->hide();
    }
    updateImageTopic();
  }
  void CameraInfoDisplay::updateNotShowSidePolygons()
  {
    not_show_side_polygons_ = not_show_side_polygons_property_->getBool();
    if (camera_info_) {
      createCameraInfoShapes(camera_info_);
    }
  }
}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( arc_rviz_plugins::CameraInfoDisplay, rviz_common::Display )
