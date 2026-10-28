cmake_minimum_required(VERSION 3.8)
project(arc_rviz_plugins)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(cv_bridge REQUIRED)
find_package(compressed_depth_image_transport REQUIRED)
# Jazzy 4.0.x installs codec headers/libraries without CMake target exports.
find_path(COMPRESSED_DEPTH_INCLUDE_DIR compressed_depth_image_transport/codec.hpp
  HINTS "${compressed_depth_image_transport_DIR}/../../../include" REQUIRED)
find_library(COMPRESSED_DEPTH_LIBRARY compressed_depth_image_transport
  HINTS "${compressed_depth_image_transport_DIR}/../../../lib" REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(sensor_msgs REQUIRED)

get_filename_component(OGRE_INCLUDE_PARENT_DIR "${OGRE_INCLUDE_DIRS}" DIRECTORY)

qt5_wrap_cpp(display_moc_files
  src/camera_info_display.h
  src/depth_view_display.h
  src/tf_trajectory_display.h
)

add_library(${PROJECT_NAME}_display_plugins SHARED
  src/camera_info_display.cpp
  src/depth_view_display.cpp
  src/tf_trajectory_display.cpp
  ${display_moc_files}
)
target_include_directories(${PROJECT_NAME}_display_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_PARENT_DIR}
)
target_link_libraries(${PROJECT_NAME}_display_plugins PUBLIC
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
)
ament_target_dependencies(${PROJECT_NAME}_display_plugins PUBLIC
  cv_bridge
  geometry_msgs
  image_geometry
  pluginlib
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  sensor_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}_display_plugins
  EXPORT ${PROJECT_NAME}_display_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY src/ DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)
ament_export_targets(${PROJECT_NAME}_display_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(cv_bridge geometry_msgs image_geometry pluginlib rclcpp rviz_common rviz_ogre_vendor rviz_rendering sensor_msgs Qt5)
add_library(depth_view_processor STATIC src/depth_view_processor.cpp)
set_target_properties(depth_view_processor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(depth_view_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OpenCV_INCLUDE_DIRS} ${COMPRESSED_DEPTH_INCLUDE_DIR})
ament_target_dependencies(depth_view_processor sensor_msgs cv_bridge image_geometry compressed_depth_image_transport)
target_link_libraries(depth_view_processor ${OpenCV_LIBS})

target_link_libraries(${PROJECT_NAME}_display_plugins PRIVATE depth_view_processor)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(depth_view_benchmark test/depth_view_benchmark.cpp)
  target_link_libraries(depth_view_benchmark depth_view_processor)
  ament_add_gtest(depth_view_processor_test test/depth_view_processor_test.cpp)
  target_link_libraries(depth_view_processor_test depth_view_processor ${COMPRESSED_DEPTH_LIBRARY})
  ament_add_gtest(depth_view_integration_test test/depth_view_integration_test.cpp TIMEOUT 90)
  target_include_directories(depth_view_integration_test PRIVATE ${COMPRESSED_DEPTH_INCLUDE_DIR} ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_PARENT_DIR})
  ament_target_dependencies(depth_view_integration_test rclcpp rviz_common rviz_rendering rviz_ogre_vendor)
  target_link_libraries(depth_view_integration_test rviz_common::rviz_common rviz_rendering::rviz_rendering
    Qt5::Widgets rviz_ogre_vendor::OgreMain depth_view_processor ${COMPRESSED_DEPTH_LIBRARY})
endif()
ament_package()
