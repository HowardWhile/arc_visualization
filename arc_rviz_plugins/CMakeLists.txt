cmake_minimum_required(VERSION 3.8)
project(arc_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(cv_bridge REQUIRED)
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
  src/tf_trajectory_display.h
)

add_library(${PROJECT_NAME}_display_plugins SHARED
  src/camera_info_display.cpp
  src/tf_trajectory_display.cpp
  ${display_moc_files}
)
target_include_directories(${PROJECT_NAME}_display_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${OGRE_INCLUDE_DIRS}
  ${OGRE_INCLUDE_PARENT_DIR}
)
target_link_libraries(${PROJECT_NAME}_display_plugins
  Qt5::Widgets
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
)
ament_target_dependencies(${PROJECT_NAME}_display_plugins
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
ament_package()
