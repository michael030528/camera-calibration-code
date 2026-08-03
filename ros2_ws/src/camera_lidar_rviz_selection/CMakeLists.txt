cmake_minimum_required(VERSION 3.8)
project(camera_lidar_rviz_selection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/chessboard_select_tool.cpp)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  sensor_msgs
  std_msgs)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

ament_package()
