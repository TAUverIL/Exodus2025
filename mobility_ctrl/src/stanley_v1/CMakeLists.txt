cmake_minimum_required(VERSION 3.5)
project(stanley_v1)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include headers from the include/ folder
include_directories(include)

# Build the shared library from your source file
add_library(${PROJECT_NAME} SHARED
  src/stanley_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
)

# Install the shared library to the lib directory
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the header files so that they are available for other packages
install(DIRECTORY include/
  DESTINATION include
)

# Install the plugin description file for pluginlib
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(nav2_core plugin_description.xml)

ament_package()
