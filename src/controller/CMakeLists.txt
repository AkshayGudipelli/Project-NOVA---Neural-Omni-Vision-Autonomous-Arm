cmake_minimum_required(VERSION 3.8)
project(controller)

# Auto-fallback for macOS Sequoia/Ventura SDK issue in Conda/Pixi environment
if(APPLE AND EXISTS "/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk")
  set(CMAKE_OSX_SYSROOT "/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk" CACHE PATH "" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# macOS Dynamic Library RPATH Setup for Pixi
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib")

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Build Hardware-Ready Joint Controller Node
add_executable(joint_controller_node src/joint_controller_node.cpp)
ament_target_dependencies(joint_controller_node
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

# Install binaries
install(TARGETS
  joint_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
