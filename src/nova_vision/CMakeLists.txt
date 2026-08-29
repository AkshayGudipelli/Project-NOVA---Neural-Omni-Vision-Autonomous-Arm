cmake_minimum_required(VERSION 3.8)
project(nova_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# macOS Dynamic Library RPATH Setup for Pixi
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib")

# Find ROS2 & OpenCV dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(mujoco_sim REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find MuJoCo library from environment
find_package(mujoco QUIET)
if(NOT mujoco_FOUND)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
    PATHS
      $ENV{CONDA_PREFIX}/include
      /usr/local/include
      /opt/homebrew/include
  )
  find_library(MUJOCO_LIBRARY NAMES mujoco
    PATHS
      $ENV{CONDA_PREFIX}/lib
      /usr/local/lib
      /opt/homebrew/lib
  )
endif()

# Build C++ Camera Node
add_executable(camera_node src/camera_node.cpp)
add_executable(object_detector_node src/object_detector_node.cpp)

target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(object_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(mujoco_FOUND)
  target_link_libraries(camera_node mujoco::mujoco)
else()
  target_include_directories(camera_node PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(camera_node ${MUJOCO_LIBRARY})
endif()

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  mujoco_sim
)

ament_target_dependencies(object_detector_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  camera_node
  object_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
