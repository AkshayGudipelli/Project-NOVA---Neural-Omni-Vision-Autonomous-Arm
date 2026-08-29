cmake_minimum_required(VERSION 3.8)
project(mujoco_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# macOS Dynamic Library RPATH Setup for Pixi
set(CMAKE_MACOSX_RPATH 1)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib")

# Find ROS2 & Vision dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Find MuJoCo C/C++ library from environment
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

# Build C++ MuJoCo simulation bridge node
add_executable(mujoco_sim_node src/mujoco_sim_node.cpp)

target_include_directories(mujoco_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(mujoco_FOUND)
  target_link_libraries(mujoco_sim_node mujoco::mujoco)
else()
  target_include_directories(mujoco_sim_node PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(mujoco_sim_node ${MUJOCO_LIBRARY})
endif()

ament_target_dependencies(mujoco_sim_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Build C++ sim_and_vision_runner executable
add_executable(sim_and_vision_runner src/sim_and_vision_runner.cpp)

target_include_directories(sim_and_vision_runner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(mujoco_FOUND)
  target_link_libraries(sim_and_vision_runner mujoco::mujoco ${OpenCV_LIBRARIES})
else()
  target_include_directories(sim_and_vision_runner PRIVATE ${MUJOCO_INCLUDE_DIR})
  target_link_libraries(sim_and_vision_runner ${MUJOCO_LIBRARY} ${OpenCV_LIBRARIES})
endif()

ament_target_dependencies(sim_and_vision_runner
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# Export headers for other packages (like nova_vision)
ament_export_include_directories(include)

# Install binaries, include headers, and models directory
install(TARGETS
  mujoco_sim_node
  sim_and_vision_runner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
