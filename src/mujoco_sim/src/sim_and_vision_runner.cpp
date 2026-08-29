#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mujoco_sim/sim_handle.hpp"

#define NO_MAIN
#include "../src/mujoco_sim_node.cpp"
#include "../../nova_vision/src/camera_node.cpp"
#undef NO_MAIN

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // 1. Construct MujocoSimNode (loads XML model and owns shared SimHandle)
  auto sim_node = std::make_shared<MujocoSimNode>();

  // 2. Construct C++ CameraNode and link the shared physics handle!
  auto camera_node = std::make_shared<CameraNode>();
  camera_node->setSimHandle(sim_node->getSimHandle());

  // 3. Spin both C++ nodes concurrently in single process
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(sim_node);
  executor.add_node(camera_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
