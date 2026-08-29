#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include <mujoco/mujoco.h>
#include "mujoco_sim/sim_handle.hpp"

using namespace std::chrono_literals;

class MujocoSimNode : public rclcpp::Node
{
public:
  MujocoSimNode()
  : Node("mujoco_sim_node"),
    sim_handle_(std::make_shared<SimHandle>()),
    sim_running_(true)
  {
    // Declare parameters for model path
    this->declare_parameter<std::string>("model_path", "");
    std::string model_path = this->get_parameter("model_path").as_string();

    if (model_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No model_path specified for MuJoCo simulation!");
      return;
    }

    // Load MuJoCo Model into shared SimHandle
    char error[1000] = "Could not load XML model";
    sim_handle_->model = mj_loadXML(model_path.c_str(), 0, error, 1000);
    if (!sim_handle_->model) {
      RCLCPP_ERROR(this->get_logger(), "MuJoCo Load Error: %s", error);
      return;
    }
    sim_handle_->data = mj_makeData(sim_handle_->model);

    RCLCPP_INFO(this->get_logger(), "MuJoCo model successfully loaded from: %s", model_path.c_str());
    RCLCPP_INFO(this->get_logger(), "Robot Joints: %d, Actuators: %d", sim_handle_->model->njnt, sim_handle_->model->nu);

    // Joint Names matching our URDF
    joint_names_ = {
      "joint1", "joint2", "joint3", "joint4", "joint5",
      "gripper_left_joint", "gripper_right_joint"
    };

    // Target command buffer (5 arm joints + 1 gripper command)
    target_ctrl_.resize(sim_handle_->model->nu, 0.0);

    // Set initial home posture: Arm upright, gripper open
    target_ctrl_[0] = 0.0;     // base_pan
    target_ctrl_[1] = 0.0;     // shoulder_pitch
    target_ctrl_[2] = 0.0;     // elbow_pitch
    target_ctrl_[3] = 0.0;     // wrist_pitch
    target_ctrl_[4] = 0.0;     // wrist_roll
    if (sim_handle_->model->nu >= 7) {
      target_ctrl_[5] = 0.02;  // gripper_left open
      target_ctrl_[6] = 0.02;  // gripper_right open
    }

    // ROS2 Subscribers (Hardware Abstraction Layer)
    joint_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/joint_commands", 10,
      std::bind(&MujocoSimNode::jointCommandCallback, this, std::placeholders::_1));

    gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "/gripper_command", 10,
      std::bind(&MujocoSimNode::gripperCommandCallback, this, std::placeholders::_1));

    // ROS2 Publishers
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Timers
    // 1. High-frequency physics stepping thread (500 Hz = 2ms)
    physics_timer_ = this->create_wall_timer(
      2ms, std::bind(&MujocoSimNode::stepPhysics, this));

    // 2. State publishing timer for ROS2 TF tree (50 Hz = 20ms)
    pub_timer_ = this->create_wall_timer(
      20ms, std::bind(&MujocoSimNode::publishJointStates, this));

    RCLCPP_INFO(this->get_logger(), "MuJoCo Simulation Bridge Node is ready and running!");
  }

  ~MujocoSimNode()
  {
    sim_running_ = false;
  }

  // Option 1 & 2 Public Getters for State Sharing
  SimHandlePtr getSimHandle() { return sim_handle_; }
  mjModel* getModel() { return sim_handle_->model; }
  mjData* getData() { return sim_handle_->data; }
  std::mutex& getMutex() { return sim_handle_->mutex; }

private:
  void jointCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sim_handle_->mutex);
    size_t num_arm_joints = std::min(static_cast<size_t>(5), msg->data.size());
    for (size_t i = 0; i < num_arm_joints; ++i) {
      target_ctrl_[i] = msg->data[i];
    }
  }

  void gripperCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(sim_handle_->mutex);
    double width = std::clamp(msg->data, 0.0, 0.03);
    if (sim_handle_->model->nu >= 7) {
      target_ctrl_[5] = width; // left finger
      target_ctrl_[6] = width; // right finger
    }
  }

  void stepPhysics()
  {
    if (!sim_handle_ || !sim_handle_->model || !sim_handle_->data) return;

    std::lock_guard<std::mutex> lock(sim_handle_->mutex);
    
    // Apply position actuator targets to MuJoCo control vector
    for (int i = 0; i < sim_handle_->model->nu; ++i) {
      sim_handle_->data->ctrl[i] = target_ctrl_[i];
    }

    // Step physical dynamics forward in time
    mj_step(sim_handle_->model, sim_handle_->data);
  }

  void publishJointStates()
  {
    if (!sim_handle_ || !sim_handle_->model || !sim_handle_->data) return;

    std::lock_guard<std::mutex> lock(sim_handle_->mutex);

    auto state_msg = sensor_msgs::msg::JointState();
    state_msg.header.stamp = this->now();
    state_msg.name = joint_names_;
    state_msg.position.resize(joint_names_.size());
    state_msg.velocity.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (static_cast<int>(i) < sim_handle_->model->nq) {
        state_msg.position[i] = sim_handle_->data->qpos[i];
        state_msg.velocity[i] = sim_handle_->data->qvel[i];
      }
    }

    joint_state_pub_->publish(state_msg);
  }

  // Shared Thread-Safe SimHandle
  SimHandlePtr sim_handle_;
  std::atomic<bool> sim_running_;

  std::vector<std::string> joint_names_;
  std::vector<double> target_ctrl_;

  // ROS2 Handles
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr physics_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

#ifndef NO_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MujocoSimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
