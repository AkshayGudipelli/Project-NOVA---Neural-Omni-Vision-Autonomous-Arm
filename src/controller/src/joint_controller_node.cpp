// #include <chrono>
// #include <memory>
// #include <string>
// #include <vector>
// #include <cmath>
// #include <algorithm>

// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include "std_msgs/msg/float64.hpp"

// using namespace std::chrono_literals;

// /**
//  * @brief Hardware-Ready Joint Controller Node
//  * 
//  * Implements smooth trapezoidal/linear interpolation and strict physical
//  * joint limit enforcement for the 6-DOF DIY Hobby Servo Arm.
//  * Works seamlessly with both MuJoCo simulation and physical Arduino/ESP32 hardware.
//  */
// class JointControllerNode : public rclcpp::Node
// {
// public:
//   JointControllerNode()
//   : Node("joint_controller_node")
//   {
//     // Declare physical parameters
//     this->declare_parameter<double>("max_joint_velocity", 1.2); // rad/s (safe for MG996R servos)
//     max_velocity_ = this->get_parameter("max_joint_velocity").as_double();

//     // 5 arm joints
//     num_joints_ = 5;
//     joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5"};

//     // Physical joint limits in radians (-90 deg to +90 deg for 180-deg hobby servos)
//     min_limits_ = {-1.5708, -1.5708, -1.5708, -1.5708, -3.1415};
//     max_limits_ = { 1.5708,  1.5708,  1.5708,  1.5708,  3.1415};

//     // State buffers
//     current_angles_.assign(num_joints_, 0.0);
//     target_angles_.assign(num_joints_, 0.0);
//     interpolated_cmd_.assign(num_joints_, 0.0);

//     // Subscribers: Receives desired target pose from IK solver / Task Planner
//     target_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
//       "/target_joint_angles", 10,
//       std::bind(&JointControllerNode::targetCallback, this, std::placeholders::_1));

//     // Subscribes to live feedback from MuJoCo or physical servo encoders
//     joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//       "/joint_states", 10,
//       std::bind(&JointControllerNode::jointStateCallback, this, std::placeholders::_1));

//     // Publisher: Publishes smooth interpolated commands to MuJoCo / Arduino HAL
//     joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_commands", 10);

//     // Control loop timer: Runs at 50 Hz (20ms, standard PWM servo refresh period)
//     control_timer_ = this->create_wall_timer(
//       20ms, std::bind(&JointControllerNode::controlLoop, this));

//     RCLCPP_INFO(this->get_logger(), "Hardware-Ready Joint Controller Node Initialized at 50 Hz.");
//   }

// private:
//   void targetCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
//   {
//     if (msg->data.size() < num_joints_) {
//       RCLCPP_WARN(this->get_logger(), "Received target command with insufficient joints (%zu/%zu)",
//         msg->data.size(), num_joints_);
//       return;
//     }

//     // Apply strict physical limit clamping before accepting target
//     for (size_t i = 0; i < num_joints_; ++i) {
//       target_angles_[i] = std::clamp(msg->data[i], min_limits_[i], max_limits_[i]);
//     }

//     RCLCPP_INFO(this->get_logger(), "New target setpoint accepted: [%.2f, %.2f, %.2f, %.2f, %.2f]",
//       target_angles_[0], target_angles_[1], target_angles_[2], target_angles_[3], target_angles_[4]);
//   }

//   void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
//   {
//     for (size_t i = 0; i < num_joints_; ++i) {
//       auto it = std::find(msg->name.begin(), msg->name.end(), joint_names_[i]);
//       if (it != msg->name.end()) {
//         size_t idx = std::distance(msg->name.begin(), it);
//         if (idx < msg->position.size()) {
//           current_angles_[i] = msg->position[idx];
//         }
//       }
//     }
//   }

//   void controlLoop()
//   {
//     double dt = 0.02; // 20ms
//     double max_step = max_velocity_ * dt;

//     // Smooth velocity interpolation: Moves interpolated_cmd_ towards target_angles_
//     for (size_t i = 0; i < num_joints_; ++i) {
//       double error = target_angles_[i] - interpolated_cmd_[i];
//       if (std::abs(error) <= max_step) {
//         interpolated_cmd_[i] = target_angles_[i];
//       } else {
//         interpolated_cmd_[i] += std::copysign(max_step, error);
//       }
//     }

//     // Publish smooth command vector to the hardware abstraction topic
//     auto cmd_msg = std_msgs::msg::Float64MultiArray();
//     cmd_msg.data = interpolated_cmd_;
//     joint_cmd_pub_->publish(cmd_msg);
//   }

//   size_t num_joints_;
//   double max_velocity_;
//   std::vector<std::string> joint_names_;
//   std::vector<double> min_limits_;
//   std::vector<double> max_limits_;

//   std::vector<double> current_angles_;
//   std::vector<double> target_angles_;
//   std::vector<double> interpolated_cmd_;

//   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
//   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
//   rclcpp::TimerBase::SharedPtr control_timer_;
// };

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<JointControllerNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }



#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
using namespace std::chrono_literals;


class JointControllerNode : public rclcpp::Node{
public:
     JointControllerNode() : Node("joint_controller_node"){

      this->declare_parameter<double>("max_joint_velocity", 0.2);
      max_velocity = this->get_parameter("max_joint_velocity").as_double();

      min_limits = {-1.5708, -1.5708, -1.5708, -1.5708, -3.1415};
      max_limits = { 1.5708,  1.5708,  1.5708,  1.5708,  3.1415};

      num_joints = 5;
      joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};

      //State Buffers
      current_angles.assign(num_joints, 0.0);
      target_angles.assign(num_joints, 0.0);
      interpolated_cmd.assign(num_joints, 0.0);

      target_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>
                  ("/target_joint_angles", 10,
                    std::bind(&JointControllerNode::jointCallback, this , std::placeholders::_1)  );

      joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>
                        ("/joint_states", 10,
                          std::bind(&JointControllerNode::jointStateCallback, this, std::placeholders::_1));

      joint_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_commands", 10);

      timer_ = this->create_wall_timer(
                20ms, 
                std::bind(&JointControllerNode::controlLoop, this));

      RCLCPP_INFO(this->get_logger(), "Hardware-Ready Joint Controller Node Initialized at 50 Hz.");


     }

private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    size_t num_joints;
    double max_velocity;
    std::vector<double> min_limits;
    std::vector<double> max_limits;
    std::vector<std::string> joint_names;

    std::vector<double> current_angles;
    std::vector<double> target_angles;
    std::vector<double> interpolated_cmd;

    void jointCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      if(msg->data.size() < num_joints){
        RCLCPP_INFO(this->get_logger(), "Insufficient joint velocity commands.");
        return;
      }

      for(size_t i=0; i<num_joints; i++){
        target_angles[i] = std::clamp(msg->data[i], min_limits[i], max_limits[i]);
      }
      
      RCLCPP_INFO(this->get_logger(), "New target setpoint accepted: [%.2f, %.2f, %.2f, %.2f, %.2f]",
      target_angles[0], target_angles[1], target_angles[2], target_angles[3], target_angles[4]);


    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg){

      for (size_t i = 0; i < num_joints; ++i) {
        auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
        if (it != msg->name.end()) {
          size_t idx = std::distance(msg->name.begin(), it);
          if (idx < msg->position.size()) {
            current_angles[i] = msg->position[idx];
          }
        }
      }

    }
    
    void controlLoop(){
      double dt = 0.02;
      double max_step = max_velocity * dt;

      for (size_t i = 0; i < num_joints; ++i) {
        double error = target_angles[i] - interpolated_cmd[i];
        if (std::abs(error) <= max_step) {
          interpolated_cmd[i] = target_angles[i];
        } else {
          interpolated_cmd[i] += std::copysign(max_step, error);
        }
      }

      auto cmd_msg = std_msgs::msg::Float64MultiArray();
      cmd_msg.data = interpolated_cmd;
      joint_cmd_pub->publish(cmd_msg);
    }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
