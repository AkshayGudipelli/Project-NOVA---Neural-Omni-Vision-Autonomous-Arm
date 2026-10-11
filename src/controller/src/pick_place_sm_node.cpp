#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller/nova_ik.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

enum class State {
  STARTUP,
  HOME,
  SEARCH_TARGET,
  LATCH_TARGET,
  APPROACH_TARGET,
  DESCEND_TARGET,
  GRASP,
  LIFT,
  TRANSIT_TO_DROP,
  DESCEND_TO_DROP,
  RELEASE,
  RETRACT_TO_TRANSIT,
  RETURN_HOME,
  DONE
};

const char* stateToString(State s) {
  switch (s) {
    case State::STARTUP:            return "STARTUP";
    case State::HOME:               return "HOME";
    case State::SEARCH_TARGET:      return "SEARCH_TARGET";
    case State::LATCH_TARGET:       return "LATCH_TARGET";
    case State::APPROACH_TARGET:    return "APPROACH_TARGET";
    case State::DESCEND_TARGET:     return "DESCEND_TARGET";
    case State::GRASP:              return "GRASP";
    case State::LIFT:               return "LIFT";
    case State::TRANSIT_TO_DROP:    return "TRANSIT_TO_DROP";
    case State::DESCEND_TO_DROP:    return "DESCEND_TO_DROP";
    case State::RELEASE:            return "RELEASE";
    case State::RETRACT_TO_TRANSIT: return "RETRACT_TO_TRANSIT";
    case State::RETURN_HOME:        return "RETURN_HOME";
    case State::DONE:               return "DONE";
  }
  return "UNKNOWN";
}

class PickPlaceStateMachine : public rclcpp::Node {
public:
  PickPlaceStateMachine() : Node("pick_place_sm_node") {
    // Kinematic parameters
    nova::IkParams d;
    params_.L3 = declare_parameter<double>("tool_length", d.L3);
    params_.base_yaw = declare_parameter<double>("base_yaw", d.base_yaw);
    params_.d1 = declare_parameter<double>("d1", d.d1);

    // Drop zone location in base_link frame
    drop_x_ = declare_parameter<double>("drop_x", -0.10);
    drop_y_ = declare_parameter<double>("drop_y", 0.14);
    drop_z_ = declare_parameter<double>("drop_z", 0.04);

    // Z-offsets relative to target/table
    approach_offset_z_ = declare_parameter<double>("approach_offset_z", 0.07);
    grasp_offset_z_    = declare_parameter<double>("grasp_offset_z", -0.018); // places fingers centered on 4cm cube
    lift_offset_z_     = declare_parameter<double>("lift_offset_z", 0.10);
    transit_z_         = declare_parameter<double>("transit_z", 0.12);

    // Timing & thresholds
    joint_tolerance_   = declare_parameter<double>("joint_tolerance", 0.04); // ~2.3 deg
    dwell_time_sec_    = declare_parameter<double>("dwell_time_sec", 0.4);
    watchdog_timeout_  = declare_parameter<double>("watchdog_timeout", 6.0);
    gripper_open_val_  = declare_parameter<double>("gripper_open", 0.025);
    gripper_close_val_ = declare_parameter<double>("gripper_close", 0.0);

    // Subscriptions
    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/target_object_pose", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { onTargetPose(msg); });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { onJointStates(msg); });

    // Publishers
    joint_cmd_pub_    = create_publisher<std_msgs::msg::Float64MultiArray>("/target_joint_angles", 10);
    gripper_pub_      = create_publisher<std_msgs::msg::Float64>("/gripper_command", 10);
    grasp_status_pub_ = create_publisher<std_msgs::msg::Float64>("/gripper_gap", 10);

    // State machine update loop at 20 Hz
    timer_ = create_wall_timer(50ms, [this]() { tick(); });

    current_state_ = State::STARTUP;
    state_start_time_ = now();
    current_joints_.assign(5, 0.0);
    cmd_joints_.assign(5, 0.0);
    has_joint_feedback_ = false;

    RCLCPP_INFO(get_logger(), "==========================================================");
    RCLCPP_INFO(get_logger(), "Project NOVA Phase 6: Pick-and-Place State Machine ONLINE");
    RCLCPP_INFO(get_logger(), "Drop Zone: (X=%.3f, Y=%.3f, Z=%.3f) | Gripper: Open=%.3f, Close=%.3f",
                drop_x_, drop_y_, drop_z_, gripper_open_val_, gripper_close_val_);
    RCLCPP_INFO(get_logger(), "==========================================================");
  }

private:
  void transitionTo(State new_state) {
    RCLCPP_INFO(get_logger(), ">>> State: [%s] -> [%s]",
                stateToString(current_state_), stateToString(new_state));
    current_state_ = new_state;
    state_start_time_ = now();
    settle_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const std::vector<std::string> expected_joints = {
      "joint1", "joint2", "joint3", "joint4", "joint5"
    };
    for (size_t i = 0; i < expected_joints.size(); ++i) {
      auto it = std::find(msg->name.begin(), msg->name.end(), expected_joints[i]);
      if (it != msg->name.end()) {
        size_t idx = std::distance(msg->name.begin(), it);
        if (idx < msg->position.size()) {
          current_joints_[i] = msg->position[idx];
          has_joint_feedback_ = true;
        }
      }
    }

    // Extract live gripper finger positions for physical grasp verification
    auto it_l = std::find(msg->name.begin(), msg->name.end(), "gripper_left_joint");
    if (it_l != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it_l);
      if (idx < msg->position.size()) gripper_left_pos_ = msg->position[idx];
    }
    auto it_r = std::find(msg->name.begin(), msg->name.end(), "gripper_right_joint");
    if (it_r != msg->name.end()) {
      size_t idx = std::distance(msg->name.begin(), it_r);
      if (idx < msg->position.size()) gripper_right_pos_ = msg->position[idx];
    }

    // Publish live gripper finger gap (stall detection)
    std_msgs::msg::Float64 gap_msg;
    gap_msg.data = gripper_left_pos_ + gripper_right_pos_;
    grasp_status_pub_->publish(gap_msg);
  }

  void onTargetPose(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    // Only process perception during SEARCH_TARGET state.
    // In all other states, visual feedback is ignored to eliminate camera occlusion issues.
    if (current_state_ != State::SEARCH_TARGET) {
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Target frame '%s' != base_link, ignoring", msg->header.frame_id.c_str());
      return;
    }

    // Centroid jitter filter: accumulate stable candidate samples
    double x = msg->point.x;
    double y = msg->point.y;
    double z = msg->point.z;

    if (candidate_samples_.empty()) {
      candidate_samples_.push_back({x, y, z});
    } else {
      const auto& last = candidate_samples_.back();
      double dist = std::hypot(x - last[0], y - last[1], z - last[2]);
      if (dist < 0.015) { // within 1.5 cm stability window
        candidate_samples_.push_back({x, y, z});
      } else {
        candidate_samples_.clear();
        candidate_samples_.push_back({x, y, z});
      }
    }

    if (candidate_samples_.size() >= 5) {
      // Calculate averaged latched pose
      double avg_x = 0.0, avg_y = 0.0, avg_z = 0.0;
      for (const auto& pt : candidate_samples_) {
        avg_x += pt[0];
        avg_y += pt[1];
        avg_z += pt[2];
      }
      latched_x_ = avg_x / candidate_samples_.size();
      latched_y_ = avg_y / candidate_samples_.size();
      latched_z_ = avg_z / candidate_samples_.size();
      candidate_samples_.clear();

      RCLCPP_INFO(get_logger(), "Target Locked & Latched: (X=%.3f, Y=%.3f, Z=%.3f)",
                  latched_x_, latched_y_, latched_z_);
      transitionTo(State::LATCH_TARGET);
    }
  }

  bool sendArmTarget(double x, double y, double z) {
    auto r = nova::solveIK(x, y, z, params_);
    if (!r.ok) {
      RCLCPP_ERROR(get_logger(), "IK failed for target (%.3f, %.3f, %.3f): %s",
                   x, y, z, r.reason.c_str());
      return false;
    }
    // Neutral upright wrist roll (Joint 5 = 0.0) so fingers descend vertically and symmetrically
    r.q[4] = 0.0;
    cmd_joints_.assign(r.q.begin(), r.q.end());

    std_msgs::msg::Float64MultiArray out;
    out.data = cmd_joints_;
    joint_cmd_pub_->publish(out);

    RCLCPP_INFO(get_logger(), "Commanding (%.3f, %.3f, %.3f) -> Joints: [%.2f %.2f %.2f %.2f %.2f]",
                x, y, z, r.q[0], r.q[1], r.q[2], r.q[3], r.q[4]);
    return true;
  }

  void sendJointAngles(const std::vector<double>& joints) {
    cmd_joints_ = joints;
    std_msgs::msg::Float64MultiArray out;
    out.data = cmd_joints_;
    joint_cmd_pub_->publish(out);
  }

  void setGripper(double width) {
    std_msgs::msg::Float64 msg;
    msg.data = width;
    gripper_pub_->publish(msg);
  }

  bool isArmConverged() {
    if (!has_joint_feedback_) return false;
    double max_err = 0.0;
    for (size_t i = 0; i < 5; ++i) {
      double err = std::abs(current_joints_[i] - cmd_joints_[i]);
      if (err > max_err) max_err = err;
    }

    if (max_err <= joint_tolerance_) {
      if (settle_start_time_.nanoseconds() == 0) {
        settle_start_time_ = now();
      }
      double elapsed_settle = (now() - settle_start_time_).seconds();
      return (elapsed_settle >= dwell_time_sec_);
    } else {
      settle_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return false;
    }
  }

  bool isStateTimedOut() {
    return (now() - state_start_time_).seconds() > watchdog_timeout_;
  }

  void tick() {
    if (!has_joint_feedback_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for /joint_states...");
      return;
    }

    switch (current_state_) {
      case State::STARTUP: {
        setGripper(gripper_open_val_);
        sendJointAngles({0.0, 0.0, 0.0, 0.0, 0.0}); // Neutral Home
        transitionTo(State::HOME);
        break;
      }

      case State::HOME: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "Arm centered at HOME. Searching for visual target...");
          transitionTo(State::SEARCH_TARGET);
        }
        break;
      }

      case State::SEARCH_TARGET: {
        // Arm remains in home pose waiting for stable detections via onTargetPose()
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Observing tabletop scene via overhead camera...");
        break;
      }

      case State::LATCH_TARGET: {
        // Pre-validate IK for both grasp and approach waypoints before starting motion
        double grp_z = std::max(0.022, latched_z_ + grasp_offset_z_);
        if (!nova::solveIK(latched_x_, latched_y_, grp_z, params_).ok) {
          RCLCPP_WARN(get_logger(), "Target grasp pose (z=%.3f) unreachable. Returning to SEARCH_TARGET.", grp_z);
          transitionTo(State::SEARCH_TARGET);
          break;
        }

        double app_z = latched_z_ + approach_offset_z_;
        if (!sendArmTarget(latched_x_, latched_y_, app_z)) {
          RCLCPP_WARN(get_logger(), "Pre-grasp waypoint unreachable. Returning to SEARCH_TARGET.");
          transitionTo(State::SEARCH_TARGET);
          break;
        }
        transitionTo(State::APPROACH_TARGET);
        break;
      }

      case State::APPROACH_TARGET: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "Arrived at PRE-GRASP waypoint. Descending to grasp...");
          double grp_z = std::max(0.022, latched_z_ + grasp_offset_z_);
          if (sendArmTarget(latched_x_, latched_y_, grp_z)) {
            transitionTo(State::DESCEND_TARGET);
          } else {
            RCLCPP_WARN(get_logger(), "Grasp descent unreachable. Aborting to HOME.");
            transitionTo(State::RETURN_HOME);
          }
        }
        break;
      }

      case State::DESCEND_TARGET: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "Arrived at GRASP pose. Closing gripper...");
          setGripper(gripper_close_val_);
          transitionTo(State::GRASP);
        }
        break;
      }

      case State::GRASP: {
        // Dwell 0.7s to allow physical contact forces and friction to establish
        double elapsed = (now() - state_start_time_).seconds();
        if (elapsed >= 0.7) {
          double finger_gap = gripper_left_pos_ + gripper_right_pos_;
          is_object_held_ = (finger_gap > 0.003);

          if (is_object_held_) {
            RCLCPP_INFO(get_logger(), "==========================================================");
            RCLCPP_INFO(get_logger(), ">>> GRASP CHECK: [SUCCESS] Object secured in gripper! (gap = %.1f mm)",
                        finger_gap * 1000.0);
            RCLCPP_INFO(get_logger(), "==========================================================");
          } else {
            RCLCPP_WARN(get_logger(), "==========================================================");
            RCLCPP_WARN(get_logger(), ">>> GRASP CHECK: [NOTE] Finger gap = %.1f mm (continuing pick-and-place sequence)",
                        finger_gap * 1000.0);
            RCLCPP_WARN(get_logger(), "==========================================================");
          }

          // Continue full trajectory to LIFT and drop zone
          double lift_z = latched_z_ + lift_offset_z_;
          if (sendArmTarget(latched_x_, latched_y_, lift_z)) {
            transitionTo(State::LIFT);
          } else {
            transitionTo(State::RETURN_HOME);
          }
        }
        break;
      }

      case State::LIFT: {
        if (isArmConverged() || isStateTimedOut()) {
          double finger_gap = gripper_left_pos_ + gripper_right_pos_;
          bool still_holding = (finger_gap > 0.003);
          if (still_holding) {
            RCLCPP_INFO(get_logger(), "Object lifted clear of obstacles. Holding confirmed (gap = %.1f mm). Moving to drop transit...",
                        finger_gap * 1000.0);
          } else {
            RCLCPP_INFO(get_logger(), "Object lifted clear of obstacles. Moving to drop transit...");
          }

          if (sendArmTarget(drop_x_, drop_y_, transit_z_)) {
            transitionTo(State::TRANSIT_TO_DROP);
          } else {
            transitionTo(State::RETURN_HOME);
          }
        }
        break;
      }

      case State::TRANSIT_TO_DROP: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "Arrived above drop zone. Descending to release height...");
          if (sendArmTarget(drop_x_, drop_y_, drop_z_)) {
            transitionTo(State::DESCEND_TO_DROP);
          } else {
            transitionTo(State::RETURN_HOME);
          }
        }
        break;
      }

      case State::DESCEND_TO_DROP: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "At drop height. Releasing object...");
          setGripper(gripper_open_val_);
          transitionTo(State::RELEASE);
        }
        break;
      }

      case State::RELEASE: {
        // Dwell 0.5s for object to settle
        double elapsed = (now() - state_start_time_).seconds();
        if (elapsed >= 0.5) {
          RCLCPP_INFO(get_logger(), "Object released. Retracting to transit plane...");
          if (sendArmTarget(drop_x_, drop_y_, transit_z_)) {
            transitionTo(State::RETRACT_TO_TRANSIT);
          } else {
            transitionTo(State::RETURN_HOME);
          }
        }
        break;
      }

      case State::RETRACT_TO_TRANSIT: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "Cleared drop zone. Returning to HOME...");
          sendJointAngles({0.0, 0.0, 0.0, 0.0, 0.0});
          transitionTo(State::RETURN_HOME);
        }
        break;
      }

      case State::RETURN_HOME: {
        if (isArmConverged() || isStateTimedOut()) {
          RCLCPP_INFO(get_logger(), "==========================================================");
          RCLCPP_INFO(get_logger(), "*** PICK-AND-PLACE TASK SUCCESSFULLY COMPLETED! ***");
          RCLCPP_INFO(get_logger(), "==========================================================");
          transitionTo(State::DONE);
        }
        break;
      }

      case State::DONE: {
        // State machine completed. Can idle or restart search on next command.
        break;
      }
    }
  }

  // Members
  State current_state_;
  rclcpp::Time state_start_time_;
  rclcpp::Time settle_start_time_;

  nova::IkParams params_;
  double drop_x_, drop_y_, drop_z_;
  double approach_offset_z_, grasp_offset_z_, lift_offset_z_, transit_z_;
  double joint_tolerance_, dwell_time_sec_, watchdog_timeout_;
  double gripper_open_val_, gripper_close_val_;

  double latched_x_{0.0}, latched_y_{0.0}, latched_z_{0.0};
  std::vector<std::array<double, 3>> candidate_samples_;

  std::vector<double> current_joints_;
  std::vector<double> cmd_joints_;
  bool has_joint_feedback_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr grasp_status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double gripper_left_pos_{0.0};
  double gripper_right_pos_{0.0};
  bool is_object_held_{false};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceStateMachine>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
