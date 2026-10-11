#include <memory>
#include <string>
#include <vector>

#include "controller/nova_ik.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// /target_object_pose (PointStamped, base_link) -> /target_joint_angles (Float64MultiArray[5])
// All math lives in nova_ik.hpp (ROS-free, unit-testable). Unreachable targets are REJECTED, never clamped.
class IkSolverNode : public rclcpp::Node {
public:
  IkSolverNode() : Node("ik_solver") {
    nova::IkParams d;
    params_.L3 = declare_parameter<double>("tool_length", d.L3);        // joint4 -> grasp point [m]
    params_.base_yaw = declare_parameter<double>("base_yaw", d.base_yaw);
    params_.d1 = declare_parameter<double>("d1", d.d1);

    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/target_joint_angles", 10);
    sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/target_object_pose", 10,
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) { onTarget(*msg); });
    RCLCPP_INFO(get_logger(), "IK ready (L3=%.3f, base_yaw=%.3f, d1=%.3f)", params_.L3, params_.base_yaw, params_.d1);
  }

private:
  void onTarget(const geometry_msgs::msg::PointStamped& msg) {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Target frame '%s' != base_link, ignored",
                           msg.header.frame_id.c_str());
      return;
    }
    const auto r = nova::solveIK(msg.point.x, msg.point.y, msg.point.z, params_);
    if (!r.ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "IK rejected (%.3f, %.3f, %.3f): %s",
                           msg.point.x, msg.point.y, msg.point.z, r.reason.c_str());
      return;
    }
    std_msgs::msg::Float64MultiArray out;
    out.data.assign(r.q.begin(), r.q.end());
    pub_->publish(out);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "IK ok (%.3f, %.3f, %.3f) elev=%.0f deg -> [%.2f %.2f %.2f %.2f %.2f]",
                         msg.point.x, msg.point.y, msg.point.z, r.elevation * 180.0 / M_PI,
                         r.q[0], r.q[1], r.q[2], r.q[3], r.q[4]);
  }

  nova::IkParams params_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IkSolverNode>());
  rclcpp::shutdown();
  return 0;
}
