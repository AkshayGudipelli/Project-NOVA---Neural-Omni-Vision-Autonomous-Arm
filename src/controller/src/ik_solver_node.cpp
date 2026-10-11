#include <vector>
#include <cmath>
#include <algorithm>


#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"


class ikSolverNode : public rclcpp::Node {
public:
  ikSolverNode() : Node("ik_solver") {
    // Physical Link Dimensions (meters)
    d1 = 0.07;   // Base height to Joint 2 pitch axis
    L1 = 0.105;  // Upper arm link length
    L2 = 0.095;  // Forearm link length
    L3 = 0.120;  // Wrist to end-effector distance

    // Physical Joint Limits (radians)
    min_limits = {-1.5708, -1.5708, -2.0000, -1.5708, -3.1415};
    max_limits = { 1.5708,  1.5708,  2.0000,  1.5708,  3.1415};

    joint_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/target_joint_angles", 10);
    target_pose_sub = this->create_subscription<geometry_msgs::msg::PointStamped>(
                      "/target_object_pose", 10 , 
                      std::bind(&ikSolverNode::targetPoseCallback, this , std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Analytical IK Solver Node Initialized.");
  }

  /**
   * @brief Sub-phase 5.1: Analytical Inverse Kinematics Calculation
   * Computes the 5 joint angles required to position the end-effector at (x, y, z).
   */
  bool computeIK(double x, double y, double z, std::vector<double>& joint_angles) {
    joint_angles.resize(5, 0.0);

    // 1. Joint 1: Base Pan Angle (azimuth angle in XY plane)
    double q1 = std::atan2(y, x);

    // 2. 2D Planar Projection & Relative Height
    double r = std::sqrt(x * x + y * y);
    double z_rel = z - d1;

    // Target approach pitch angle (e.g., -45 deg angled grasp)
    double theta_pitch = -0.785398; // -pi/4 rad

    // Wrist Center Position (rw, zw)
    double rw = r - L3 * std::cos(theta_pitch);
    double zw = z_rel - L3 * std::sin(theta_pitch);

    // Distance squared from Joint 2 to Wrist Center
    double D2 = rw * rw + zw * zw;
    double D = std::sqrt(D2);

    // 3. Law of Cosines for Elbow Pitch Angle (q3)
    double cos_beta = (L1 * L1 + L2 * L2 - D2) / (2.0 * L1 * L2);
    if (cos_beta < -1.0 || cos_beta > 1.0) {
      RCLCPP_WARN(this->get_logger(), "Target position (%.3f, %.3f, %.3f) is out of reachable workspace!", x, y, z);
      return false; // Target unreachable
    }
    double beta = std::acos(cos_beta);
    double q3 = 3.1415926535 - beta;

    // 4. Shoulder Pitch Angle (q2)
    double alpha1 = std::atan2(zw, rw);
    double cos_alpha2 = (L1 * L1 + D2 - L2 * L2) / (2.0 * L1 * D);
    cos_alpha2 = std::clamp(cos_alpha2, -1.0, 1.0);
    double alpha2 = std::acos(cos_alpha2);
    double q2 = (3.1415926535 / 2.0) - (alpha1 + alpha2);

    // 5. Wrist Pitch Angle (q4) & Wrist Roll Angle (q5)
    double q4 = theta_pitch - (q2 + q3);
    double q5 = 0.0;

    // 6. Clamp calculated joint angles to physical joint limits
    joint_angles[0] = std::clamp(q1, min_limits[0], max_limits[0]);
    joint_angles[1] = std::clamp(q2, min_limits[1], max_limits[1]);
    joint_angles[2] = std::clamp(q3, min_limits[2], max_limits[2]);
    joint_angles[3] = std::clamp(q4, min_limits[3], max_limits[3]);
    joint_angles[4] = std::clamp(q5, min_limits[4], max_limits[4]);

    return true;
  }

  void targetPoseCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    double x = msg->point.x;
    double y = msg->point.y;
    double z = msg->point.z;
    
    std::vector<double> joint_angles;
    if (computeIK(x, y, z, joint_angles)){
        auto target_joint_angle_msg = std_msgs::msg::Float64MultiArray();
        target_joint_angle_msg.data = joint_angles;
        joint_cmd_pub->publish(target_joint_angle_msg);

        RCLCPP_INFO(this->get_logger(),
          "IK Solution Found for Target (X=%.3f m, Y=%.3f m, Z=%.3f m) -> Joints: [q1: %.2f, q2: %.2f, q3: %.2f, q4: %.2f, q5: %.2f]",
          x, y, z, joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4]);
    }

  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_pose_sub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub;


  double d1, L1, L2, L3;
  std::vector<double> min_limits;
  std::vector<double> max_limits;


};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ikSolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}