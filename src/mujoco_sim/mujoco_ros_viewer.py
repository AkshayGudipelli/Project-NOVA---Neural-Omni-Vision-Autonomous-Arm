import os
import time
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge
import mujoco
import mujoco.viewer

class MujocoRosBridge(Node):
    def __init__(self, model, data):
        super().__init__('mujoco_ros_bridge')
        self.m = model
        self.d = data
        self.lock = threading.Lock()

        self.joint_names = [
            "joint1", "joint2", "joint3", "joint4", "joint5",
            "gripper_left_joint", "gripper_right_joint"
        ]

        # Target control buffer
        self.target_ctrl = [0.0] * self.m.nu
        if self.m.nu >= 7:
            self.target_ctrl[5] = 0.02
            self.target_ctrl[6] = 0.02

        # Subscribers
        self.create_subscription(
            Float64MultiArray, '/joint_commands', self.joint_cmd_cb, 10
        )
        self.create_subscription(
            Float64, '/gripper_command', self.gripper_cmd_cb, 10
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.create_timer(0.02, self.publish_joint_states) # 50 Hz

        # Offscreen Camera Vision Publishers (RGB + Metric Depth)
        self.renderer = mujoco.Renderer(self.m, height=480, width=640)
        self.cv_bridge = CvBridge()
        self.rgb_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.last_render_time = 0.0

    def joint_cmd_cb(self, msg):
        with self.lock:
            for i in range(min(5, len(msg.data))):
                self.target_ctrl[i] = msg.data[i]

    def gripper_cmd_cb(self, msg):
        with self.lock:
            width = max(0.0, min(0.03, msg.data))
            if self.m.nu >= 7:
                self.target_ctrl[5] = width
                self.target_ctrl[6] = width

    def publish_joint_states(self):
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [float(self.d.qpos[i]) for i in range(len(self.joint_names))]
            msg.velocity = [float(self.d.qvel[i]) for i in range(len(self.joint_names))]
            self.joint_state_pub.publish(msg)

    def publish_camera_frames(self):
        # Throttle to 30 FPS on the main OpenGL thread
        now = time.time()
        if now - self.last_render_time < 0.033:
            return
        self.last_render_time = now

        with self.lock:
            # Render RGB from workspace_camera
            self.renderer.disable_depth_rendering()
            self.renderer.update_scene(self.d, camera="workspace_camera")
            rgb_array = self.renderer.render()

            # Render Depth (Metric distance in meters)
            self.renderer.enable_depth_rendering()
            self.renderer.update_scene(self.d, camera="workspace_camera")
            depth_array = self.renderer.render()

        stamp = self.get_clock().now().to_msg()

        # RGB ROS2 Message
        rgb_msg = self.cv_bridge.cv2_to_imgmsg(rgb_array, encoding="rgb8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "camera_optical_frame"
        self.rgb_pub.publish(rgb_msg)

        # Depth ROS2 Message (32-bit Float32 in meters)
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_array, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_optical_frame"
        self.depth_pub.publish(depth_msg)

    def apply_control_and_step(self):
        with self.lock:
            for i in range(self.m.nu):
                self.d.ctrl[i] = self.target_ctrl[i]
            mujoco.mj_step(self.m, self.d)


def main():
    rclpy.init()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    scene_path = os.path.join(script_dir, "models", "scene.xml")

    print(f"Loading MuJoCo scene: {scene_path}")
    model = mujoco.MjModel.from_xml_path(scene_path)
    data = mujoco.MjData(model)

    bridge_node = MujocoRosBridge(model, data)

    # Spin ROS2 callbacks on a background daemon thread
    ros_thread = threading.Thread(target=rclpy.spin, args=(bridge_node,), daemon=True)
    ros_thread.start()

    print("Opening interactive MuJoCo Simulation Window...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            step_start = time.time()

            # Step physics and apply ROS2 joint commands
            bridge_node.apply_control_and_step()

            # Render & publish offscreen camera frames from main OpenGL thread (30 Hz)
            bridge_node.publish_camera_frames()

            # Render live viewer frame
            viewer.sync()

            # Maintain simulation clock pace
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
