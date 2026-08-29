#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"

#include <mujoco/mujoco.h>
#include "mujoco_sim/sim_handle.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
  CameraNode()
  : Node("camera_node"),
    sim_handle_(nullptr),
    width_(640),
    height_(480),
    fovy_deg_(70.0),
    rendering_initialized_(false)
  {
    // Declare parameters
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<double>("fovy", 70.0);
    this->declare_parameter<std::string>("camera_name", "workspace_camera");

    width_ = this->get_parameter("image_width").as_int();
    height_ = this->get_parameter("image_height").as_int();
    fovy_deg_ = this->get_parameter("fovy").as_double();
    camera_name_ = this->get_parameter("camera_name").as_string();

    // ROS2 Publishers
    rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

    // Compute optical camera intrinsics matrix K
    computeCameraInfo();

    // 30 Hz Capture Timer Loop (33ms)
    timer_ = this->create_wall_timer(33ms, std::bind(&CameraNode::captureLoop, this));

    RCLCPP_INFO(this->get_logger(), "C++ RGB-D Camera Node Initialized at 30 Hz (%dx%d).", width_, height_);
  }

  void setSimHandle(SimHandlePtr sim_handle)
  {
    sim_handle_ = sim_handle;
    if (sim_handle_ && sim_handle_->model) {
      initMuJoCoRendering();
    }
  }

  ~CameraNode()
  {
    if (rendering_initialized_) {
      mjr_freeContext(&con_);
      mjv_freeScene(&scn_);
    }
  }

private:
  void computeCameraInfo()
  {
    double fovy_rad = fovy_deg_ * (M_PI / 180.0);
    double fy = (height_ / 2.0) / std::tan(fovy_rad / 2.0);
    double fx = fy; // Square pixels
    double cx = width_ / 2.0;
    double cy = height_ / 2.0;

    camera_info_msg_.header.frame_id = "camera_optical_frame";
    camera_info_msg_.height = height_;
    camera_info_msg_.width = width_;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};

    // 3x3 Intrinsic Matrix K
    camera_info_msg_.k = {
      fx,  0.0, cx,
      0.0, fy,  cy,
      0.0, 0.0, 1.0
    };

    // 3x4 Projection Matrix P
    camera_info_msg_.p = {
      fx,  0.0, cx,  0.0,
      0.0, fy,  cy,  0.0,
      0.0, 0.0, 1.0, 0.0
    };
  }

  void initMuJoCoRendering()
  {
    if (!sim_handle_ || !sim_handle_->model) return;

    // Lookup fixed camera ID in MuJoCo model
    int cam_id = mj_name2id(sim_handle_->model, mjOBJ_CAMERA, camera_name_.c_str());
    if (cam_id == -1) {
      RCLCPP_WARN(this->get_logger(), "Camera '%s' not found in model! Using free camera.", camera_name_.c_str());
      mjv_defaultCamera(&cam_);
    } else {
      mjv_defaultCamera(&cam_);
      cam_.type = mjCAMERA_FIXED;
      cam_.fixedcamid = cam_id;
    }

    mjv_defaultOption(&opt_);
    mjv_makeScene(sim_handle_->model, &scn_, 2000);

    mjr_defaultContext(&con_);
    mjr_makeContext(sim_handle_->model, &con_, mjFONTSCALE_150);

    rgb_buffer_.resize(width_ * height_ * 3);
    depth_buffer_.resize(width_ * height_);

    rendering_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "MuJoCo Offscreen Rendering Context Initialized for camera '%s'", camera_name_.c_str());
  }

  void captureLoop()
  {
    auto stamp = this->now();

    // If simulation handle is present, perform offscreen render
    if (sim_handle_ && sim_handle_->model && sim_handle_->data) {
      if (!rendering_initialized_) {
        initMuJoCoRendering();
      }

      // Thread-safe scene update from simulation state
      {
        std::lock_guard<std::mutex> lock(sim_handle_->mutex);
        mjv_updateScene(sim_handle_->model, sim_handle_->data, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
      } // Mutex unlocks immediately so 500 Hz physics is never blocked during GPU render!

      mjrRect viewport = {0, 0, width_, height_};
      mjr_render(viewport, &scn_, &con_);
      mjr_readPixels(rgb_buffer_.data(), depth_buffer_.data(), viewport, &con_);
    }

    // Publish RGB & Depth Images ONLY when rendering is active
    if (rendering_initialized_) {
      auto rgb_msg = sensor_msgs::msg::Image();
      rgb_msg.header.stamp = stamp;
      rgb_msg.header.frame_id = "camera_optical_frame";
      rgb_msg.height = height_;
      rgb_msg.width = width_;
      rgb_msg.encoding = sensor_msgs::image_encodings::RGB8;
      rgb_msg.is_bigendian = false;
      rgb_msg.step = width_ * 3;
      rgb_msg.data = rgb_buffer_;
      rgb_pub_->publish(rgb_msg);

      auto depth_msg = sensor_msgs::msg::Image();
      depth_msg.header.stamp = stamp;
      depth_msg.header.frame_id = "camera_optical_frame";
      depth_msg.height = height_;
      depth_msg.width = width_;
      depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      depth_msg.is_bigendian = false;
      depth_msg.step = width_ * sizeof(float);
      depth_msg.data.resize(height_ * depth_msg.step);
      std::memcpy(depth_msg.data.data(), depth_buffer_.data(), depth_msg.data.size());
      depth_pub_->publish(depth_msg);
    }

    // Always publish Camera Calibration Info
    camera_info_msg_.header.stamp = stamp;
    info_pub_->publish(camera_info_msg_);
  }

  SimHandlePtr sim_handle_;
  int width_;
  int height_;
  double fovy_deg_;
  std::string camera_name_;
  bool rendering_initialized_;

  mjvScene scn_;
  mjrContext con_;
  mjvOption opt_;
  mjvCamera cam_;

  std::vector<unsigned char> rgb_buffer_;
  std::vector<float> depth_buffer_;

  sensor_msgs::msg::CameraInfo camera_info_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef NO_MAIN
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif


// #include "rclcpp/rclcpp.hpp"
//     #include "sensor_msgs/msg/image.hpp"
//     #include "sensor_msgs/image_encodings.hpp"
//     #include "cv_bridge/cv_bridge.hpp"
//     #include "sensor_msgs/msg/camera_info.hpp"
//     #include <mujoco/mujoco.h>
//     #include "mujoco_sim/sim_handle.hpp"
//     #include <opencv2/opencv.hpp>
    
// using namespace std::chrono_literals;

// class CameraNode : public rclcpp::Node {
// public:
//     CameraNode() : Node("camera_node"){
//         computeCameraInfo();
//         camera_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", 10);

//         timer = this->create_wall_timer(
//                     30ms,
//                     std::bind(&CameraNode::publishSyntheticImage, this));

//         RCLCPP_INFO(this->get_logger(), "C++ Camera Info Node Initialized at 30 Hz.");
//     }

//     void setSimHandle(SimHandlePtr sim_handle) {
//         sim_handle_ = sim_handle;
//     }

//     ~CameraNode() {
//         if (rendering_initialized_) {
//             mjr_freeContext(&con_);
//             mjv_freeScene(&scn_);
//         }
//     }
    
//     private:
//         rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
//         rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
//         rclcpp::TimerBase::SharedPtr timer;
//         rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub;
    
//         SimHandlePtr sim_handle_{nullptr};
//         bool rendering_initialized_{false};
    
//         mjvScene scn_;
//         mjrContext con_;
//         mjvOption opt_;
//         mjvCamera cam_;
    
//         std::vector<unsigned char> rgb_buffer_;
//         std::vector<float> depth_buffer_;
    
//         int width = 640;
//         int height = 480;
//         double fovy = 70.0;
//         double theta_y;
//         double f_y, f_x;
//         double c_x, c_y;
    
//         void initMuJoCoRendering() {
//         if (!sim_handle_ || !sim_handle_->model) return;

//         publisher = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);
//         depth_pub = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", 10);
    
//         int cam_id = mj_name2id(sim_handle_->model, mjOBJ_CAMERA, "workspace_camera");
//         mjv_defaultCamera(&cam_);
//         if (cam_id == -1) {
//             RCLCPP_WARN(this->get_logger(), "Camera 'workspace_camera' not found! Using free camera.");
//         } else {
//             cam_.type = mjCAMERA_FIXED;
//             cam_.fixedcamid = cam_id;
//         }
    
//         mjv_defaultOption(&opt_);
//         mjv_makeScene(sim_handle_->model, &scn_, 2000);
//         mjr_defaultContext(&con_);
//         mjr_makeContext(sim_handle_->model, &con_, mjFONTSCALE_150);
    
//         rgb_buffer_.resize(width * height * 3);
//         depth_buffer_.resize(width * height);
//         rendering_initialized_ = true;
//         RCLCPP_INFO(this->get_logger(), "MuJoCo Offscreen Rendering Context Initialized.");
//         }
    
//         void computeCameraInfo() {
//         theta_y = fovy * M_PI / 180.0; // deg to rad conversion
//         f_y = height / (2.0 * std::tan(theta_y / 2.0));
//         f_x = f_y;
    
//         c_x = width / 2.0;
//         c_y = height / 2.0;
//         }
    
//         void publishSyntheticImage() {
//         // A. Render real 3D MuJoCo scene if sim handle is connected
//         if (sim_handle_ && sim_handle_->model && sim_handle_->data) {
//             if (!rendering_initialized_) {
//                 initMuJoCoRendering();
//             }

//             // Thread-safe Short-Lock Scene Snapshot (Microsecond lock!)
//             {
//                 std::lock_guard<std::mutex> lock(sim_handle_->mutex);
//                 mjv_updateScene(sim_handle_->model, sim_handle_->data, &opt_, nullptr, &cam_, mjCAT_ALL, &scn_);
//             } // MUTEX UNLOCKS HERE! Physics keeps running at 500 Hz!

//             // Offscreen GPU Render & Read Pixels
//             mjrRect viewport = {0, 0, width, height};
//             mjr_render(viewport, &scn_, &con_);
//             mjr_readPixels(rgb_buffer_.data(), depth_buffer_.data(), viewport, &con_);
//         }

//         auto stamp = this->now();

//         // Publish RGB & Depth ONLY when rendering is active
//         if (rendering_initialized_) {
//             cv::Mat image(height, width, CV_8UC3, rgb_buffer_.data());
//             cv::cvtColor(image, image, cv::COLOR_RGB2BGR); // Convert RGB to BGR for cv_bridge

//             std_msgs::msg::Header header;
//             header.stamp = stamp;
//             header.frame_id = "camera_optical_frame";
//             cv_bridge::CvImage cv_image(header, sensor_msgs::image_encodings::BGR8, image);
//             publisher->publish(*cv_image.toImageMsg());

//             cv::Mat depth_mat(height, width, CV_32FC1, depth_buffer_.data());
//             cv_bridge::CvImage depth_cv_img(header, sensor_msgs::image_encodings::TYPE_32FC1, depth_mat);
//             depth_pub->publish(*depth_cv_img.toImageMsg());
//         }

//         // Always publish optical camera intrinsics info
//         auto info_msg = sensor_msgs::msg::CameraInfo();
//         info_msg.header.stamp = stamp;
//         info_msg.header.frame_id = "camera_optical_frame";
//         info_msg.height = height;
//         info_msg.width = width;
//         info_msg.distortion_model = "plumb_bob";
//         info_msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
//         info_msg.k = {f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0};
//         info_msg.p = {f_x, 0.0, c_x, 0.0, 0.0, f_y, c_y, 0.0, 0.0, 0.0, 1.0, 0.0};
//         camera_info_pub->publish(info_msg);
//         }
//     };

//     #ifndef NO_MAIN
//     int main(int argc, char** argv) {
//         rclcpp::init(argc, argv);
//         rclcpp::spin(std::make_shared<CameraNode>());
//         rclcpp::shutdown();
//         return 0;
//     }
//     #endif
