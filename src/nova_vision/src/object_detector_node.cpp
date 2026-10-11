#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <cmath>

using namespace std::chrono_literals;

struct ColorPreset {
    std::string name;
    cv::Scalar lower1;
    cv::Scalar upper1;
    cv::Scalar lower2{0, 0, 0};
    cv::Scalar upper2{0, 0, 0};
    bool dual_range{false};
    float marker_r{1.0f};
    float marker_g{0.0f};
    float marker_b{0.0f};
};

class ObjectDetectionNode : public rclcpp::Node {

public:
    ObjectDetectionNode() : Node("object_detection_node") {
        setupColorPresets();

        // Declare dynamic target_color parameter
        this->declare_parameter<std::string>("target_color", "red");
        target_color_ = this->get_parameter("target_color").as_string();
        std::transform(target_color_.begin(), target_color_.end(), target_color_.begin(), ::tolower);

        // Dynamic Parameter Reconfiguration Callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                for (const auto& param : params) {
                    if (param.get_name() == "target_color") {
                        std::string new_color = param.as_string();
                        std::transform(new_color.begin(), new_color.end(), new_color.begin(), ::tolower);
                        if (presets_.find(new_color) != presets_.end()) {
                            target_color_ = new_color;
                            RCLCPP_INFO(this->get_logger(),
                                "==========================================================");
                            RCLCPP_INFO(this->get_logger(),
                                "*** Target Color DYNAMICALLY Changed to: '%s' ***", target_color_.c_str());
                            RCLCPP_INFO(this->get_logger(),
                                "==========================================================");
                        } else {
                            result.successful = false;
                            result.reason = "Unknown color. Available: red, orange, yellow, green, cyan, blue, purple, magenta, white, black";
                            RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
                        }
                    }
                }
                return result;
            });

        // Phase 4.1: Subscribing to 2D Color Camera Stream
        cam_data = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, 
            std::bind(&ObjectDetectionNode::imgModification, this, std::placeholders::_1));  

        // Phase 4.2: Subscribing to Metric Depth Stream (32FC1 meters)
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
             "/camera/depth/image_raw", 10,
             std::bind(&ObjectDetectionNode::depthCallback, this, std::placeholders::_1));

        // Phase 4.3: Initializing ROS2 TF2 Buffer & Listener Infrastructure
        tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);

        // Phase 4.4: Publish the target pose and publish 3D RViz Marker
        target_obj_pose_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_object_pose", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        RCLCPP_INFO(this->get_logger(),
            "3D Perception Node Ready. Active Target Color: '%s'", target_color_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Available Presets: [red, orange, yellow, green, cyan, blue, purple, magenta, white, black]");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_data;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_obj_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    cv::Mat depth_image;

    // Active color target & presets map
    std::string target_color_{"red"};
    std::unordered_map<std::string, ColorPreset> presets_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    void setupColorPresets() {
        // Red (dual range because Hue wraps around 0 and 180)
        presets_["red"] = {"red", cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255),
                           cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), true, 1.0f, 0.0f, 0.0f};

        // Orange (H ~ 11-25)
        presets_["orange"] = {"orange", cv::Scalar(11, 130, 80), cv::Scalar(25, 255, 255),
                              cv::Scalar(), cv::Scalar(), false, 1.0f, 0.5f, 0.0f};

        // Yellow (H ~ 26-38)
        presets_["yellow"] = {"yellow", cv::Scalar(26, 120, 80), cv::Scalar(38, 255, 255),
                              cv::Scalar(), cv::Scalar(), false, 1.0f, 1.0f, 0.0f};

        // Green (H ~ 39-85)
        presets_["green"] = {"green", cv::Scalar(39, 100, 60), cv::Scalar(85, 255, 255),
                             cv::Scalar(), cv::Scalar(), false, 0.0f, 1.0f, 0.0f};

        // Cyan (H ~ 86-105)
        presets_["cyan"] = {"cyan", cv::Scalar(86, 100, 60), cv::Scalar(105, 255, 255),
                            cv::Scalar(), cv::Scalar(), false, 0.0f, 1.0f, 1.0f};

        // Blue (H ~ 106-135)
        presets_["blue"] = {"blue", cv::Scalar(106, 120, 60), cv::Scalar(135, 255, 255),
                            cv::Scalar(), cv::Scalar(), false, 0.0f, 0.3f, 1.0f};

        // Purple (H ~ 136-155)
        presets_["purple"] = {"purple", cv::Scalar(136, 80, 40), cv::Scalar(155, 255, 255),
                              cv::Scalar(), cv::Scalar(), false, 0.6f, 0.0f, 0.8f};

        // Magenta / Pink (H ~ 145-169)
        presets_["magenta"] = {"magenta", cv::Scalar(145, 100, 60), cv::Scalar(169, 255, 255),
                               cv::Scalar(), cv::Scalar(), false, 1.0f, 0.0f, 1.0f};
        presets_["pink"] = presets_["magenta"];

        // White (Low S, High V)
        presets_["white"] = {"white", cv::Scalar(0, 0, 180), cv::Scalar(180, 45, 255),
                             cv::Scalar(), cv::Scalar(), false, 1.0f, 1.0f, 1.0f};

        // Black (Low V)
        presets_["black"] = {"black", cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 55),
                             cv::Scalar(), cv::Scalar(), false, 0.2f, 0.2f, 0.2f};
    }

    // TF2 Listener & Buffer members
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // Camera Pinhole Intrinsics (fovy = 70 deg, 640x480 resolution)
    const double fx = 342.75;
    const double fy = 342.75;
    const double cx = 320.0;
    const double cy = 240.0;

    // Phase 4.2 Sub-Step: Metric Depth Callback
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (!msg || msg->data.empty()) return;
        try {
            cv_bridge::CvImagePtr cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            depth_image = cv_depth_ptr->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge depth exception: %s", e.what());
        }
    }

    void imgModification(const sensor_msgs::msg::Image::SharedPtr msg) {

      auto marker_msg = visualization_msgs::msg::Marker();

        if (!msg || msg->data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received an empty message.");
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception thrown: %s", e.what());
            return;
        }
        
        // ------------------------------------------------------------------------------
        // PHASE 4.1: 2D COLOR SEGMENTATION & CENTROID CALCULATION
        // ------------------------------------------------------------------------------
        // Step 4.1a: Convert BGR Color Space to HSV (Hue, Saturation, Value)
        cv::Mat hsv_img;
        cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

        // Step 4.1b: Retrieve active color preset & apply HSV mask
        auto it_preset = presets_.find(target_color_);
        if (it_preset == presets_.end()) {
            it_preset = presets_.find("red");
        }
        const auto& preset = it_preset->second;

        cv::Mat mask;
        if (preset.dual_range) {
            cv::Mat mask1, mask2;
            cv::inRange(hsv_img, preset.lower1, preset.upper1, mask1);
            cv::inRange(hsv_img, preset.lower2, preset.upper2, mask2);
            cv::bitwise_or(mask1, mask2, mask);
        } else {
            cv::inRange(hsv_img, preset.lower1, preset.upper1, mask);
        }

        // Step 4.1c: Find Contours & Identify Target Shape (Filtering Noise & Table Background)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0.0;
        int max_idx = -1;

        if (!contours.empty()) {
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                // Reject table background (> 6,000 px) and camera noise (< 80 px)
                if (area > 80.0 && area < 6000.0 && area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }
        }

        // Step 4.1d: Compute 2D Pixel Centroid (u, v) using Image Moments
        if (max_idx != -1 && max_area > 80.0) {
            cv::Moments m = cv::moments(contours[max_idx]);
            if (m.m00 > 0) {
                double u = m.m10 / m.m00; // Pixel Column u
                double v = m.m01 / m.m00; // Pixel Row v
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Target [%s] Detected at Pixel (u=%.1f, v=%.1f) | Area: %.1f",
                    preset.name.c_str(), u, v, max_area);

                // ----------------------------------------------------------------------
                // PHASE 4.2: DEPTH LOOKUP & PINHOLE BACK-PROJECTION (CAMERA FRAME)
                // ----------------------------------------------------------------------
                if (!depth_image.empty()) {
                    // 5x5 Median depth lookup to filter noise
                    float Z_cam = 0.0f;
                    std::vector<float> valid_depths;
                    int u_int = static_cast<int>(u);
                    int v_int = static_cast<int>(v);

                    for (int dy = -2; dy <= 2; ++dy) {
                        for (int dx = -2; dx <= 2; ++dx) {
                            int py = std::clamp(v_int + dy, 0, depth_image.rows - 1);
                            int px = std::clamp(u_int + dx, 0, depth_image.cols - 1);
                            float d = depth_image.at<float>(py, px);
                            if (d > 0.15f && d < 2.0f) { // Tabletop workspace depth range
                                valid_depths.push_back(d);
                            }
                        }
                    }

                    if (!valid_depths.empty()) {
                        std::sort(valid_depths.begin(), valid_depths.end());
                        Z_cam = valid_depths[valid_depths.size() / 2];
                    }

                    if (Z_cam > 0.15f) {
                        // Apply Pinhole Camera Back-Projection Formulas
                        double X_cam = ((u - cx) * Z_cam) / fx;
                        double Y_cam = ((v - cy) * Z_cam) / fy;

                        RCLCPP_INFO(this->get_logger(), 
                            "3D Camera Position: X=%.3f m, Y=%.3f m, Z=%.3f m (Pixel u=%.1f, v=%.1f)", 
                            X_cam, Y_cam, Z_cam, u, v);
                        
                        // --------------------------------------------------------------
                        // PHASE 4.3: CAMERA-TO-BASE COORDINATE FRAME TRANSFORMATION
                        // --------------------------------------------------------------
                        try {
                            auto input_point = geometry_msgs::msg::PointStamped();
                            input_point.header.frame_id = "camera_optical_frame";
                            input_point.header.stamp = this->now();
                            
                            input_point.point.x = X_cam;
                            input_point.point.y = Y_cam;
                            input_point.point.z = Z_cam;

                            // Transform 3D point from camera_optical_frame into robot base_link
                            geometry_msgs::msg::PointStamped output_point = tf2_buffer->transform(input_point, "base_link");
                            RCLCPP_INFO(this->get_logger(), 
                                "3D Base Frame Position: X=%.3f m, Y=%.3f m, Z=%.3f m", 
                                output_point.point.x, output_point.point.y, output_point.point.z);
                           
                        // --------------------------------------------------------------
                        // PHASE 4.4: 3D Target Pose Publishing & RViz Marker Visualization
                        // --------------------------------------------------------------      
                            target_obj_pose_pub->publish(output_point);

                            marker_msg.type = visualization_msgs::msg::Marker::SPHERE;

                            marker_msg.header.frame_id = "base_link";
                            marker_msg.header.stamp = this->now();
                            marker_msg.action = visualization_msgs::msg::Marker::ADD;
                            
                            marker_msg.scale.x = 0.05;
                            marker_msg.scale.y = 0.05;
                            marker_msg.scale.z = 0.05;
                            
                            marker_msg.color.r = preset.marker_r;
                            marker_msg.color.g = preset.marker_g;
                            marker_msg.color.b = preset.marker_b;
                            marker_msg.color.a = 1.0;
                            
                            marker_msg.pose.position = output_point.point;
                            marker_pub->publish(marker_msg);  

                        }
                        catch (const tf2::TransformException& e) {
                            RCLCPP_WARN(this->get_logger(), "Transform Exception: %s", e.what());
                            return;
                        }
                    }
                }
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
