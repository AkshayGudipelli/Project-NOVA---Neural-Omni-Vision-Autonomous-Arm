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

#include <vector>
#include <cmath>

using namespace std::chrono_literals;

/* ======================================================================================
 * PROJECT NOVA — PHASE 4: 3D PERCEPTION & VISION PROCESSING PIPELINE
 * ======================================================================================
 * Subphase Breakdown:
 *   - Phase 4.1: 2D Color Segmentation & Centroid Extraction (HSV Thresholding)
 *   - Phase 4.2: Depth Map Lookup & Pinhole Back-Projection (2D Pixel + Depth -> 3D Camera Point)
 *   - Phase 4.3: Camera-to-Base Coordinate Frame Transformation (tf2 Listener)
 *   - Phase 4.4: 3D Target Pose Publishing & RViz Marker Visualization
 * ======================================================================================
 */

class ObjectDetectionNode : public rclcpp::Node {

public:
    ObjectDetectionNode() : Node("object_detection_node") {

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

        //Phase 4.4: Publish the target pose and publish 3d Rviz Marker
        target_obj_pose_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/target_object_pose", 10);
        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_data;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_obj_pose_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    cv::Mat depth_image;

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

        // Step 4.1b: Apply Color Threshold Mask for Target Object (Orange)
        cv::Mat mask;
        cv::Scalar lower_bound = cv::Scalar(10, 100, 100);
        cv::Scalar upper_bound = cv::Scalar(25, 255, 255);
        cv::inRange(hsv_img, lower_bound, upper_bound, mask);

        // Step 4.1c: Find Contours & Identify Largest Shape (Filtering Noise)
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0.0;
        int max_idx = -1;

        if (!contours.empty()) {
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }
        }

        // Step 4.1d: Compute 2D Pixel Centroid (u, v) using Image Moments
        if (max_idx != -1 && max_area > 100.0) {
            cv::Moments m = cv::moments(contours[max_idx]);
            if (m.m00 > 0) {
                double u = m.m10 / m.m00; // Pixel Column u
                double v = m.m01 / m.m00; // Pixel Row v
                RCLCPP_INFO(this->get_logger(), "Target Detected at Pixel (u=%.1f, v=%.1f) | Area: %.1f", u, v, max_area);

                // ----------------------------------------------------------------------
                // PHASE 4.2: DEPTH LOOKUP & PINHOLE BACK-PROJECTION (CAMERA FRAME)
                // ----------------------------------------------------------------------
                if (!depth_image.empty()) {
                    float Z_cam = depth_image.at<float>(static_cast<int>(v), static_cast<int>(u));
                    if (Z_cam > 0.0f) {
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
                            
                            marker_msg.color.r = 1.0;
                            marker_msg.color.g = 0.0;
                            marker_msg.color.b = 0.0;
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
