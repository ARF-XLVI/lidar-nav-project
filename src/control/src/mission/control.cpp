#include "control_.hpp"
#include "math_.hpp"
#include "geo_.hpp"
#include "drone_controller.hpp"
#include <iostream>
#include <vector>
#include <string>

#define RATE 10.0

float takeoff_altitude;
float forward_distance;
float gate_width;
float centering_tolerance;
float waypoint_tolerance;
std::string control_mode;  
float max_velocity;
float proportional_gain;
float rotation_angle; 
float rotation_radians;

float lateral_tolerance;
float vertical_tolerance;
int min_points_vertical;
int min_points_horizontal;
float max_time;  

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(RATE);
    auto node = std::make_shared<DroneController>();
    
    node->declare_parameter<float>("gate_centering.takeoff_altitude", 2.0);
    node->declare_parameter<float>("gate_centering.forward_distance", 3.0);
    node->declare_parameter<float>("gate_centering.gate_width", 1.5);
    node->declare_parameter<float>("gate_centering.centering_tolerance", 0.15);
    node->declare_parameter<float>("gate_centering.waypoint_tolerance", 0.3);
    node->declare_parameter<std::string>("gate_centering.control_mode", "velocity"); // "position" or "velocity"
    node->declare_parameter<float>("gate_centering.max_velocity", 0.2);  // For velocity control
    node->declare_parameter<float>("gate_centering.proportional_gain", 0.5);  // For velocity control
    node->declare_parameter<float>("gate_centering.rotation_angle", 90.0);  // Rotation in degrees

    node->declare_parameter<float>("gate_centering_3d.lateral_tolerance", 0.10);
    node->declare_parameter<float>("gate_centering_3d.vertical_tolerance", 0.10);
    node->declare_parameter<int>("gate_centering_3d.min_points_vertical", 10);
    node->declare_parameter<int>("gate_centering_3d.min_points_horizontal", 30);
    node->declare_parameter<float>("gate_centering_3d.max_time", 30.0);
    
    node->get_parameter("gate_centering.takeoff_altitude", takeoff_altitude);
    node->get_parameter("gate_centering.forward_distance", forward_distance);
    node->get_parameter("gate_centering.gate_width", gate_width);
    node->get_parameter("gate_centering.centering_tolerance", centering_tolerance);
    node->get_parameter("gate_centering.waypoint_tolerance", waypoint_tolerance);
    node->get_parameter("gate_centering.control_mode", control_mode);
    node->get_parameter("gate_centering.max_velocity", max_velocity);
    node->get_parameter("gate_centering.proportional_gain", proportional_gain);
    node->get_parameter("gate_centering.rotation_angle", rotation_angle);
    rotation_radians = rotation_angle * M_PI / 180.0;
    
    // Get 3D LIDAR gate centering parameters
    node->get_parameter("gate_centering_3d.lateral_tolerance", lateral_tolerance);
    node->get_parameter("gate_centering_3d.vertical_tolerance", vertical_tolerance);
    node->get_parameter("gate_centering_3d.min_points_vertical", min_points_vertical);
    node->get_parameter("gate_centering_3d.min_points_horizontal", min_points_horizontal);
    node->get_parameter("gate_centering_3d.max_time", max_time);
    
    RCLCPP_INFO(node->get_logger(), "=== Centering Basic ===");
    RCLCPP_INFO(node->get_logger(), "Takeoff Alt: %.2f m", takeoff_altitude);
    RCLCPP_INFO(node->get_logger(), "Forward Dist: %.2f m", forward_distance);
    RCLCPP_INFO(node->get_logger(), "Lebar Gate: %.2f m", gate_width);
    RCLCPP_INFO(node->get_logger(), "Centering Tolerance: %.2f m", centering_tolerance);
    RCLCPP_INFO(node->get_logger(), "Control Mode: %s", control_mode.c_str());
    if (control_mode == "velocity") {
        RCLCPP_INFO(node->get_logger(), "Max Velocity: %.2f m/s", max_velocity);
        RCLCPP_INFO(node->get_logger(), "Proportional Gain: %.2f", proportional_gain);
    }

    // MISSION START HERE
    
    geometry_msgs::msg::PoseStamped posee;
    initFrame(node, posee);
    
    while(rclcpp::ok() && node->getCurrentLocalPose().pose.position.z == 0.0) {
        RCLCPP_INFO(node->get_logger(), "Wait local pose data...");
        rclcpp::spin_some(node);
        rate.sleep();
    }
    
    RCLCPP_INFO(node->get_logger(), "Step 1: Taking off %.2f meters...", takeoff_altitude);
    takeoff(node, rate, posee, takeoff_altitude);
    holdPosition(node, rate, posee, 2.0);

    RCLCPP_INFO(node->get_logger(), "Step 2: Centering gate with 3D LIDAR...");
    bool centering_status = false;
    
    centeringGateLidar3D(node, rate, posee, 
                         gate_width, 
                         lateral_tolerance, 
                         vertical_tolerance, 
                         centering_status, 
                         max_velocity, 
                         proportional_gain, 
                         max_time, 
                         min_points_vertical, 
                         min_points_horizontal);

    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
        RCLCPP_INFO(node->get_logger(), "Moving forward");
        executeLocalWaypointMove(node, rate, posee, forward_distance , 0.0, 0.0, 0.0, waypoint_tolerance);
        holdPosition(node, rate, posee, 1.0);
    } else {
        RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Rotating %.2f degrees to the right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, rotation_radians - 30.0);
    rotateByDegrees(node, rate, posee, 90.0);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Moving forward %.2f meters again...", forward_distance);
    executeLocalWaypointMove(node, rate, posee, forward_distance - 1.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Returning to launch position...");
    setMode(node, rate, "land");
    waitLand(node, rate);
    
    RCLCPP_INFO(node->get_logger(), "================= Mission Complete =================");
    
    rclcpp::shutdown();
    return 0;
}
