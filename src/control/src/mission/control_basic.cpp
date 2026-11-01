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
    
    RCLCPP_INFO(node->get_logger(), "Wait lidar data...");
    while(rclcpp::ok() && node->getCurrentLaserScan().ranges.empty()) {
        RCLCPP_INFO(node->get_logger(), "Waiting lidar scan...");
        rclcpp::spin_some(node);
        rate.sleep();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    RCLCPP_INFO(node->get_logger(), "Lidar data received!");
    
    RCLCPP_INFO(node->get_logger(), "Step 1: Taking off %.2f meters...", takeoff_altitude);
    takeoff(node, rate, posee, takeoff_altitude);
    holdPosition(node, rate, posee, 2.0);
    
    // Step 2: Move forward to approach gate
    // RCLCPP_INFO(node->get_logger(), "Step 2: Moving forward %.2f meters to approach gate...", forward_distance);
    // executeLocalWaypointMove(node, rate, posee, forward_distance, 0.0, 0.0, 0.0, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 3: Centering to gate lidar %s control...", control_mode.c_str());
    bool centering_status = false;
    
    if (control_mode == "velocity") {
        centeringGateLidarVelocity(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0);
    } else if (control_mode == "position") {
        centeringGateLidar(node, rate, posee, gate_width, centering_tolerance, centering_status, 30.0);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Invalid control_mode: %s. Use 'position' or 'velocity'", control_mode.c_str());
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 1;
    }

    // centeringGateNear(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0, 0.2, 0.785);  
    
    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
    } else {
        RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 0;
    }
    
    holdPosition(node, rate, posee, 2.0);
    
    RCLCPP_INFO(node->get_logger(), "Step 4: Moving forward through the gate...");
    executeLocalWaypointMove(node, rate, posee, forward_distance + 8.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    strafeRight(node, rate, posee, 1.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    // centeringGateLidarVelocity(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0);

    centeringGateNear(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0, 0.2, 0.785);
    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
        RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>> Step 4: Moving forward through the gate... <<<<<<<<<<<<");
        executeLocalWaypointMove(node, rate, posee, forward_distance + 7.0, 0.0, 0.0, 0.0, waypoint_tolerance);
        holdPosition(node, rate, posee, 1.0);
    } else {
        RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 5: Moving forward %.2f meters again...", forward_distance);
    executeLocalWaypointMove(node, rate, posee, forward_distance + 7.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 5: Moving forward %.2f meters again...", forward_distance);
    executeLocalWaypointMove(node, rate, posee, forward_distance + 2.0, 0.0, 0.0, 0.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);


    // RCLCPP_INFO(node->get_logger(), "Step 3: Strafing left 2.0 meters...");
    // strafeLeft(node, rate, posee, 0.5, waypoint_tolerance);
    // holdPosition(node, rate, posee, 1.0);

    // centeringGateLidarVelocity(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0);
    
    centeringGateNear(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0, 0.2, 0.785);  // forward_zone_min=0.2m, max_lateral_angle=45°
    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
        RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>> Step 5: Moving forward through the gate... <<<<<<<<<<<<");
        executeLocalWaypointMove(node, rate, posee, forward_distance + 14.5, 0.0, 0.0, 0.0, waypoint_tolerance);
        holdPosition(node, rate, posee, 1.0);
    } else {
        RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, rotation_radians - 30.0);
    rotateByDegrees(node, rate, posee, -30.0);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    strafeLeft(node, rate, posee, 0.3, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    centeringGateLidarVelocity(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0);
    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
        RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>> Step 5: Moving forward through the gate... <<<<<<<<<<<<");
        executeLocalWaypointMove(node, rate, posee, forward_distance + 2.0, 0.0, 0.0, 0.0, waypoint_tolerance);
        holdPosition(node, rate, posee, 1.0);
    } else {
        RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    rotateByDegrees(node, rate, posee, -55.0);
    holdPosition(node, rate, posee, 1.0);

    // RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    // holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 5: Moving forward %.2f meters again...", forward_distance);
    executeLocalWaypointMove(node, rate, posee, forward_distance + 5.3, 0.0, 0.0, 0.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    rotateByDegrees(node, rate, posee, -90.0);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 5: Moving forward %.2f meters again...", forward_distance);
    executeLocalWaypointMove(node, rate, posee, forward_distance - 1.8, 0.0, 0.0, 0.0, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 3: Strafing right 2.0 meters...");
    strafeRight(node, rate, posee, 0.6, waypoint_tolerance);
    holdPosition(node, rate, posee, 1.0);

    centeringGateLidarVelocity(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0);
    centeringGateNear(node, rate, posee, gate_width, centering_tolerance, centering_status, max_velocity, proportional_gain, 30.0, 0.2, 0.785);
    if (centering_status) {
        RCLCPP_INFO(node->get_logger(), "Gate centering successful!");
        RCLCPP_INFO(node->get_logger(), ">>>>>>>>>>>> Step 5: Moving forward through the gate... <<<<<<<<<<<<");
        executeLocalWaypointMove(node, rate, posee, forward_distance + 2.0, 0.0, 0.0, 0.0, waypoint_tolerance);
        holdPosition(node, rate, posee, 1.0);
        setMode(node, rate, "rtl");
    } else {
        RCLCPP_WARN(node->get_logger(), "Gate centering failed or timeout!");
        setMode(node, rate, "rtl");
        waitLand(node, rate);
        rclcpp::shutdown();
        return 0;
    }

    // RCLCPP_INFO(node->get_logger(), "Step 4: Rotating %.2f degrees to the right...", rotation_angle);
    // executeLocalWaypointRotation(node, rate, posee, rotation_radians);
    // holdPosition(node, rate, posee, 1.0);

    RCLCPP_INFO(node->get_logger(), "Step 5: Returning to launch position...");
    setMode(node, rate, "rtl");
    waitLand(node, rate);
    
    RCLCPP_INFO(node->get_logger(), "================= Mission Complete =================");
    
    rclcpp::shutdown();
    return 0;
}
