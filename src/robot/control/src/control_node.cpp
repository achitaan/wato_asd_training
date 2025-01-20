#include "control_node.hpp"
#include <chrono>
#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <optional>

using namespace std;


ControlNode::ControlNode() : Node("control_node") {
    // Initialize parameters
    lookahead_distance_ = 1.2;  // Lookahead distance
    linear_speed_ = 0.5;       // Constant forward speed

    // Subscribers and Publishers
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

    odometry_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]() { controlLoop(); });
}

void ControlNode::controlLoop() {
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
        return;
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        
        RCLCPP_WARN(this->get_logger(), "No valid lookahead point found.");
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
        return;
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Publish the velocity command
    cmd_vel_publisher_->publish(cmd_vel);

    RCLCPP_INFO(this->get_logger(), "Found lookahead point at: (%.2f, %.2f)",
                lookahead_point->pose.position.x,
                lookahead_point->pose.position.y);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty()) {
        return std::nullopt;
    }

    const auto &robot_position = robot_odom_->pose.pose.position;

    if (computeDistance(robot_position, current_path_->poses.back().pose.position) < GOAL_TOLERANCE) {
      return std::nullopt;
    }

    for (const auto &pose : current_path_->poses) {
      double distance = computeDistance(robot_position, pose.pose.position);
      if (distance >= lookahead_distance_) {
        return pose;
      }
    }

    return std::nullopt;
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    geometry_msgs::msg::Twist cmd_vel;

    const auto &robot_position = robot_odom_->pose.pose.position;
    const auto &target_position = target.pose.position;

    // Calculate the heading angle to the target
    double heading_angle = std::atan2(target_position.y - robot_position.y,
                                      target_position.x - robot_position.x);

    // Get robot's current orientation
    double robot_yaw = extractYaw(robot_odom_->pose.pose.orientation);

    // Calculate heading error
    double angular_velocity = heading_angle - robot_yaw;

    // Normalize heading error to [-pi, pi]
    while (angular_velocity > M_PI) angular_velocity -= 2.0 * M_PI;
    while (angular_velocity < -M_PI) angular_velocity += 2.0 * M_PI;

    // Generate velocity commands
    cmd_vel.linear.x = std::sqrt(
      std::pow(robot_position.x - target_position.x, 2) + std::pow(robot_position.y - target_position.y, 2)
    );  // Constant forward speed
    cmd_vel.angular.z = 3.0 * angular_velocity;  // Proportional control for angular velocity

    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::sqrt(
        std::pow(a.x - b.x, 2) +
        std::pow(a.y - b.y, 2) +
        std::pow(a.z - b.z, 2)
    );
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    return yaw;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
