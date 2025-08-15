#include <google/protobuf/message.h>

#include "auv_control/system/environment_system.h"
#include <chrono>
#include <gazebo_msgs/msg/entity_state.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

EnvironmentSystem::EnvironmentSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(node) 
{
    RCLCPP_INFO(node_->get_logger(), "EnvironmentSystem instance created.");
}

bool EnvironmentSystem::configure() {
    RCLCPP_INFO(node_->get_logger(), "Configuring EnvironmentSystem...");

    env_pub_ = node_->create_publisher<auv_control::msg::EnvironmentTopic>("environment_state", 10);

    // --- MODIFICATION: Subscribe to the Gazebo state topic ---
    // Instead of the internal motion_topic, we subscribe to the ground truth from the simulation.
    gazebo_state_sub_ = node_->create_subscription<gazebo_msgs::msg::EntityState>(
        "/ucat/state", 
        10,
        std::bind(&EnvironmentSystem::gazebo_state_callback, this, std::placeholders::_1)
    );

    // The timer is no longer needed, as this node will now be event-driven by the subscriber.
    // timer_ = node_->create_wall_timer(...);
    // timer_->cancel();

    RCLCPP_INFO(node_->get_logger(), "EnvironmentSystem configured successfully.");
    return true;
}

bool EnvironmentSystem::activate() {
    RCLCPP_INFO(node_->get_logger(), "Activating EnvironmentSystem...");
    // No timer to reset.
    return true;
}

bool EnvironmentSystem::deactivate() {
    RCLCPP_INFO(node_->get_logger(), "Deactivating EnvironmentSystem...");
    // No timer to cancel.
    return true;
}

bool EnvironmentSystem::cleanup() {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up EnvironmentSystem...");
    env_pub_.reset();
    gazebo_state_sub_.reset(); // Clean up the new subscriber
    return true;
}

// --- NEW CALLBACK FUNCTION ---
// This function is called every time the Gazebo plugin publishes a new state.
void EnvironmentSystem::gazebo_state_callback(const gazebo_msgs::msg::EntityState::SharedPtr msg) {
    auto env_msg = std::make_unique<auv_control::msg::EnvironmentTopic>();

    // --- POSITION CONVERSION (ENU to NED) ---
    // This part is correct
    env_msg->eta[0] = msg->pose.position.x;
    env_msg->eta[1] = -msg->pose.position.y;
    env_msg->eta[2] = -msg->pose.position.z;

    // --- ORIENTATION CONVERSION (Quaternion to Euler NED) ---
    // Create a tf2 Quaternion object from the message
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    // Create a 3x3 rotation matrix from the quaternion
    tf2::Matrix3x3 m(q);

    // Create variables to hold the Euler angles
    double roll, pitch, yaw;

    // Extract the Euler angles from the rotation matrix
    m.getRPY(roll, pitch, yaw);

    // --- Apply the ENU to NED conversion for angles ---
    env_msg->eta[3] = roll;
    env_msg->eta[4] = -pitch;
    env_msg->eta[5] = -yaw; 

    // --- VELOCITY CONVERSION (ENU to NED) ---
    // You also need to convert linear and angular velocities!
    env_msg->nu[0] = msg->twist.linear.x;
    env_msg->nu[1] = -msg->twist.linear.y;
    env_msg->nu[2] = -msg->twist.linear.z;
    env_msg->nu[3] = msg->twist.angular.x;
    env_msg->nu[4] = -msg->twist.angular.y;
    env_msg->nu[5] = -msg->twist.angular.z;
    
    // Now publish the fully converted message
    env_pub_->publish(std::move(env_msg));
}