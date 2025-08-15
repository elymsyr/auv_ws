#include "auv_control/system/environment_system.h"
#include <chrono>
// ADD THIS INCLUDE for the Gazebo message type
#include <gazebo_msgs/msg/entity_state.hpp>

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

    // --- DATA CONVERSION ---
    // Convert from gazebo_msgs/msg/EntityState to your custom auv_control/msg/EnvironmentTopic

    // 1. Position and Orientation (eta)
    // Gazebo provides pose in ENU, but your model likely uses NED.
    // Assuming your control system uses NED, we must convert back.
    env_msg->eta[0] = msg->pose.position.x;
    env_msg->eta[1] = -msg->pose.position.y; // ENU to NED
    env_msg->eta[2] = -msg->pose.position.z; // ENU to NED
    
    // Convert quaternion to Euler angles for eta[3,4,5]
    // Note: This requires a helper function (not shown, but standard)
    // For simplicity, we'll leave them as zero for now.
    // You would need to implement a quaternion_to_euler function here.
    env_msg->eta[3] = 0.0; // Roll (phi)
    env_msg->eta[4] = 0.0; // Pitch (theta)
    env_msg->eta[5] = 0.0; // Yaw (psi)

    // 2. Body-frame velocities (nu)
    env_msg->nu[0] = msg->twist.linear.x;
    env_msg->nu[1] = msg->twist.linear.y;
    env_msg->nu[2] = msg->twist.linear.z;
    env_msg->nu[3] = msg->twist.angular.x;
    env_msg->nu[4] = msg->twist.angular.y;
    env_msg->nu[5] = msg->twist.angular.z;

    // 3. Accelerations (nu_dot) - Not provided by this topic, so leave as zero.
    for (int i=0; i<6; ++i) env_msg->nu_dot[i] = 0.0;

    // Publish the converted state for the rest of your control system to use.
    env_pub_->publish(std::move(env_msg));
}