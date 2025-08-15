#include "auv_control/system/motion_system.h"
// ADD THIS INCLUDE for the standard message type
#include "std_msgs/msg/float32_multi_array.hpp"

MotionSystem::MotionSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const rclcpp::Logger& logger)
    : node_(node), logger_(logger)
{
    mpc_ = std::make_unique<NonlinearMPC>(logger_);
    RCLCPP_INFO(node_->get_logger(), "MotionSystem instance created.");
}

bool MotionSystem::configure() {
    RCLCPP_INFO(node_->get_logger(), "Configuring MotionSystem...");
    
    try {
        mpc_->initialization();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(node_->get_logger(), "MPC initialization failed: %s", e.what());
        return false;
    }
    
    // --- MODIFICATION: Publish the thruster command for Gazebo ---
    // Change publisher type to std_msgs::msg::Float32MultiArray
    // Change topic name to /ucat/thruster_cmd
    thruster_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("/ucat/thruster_cmd", 10);

    mission_sub_ = node_->create_subscription<auv_control::msg::MissionTopic>(
        "mission_trajectory", 10,
        std::bind(&MotionSystem::mission_callback, this, std::placeholders::_1));

    env_sub_ = node_->create_subscription<auv_control::msg::EnvironmentTopic>(
        "environment_state", 10,
        std::bind(&MotionSystem::environment_callback, this, std::placeholders::_1));

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MotionSystem::timer_callback, this));
    timer_->cancel();

    RCLCPP_INFO(node_->get_logger(), "MotionSystem configured successfully.");
    return true;
}

bool MotionSystem::activate() {
    RCLCPP_INFO(node_->get_logger(), "Activating MotionSystem...");
    timer_->reset(); // Start the control loop timer.
    return true;
}

bool MotionSystem::deactivate() {
    RCLCPP_INFO(node_->get_logger(), "Deactivating MotionSystem...");
    timer_->cancel(); // Stop the control loop timer.
    return true;
}

bool MotionSystem::cleanup() {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up MotionSystem...");
    thruster_pub_.reset(); // Clean up the new publisher
    mission_sub_.reset();
    env_sub_.reset();
    timer_.reset();
    return true;
}

// --- Subscriber Callbacks ---

void MotionSystem::mission_callback(const auv_control::msg::MissionTopic::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mission_mtx_);
    mission_state_ = *msg;
}

void MotionSystem::environment_callback(const auv_control::msg::EnvironmentTopic::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(env_mtx_);
    env_state_ = *msg;
}

// --- Main Control Loop (Timer Callback) ---

void MotionSystem::timer_callback() {
    auv_control::msg::MissionTopic local_mission;
    auv_control::msg::EnvironmentTopic local_env;
    {
        std::lock_guard<std::mutex> lock_mission(mission_mtx_);
        local_mission = mission_state_;
    }
    {
        std::lock_guard<std::mutex> lock_env(env_mtx_);
        local_env = env_state_;
    }
    
    try {
        std::array<double, 8> propeller_commands = mpc_->solve(local_env, local_mission);
        
        // --- MODIFICATION: Create and populate the thruster message ---
        auto thruster_msg = std::make_unique<std_msgs::msg::Float32MultiArray>();
        
        // Copy the 8 thruster commands into the message data field
        thruster_msg->data.assign(propeller_commands.begin(), propeller_commands.end());
        
        // Publish the thruster command message
        thruster_pub_->publish(std::move(thruster_msg));

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "MotionSystem MPC solve error: %s", e.what());
    }
}