#include "auv_control/system/control_system.h"

// The constructor simply stores the node handle for later use.
ControlSystem::ControlSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(node) {}

// configure() is called by the main orchestrator during its on_configure transition.
bool ControlSystem::configure() {
    // Log using the main node's logger
    RCLCPP_INFO(node_->get_logger(), "Configuring ControlSystem...");

    // Create a subscription to the "motion_topic".
    // When a message arrives, the 'motion_topic_callback' method will be called.
    // The topic name "motion_topic" is an example; you can choose what you like.
    motion_sub_ = node_->create_subscription<auv_control::msg::MotionTopic>(
        "motion_topic", 
        10, // QoS history depth
        std::bind(&ControlSystem::motion_topic_callback, this, std::placeholders::_1)
    );

    // Create a subscription to the "signal_topic".
    signal_sub_ = node_->create_subscription<auv_control::msg::SignalTopic>(
        "signal_topic", 
        10, 
        std::bind(&ControlSystem::signal_topic_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(node_->get_logger(), "ControlSystem configured successfully.");
    return true;
}

// cleanup() is called by the main orchestrator during its on_cleanup transition.
bool ControlSystem::cleanup() {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up ControlSystem...");

    // Resetting the smart pointers is enough to destroy the subscribers.
    motion_sub_.reset();
    signal_sub_.reset();
    
    RCLCPP_INFO(node_->get_logger(), "ControlSystem cleaned up successfully.");
    return true;
}

// --- Callback Implementations ---

void ControlSystem::motion_topic_callback(const auv_control::msg::MotionTopic::SharedPtr msg) {
    // This function is the entire "receiver_loop" for the motion topic.
    // Lock the mutex to ensure thread-safe writing.
    std::lock_guard<std::mutex> lock(motion_mtx_);
    motion_state_ = *msg;
}

void ControlSystem::signal_topic_callback(const auv_control::msg::SignalTopic::SharedPtr msg) {
    // Lock the mutex to ensure thread-safe writing.
    std::lock_guard<std::mutex> lock(signal_mtx_);
    signal_state_ = *msg;
}


// --- Thread-Safe Data Accessor Methods ---

auv_control::msg::MotionTopic ControlSystem::get_motion_state() {
    std::lock_guard<std::mutex> lock(motion_mtx_);
    return motion_state_;
}

auv_control::msg::SignalTopic ControlSystem::get_signal_state() {
    std::lock_guard<std::mutex> lock(signal_mtx_);
    return signal_state_;
}