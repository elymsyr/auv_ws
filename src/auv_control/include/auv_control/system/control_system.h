#ifndef AUV_CONTROL_SYSTEM_CONTROL_SYSTEM_H
#define AUV_CONTROL_SYSTEM_CONTROL_SYSTEM_H

// ROS 2 Includes for lifecycle nodes and messages
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "auv_control/msg/motion_topic.hpp"
#include "auv_control/msg/signal_topic.hpp"

#include <mutex>
#include <memory>

class ControlSystem {
public:
    // The constructor now takes a pointer to the main ROS 2 node.
    // This allows it to create its own publishers and subscribers.
    explicit ControlSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    ~ControlSystem() = default;

    // These methods will be called by the main orchestrator node during its lifecycle transitions.
    bool configure(); // Replaces the old init_()
    bool cleanup();   // Replaces the old halt()

    // --- Thread-Safe Data Access ---
    // Provides a safe way for other parts of the system to get the latest data.
    auv_control::msg::MotionTopic get_motion_state();
    auv_control::msg::SignalTopic get_signal_state();

private:
    // --- ROS 2 Callbacks ---
    // These functions are automatically called by the ROS 2 executor when a message arrives.
    void motion_topic_callback(const auv_control::msg::MotionTopic::SharedPtr msg);
    void signal_topic_callback(const auv_control::msg::SignalTopic::SharedPtr msg);

    // Pointer to the ROS 2 node that owns this class instance
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    // ROS 2 native subscribers
    rclcpp::Subscription<auv_control::msg::MotionTopic>::SharedPtr motion_sub_;
    rclcpp::Subscription<auv_control::msg::SignalTopic>::SharedPtr signal_sub_;
    
    // --- Member Data and Mutexes ---
    // The class still owns its data and the mutexes to protect it.
    auv_control::msg::MotionTopic motion_state_;
    auv_control::msg::SignalTopic signal_state_;

    // Mutexes are now suffixed with _ for clarity as private members
    std::mutex motion_mtx_;
    std::mutex signal_mtx_;
};

#endif // AUV_CONTROL_SYSTEM_CONTROL_SYSTEM_H