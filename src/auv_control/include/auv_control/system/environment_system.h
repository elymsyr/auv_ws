#ifndef AUV_CONTROL_SYSTEM_ENVIRONMENT_SYSTEM_H
#define AUV_CONTROL_SYSTEM_ENVIRONMENT_SYSTEM_H

// ROS 2 Includes
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "auv_control/msg/environment_topic.hpp"
#include "auv_control/msg/motion_topic.hpp"

#include <mutex>
#include <memory>

class EnvironmentSystem {
public:
    explicit EnvironmentSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    ~EnvironmentSystem() = default;

    // Lifecycle management methods called by the orchestrator
    bool configure();
    bool activate();
    bool deactivate();
    bool cleanup();

private:
    // Callback for the motion topic subscription
    void motion_topic_callback(const auv_control::msg::MotionTopic::SharedPtr msg);

    // Callback for the timer, this is our main processing loop
    void timer_callback();

    // Pointer to the main ROS 2 node
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    // ROS 2 publisher, subscriber, and timer
    rclcpp::Publisher<auv_control::msg::EnvironmentTopic>::SharedPtr env_pub_;
    rclcpp::Subscription<auv_control::msg::MotionTopic>::SharedPtr motion_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Member data and mutex for thread-safe data exchange
    auv_control::msg::MotionTopic motion_state_;
    std::mutex motion_mtx_;
};

#endif // AUV_CONTROL_SYSTEM_ENVIRONMENT_SYSTEM_H