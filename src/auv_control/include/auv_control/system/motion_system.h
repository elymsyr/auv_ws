#ifndef AUV_CONTROL_SYSTEM_MOTION_SYSTEM_H
#define AUV_CONTROL_SYSTEM_MOTION_SYSTEM_H

// ROS 2 Includes
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "auv_control/msg/motion_topic.hpp"
#include "auv_control/msg/mission_topic.hpp"
#include "auv_control/msg/environment_topic.hpp"

// Your project includes for the controller logic
#include "auv_control/control/fossennet.h"

#include <mutex>
#include <memory>

class MotionSystem {
public:
    explicit MotionSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const rclcpp::Logger& logger);

    // Lifecycle methods to be called by the orchestrator
    bool configure();
    bool activate();
    bool deactivate();
    bool cleanup();

private:
    // Callbacks for subscribers
    void mission_callback(const auv_control::msg::MissionTopic::SharedPtr msg);
    void environment_callback(const auv_control::msg::EnvironmentTopic::SharedPtr msg);

    // Callback for the timer - this is the main MPC loop
    void timer_callback();

    // ROS 2 Components
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<auv_control::msg::MotionTopic>::SharedPtr motion_pub_;
    rclcpp::Subscription<auv_control::msg::MissionTopic>::SharedPtr mission_sub_;
    rclcpp::Subscription<auv_control::msg::EnvironmentTopic>::SharedPtr env_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger logger_;

    // Internal MPC object and state
    std::unique_ptr<NonlinearMPC> mpc_;
    auv_control::msg::MissionTopic mission_state_;
    auv_control::msg::EnvironmentTopic env_state_;

    // Mutexes for thread-safe access to shared data
    std::mutex mission_mtx_;
    std::mutex env_mtx_;
};

#endif // AUV_CONTROL_SYSTEM_MOTION_SYSTEM_H