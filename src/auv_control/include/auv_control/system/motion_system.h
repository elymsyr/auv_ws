#ifndef MOTION_SYSTEM_H
#define MOTION_SYSTEM_H

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "auv_control/msg/mission_topic.hpp"
#include "auv_control/msg/environment_topic.hpp"
// REMOVED old motion topic message
// #include "auv_control/msg/motion_topic.hpp"
// ADD the standard message for thruster commands
#include "std_msgs/msg/float32_multi_array.hpp"

#include "auv_control/control/fossennet.h" // Assuming this is your MPC class
#include <mutex>

class MotionSystem {
public:
    MotionSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const rclcpp::Logger& logger);
    bool configure();
    bool activate();
    bool deactivate();
    bool cleanup();

private:
    void mission_callback(const auv_control::msg::MissionTopic::SharedPtr msg);
    void environment_callback(const auv_control::msg::EnvironmentTopic::SharedPtr msg);
    void timer_callback();

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Logger logger_;
    
    // CHANGE the publisher declaration to match the .cpp file
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_pub_;
    
    rclcpp::Subscription<auv_control::msg::MissionTopic>::SharedPtr mission_sub_;
    rclcpp::Subscription<auv_control::msg::EnvironmentTopic>::SharedPtr env_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mission_mtx_;
    auv_control::msg::MissionTopic mission_state_;
    std::mutex env_mtx_;
    auv_control::msg::EnvironmentTopic env_state_;

    std::unique_ptr<NonlinearMPC> mpc_;
};

#endif // MOTION_SYSTEM_H