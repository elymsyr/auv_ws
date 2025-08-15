#ifndef ENVIRONMENT_SYSTEM_H
#define ENVIRONMENT_SYSTEM_H

#include <google/protobuf/message.h>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "auv_control/msg/environment_topic.hpp"
#include "auv_control/msg/motion_topic.hpp"
// ADD THIS for the Gazebo message
#include "gazebo_msgs/msg/entity_state.hpp"
#include <mutex>

class EnvironmentSystem {
public:
    EnvironmentSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node);
    bool configure();
    bool activate();
    bool deactivate();
    bool cleanup();

private:
    // REMOVED old motion_topic_callback and timer_callback
    // void motion_topic_callback(const auv_control::msg::MotionTopic::SharedPtr msg);
    // void timer_callback();
    
    // ADD the new callback function for the Gazebo state
    void gazebo_state_callback(const gazebo_msgs::msg::EntityState::SharedPtr msg);

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<auv_control::msg::EnvironmentTopic>::SharedPtr env_pub_;
    
    // REMOVED old subscriber and timer
    // rclcpp::Subscription<auv_control::msg::MotionTopic>::SharedPtr motion_sub_;
    // rclcpp::TimerBase::SharedPtr timer_;
    
    // ADD the new subscriber member variable
    rclcpp::Subscription<gazebo_msgs::msg::EntityState>::SharedPtr gazebo_state_sub_;
    
    // REMOVED old state variables that are no longer needed
    // std::mutex motion_mtx_;
    // auv_control::msg::MotionTopic motion_state_;
};

#endif // ENVIRONMENT_SYSTEM_H