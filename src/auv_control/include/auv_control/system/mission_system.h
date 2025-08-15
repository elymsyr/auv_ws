#ifndef AUV_CONTROL_SYSTEM_MISSION_SYSTEM_H
#define AUV_CONTROL_SYSTEM_MISSION_SYSTEM_H

// ROS 2 Includes for lifecycle nodes, messages, and services
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "auv_control/msg/mission_topic.hpp"
#include "auv_control/msg/signal_topic.hpp"
#include "auv_control/msg/environment_topic.hpp"
#include "auv_control/msg/test_sonar_topic.hpp"
#include "auv_control/srv/set_mission.hpp"
#include "auv_control/srv/trigger.hpp"
#include "auv_control/srv/get_status.hpp"

// Your project's pure C++ mission logic includes
#include "auv_control/mission/mission.h"
#include "auv_control/mission/mission_imp.h"
#include "auv_control/mission/mission_sonar_imp.h" // For instantiating the specific mission

#include <mutex>
#include <memory>

class MissionSystem {
public:
    // Constructor takes a handle to the parent orchestrator node
    explicit MissionSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

    // Lifecycle methods called by the orchestrator
    bool configure();
    bool activate();
    bool deactivate();
    bool cleanup();

private:
    // --- Callbacks for ROS 2 Components ---

    // For subscriptions
    void environment_callback(const auv_control::msg::EnvironmentTopic::SharedPtr msg);
    void test_sonar_callback(const auv_control::msg::TestSonarTopic::SharedPtr msg);
    
    // For the periodic timer (main logic loop)
    void timer_callback();

    // For services (external control)
    void set_mission_callback(const std::shared_ptr<auv_control::srv::SetMission::Request> request,
                              std::shared_ptr<auv_control::srv::SetMission::Response> response);
    void start_mission_callback(const std::shared_ptr<auv_control::srv::Trigger::Request> request,
                                std::shared_ptr<auv_control::srv::Trigger::Response> response);
    void stop_mission_callback(const std::shared_ptr<auv_control::srv::Trigger::Request> request,
                               std::shared_ptr<auv_control::srv::Trigger::Response> response);
    void get_status_callback(const std::shared_ptr<auv_control::srv::GetStatus::Request> request,
                               std::shared_ptr<auv_control::srv::GetStatus::Response> response);
    
    // --- Helper Function ---
    void array_to_ros_msg(const std::array<std::array<double, 12>, HORIZON>& ref_path, 
                          auv_control::msg::MissionTopic& msg);

    // --- ROS 2 Component Members ---
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Publisher<auv_control::msg::MissionTopic>::SharedPtr mission_pub_;
    rclcpp::Publisher<auv_control::msg::SignalTopic>::SharedPtr signal_pub_;
    rclcpp::Subscription<auv_control::msg::EnvironmentTopic>::SharedPtr env_sub_;
    rclcpp::Subscription<auv_control::msg::TestSonarTopic>::SharedPtr test_sonar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<auv_control::srv::SetMission>::SharedPtr set_mission_service_;
    rclcpp::Service<auv_control::srv::Trigger>::SharedPtr start_mission_service_;
    rclcpp::Service<auv_control::srv::Trigger>::SharedPtr stop_mission_service_;
    rclcpp::Service<auv_control::srv::GetStatus>::SharedPtr get_status_service_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr unpause_client_;
    void call_gazebo_pause_service(bool pause);

    // --- Internal State & Mission Logic ---
    auv_control::msg::EnvironmentTopic env_state_;
    std::unique_ptr<Mission> active_mission_; // Polymorphic pointer to the current mission object
    bool mission_running_ = false;

    // --- Thread Safety ---
    std::mutex env_mtx_;             // Protects env_state_
    std::mutex mission_control_mtx_; // Protects active_mission_ and mission_running_
};

#endif // AUV_CONTROL_SYSTEM_MISSION_SYSTEM_H