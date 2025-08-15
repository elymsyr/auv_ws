#include "auv_control/system/mission_system.h"
#include <array>
#include <vector>

MissionSystem::MissionSystem(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
    : node_(node) {}

bool MissionSystem::configure() {
    RCLCPP_INFO(node_->get_logger(), "Configuring MissionSystem...");

    // Create Publishers
    mission_pub_ = node_->create_publisher<auv_control::msg::MissionTopic>("mission_trajectory", 10);
    signal_pub_ = node_->create_publisher<auv_control::msg::SignalTopic>("signal_status", 10);

    // Create Subscribers
    env_sub_ = node_->create_subscription<auv_control::msg::EnvironmentTopic>(
        "environment_state", 10,
        std::bind(&MissionSystem::environment_callback, this, std::placeholders::_1));
    
    // Subscribe to the sonar data needed by specific missions
    test_sonar_sub_ = node_->create_subscription<auv_control::msg::TestSonarTopic>(
        "test_sonar_topic", 10,
        std::bind(&MissionSystem::test_sonar_callback, this, std::placeholders::_1));

    // Create Timer (original runtime was 100ms)
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MissionSystem::timer_callback, this));
    timer_->cancel(); // Don't start until activated

    // Create Services to control the mission externally
    set_mission_service_ = node_->create_service<auv_control::srv::SetMission>(
        "mission/set", std::bind(&MissionSystem::set_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
    start_mission_service_ = node_->create_service<auv_control::srv::Trigger>(
        "mission/start", std::bind(&MissionSystem::start_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_mission_service_ = node_->create_service<auv_control::srv::Trigger>(
        "mission/stop", std::bind(&MissionSystem::stop_mission_callback, this, std::placeholders::_1, std::placeholders::_2));
    get_status_service_ = node_->create_service<auv_control::srv::GetStatus>(
        "mission/get_status", std::bind(&MissionSystem::get_status_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(node_->get_logger(), "MissionSystem configured successfully.");
    
    pause_client_ = node_->create_client<std_srvs::srv::Empty>("/pause_physics");
    unpause_client_ = node_->create_client<std_srvs::srv::Empty>("/unpause_physics");

    RCLCPP_INFO(node_->get_logger(), "MissionSystem configured successfully.");
    return true;
}

bool MissionSystem::activate() {
    RCLCPP_INFO(node_->get_logger(), "Activating MissionSystem...");
    timer_->reset(); // Start the main loop
    return true;
}

bool MissionSystem::deactivate() {
    RCLCPP_INFO(node_->get_logger(), "Deactivating MissionSystem...");
    timer_->cancel(); // Stop the main loop
    return true;
}

bool MissionSystem::cleanup() {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up MissionSystem...");
    // Destroy all ROS 2 components by resetting their smart pointers
    mission_pub_.reset();
    signal_pub_.reset();
    env_sub_.reset();
    test_sonar_sub_.reset();
    timer_.reset();
    set_mission_service_.reset();
    start_mission_service_.reset();
    stop_mission_service_.reset();
    get_status_service_.reset();
    active_mission_.reset(); // Destroy the mission object
    return true;
}

// --- Subscription Callbacks ---

void MissionSystem::environment_callback(const auv_control::msg::EnvironmentTopic::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(env_mtx_);
    env_state_ = *msg;
}

void MissionSystem::test_sonar_callback(const auv_control::msg::TestSonarTopic::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mission_control_mtx_);
    // Only proceed if a mission is active
    if (active_mission_ && mission_running_) {
        // Use dynamic_cast to check if the active mission is a SonarMission
        if (auto sonar_mission = dynamic_cast<SonarMission*>(active_mission_.get())) {
            // If it is, pass the sonar data to it
            sonar_mission->update_sonar_data(msg);
        }
    }
}

// --- Main Loop ---

void MissionSystem::timer_callback() {
    // Get a thread-safe copy of the current environment state
    auv_control::msg::EnvironmentTopic current_env_state;
    {
        std::lock_guard<std::mutex> lock(env_mtx_);
        current_env_state = env_state_;
    }
    
    // Convert the ROS message to the C-style array the mission logic expects
    std::array<double, 12> current_state_array;
    std::copy(std::begin(current_env_state.eta), std::end(current_env_state.eta), std::begin(current_state_array));
    std::copy(std::begin(current_env_state.nu), std::end(current_env_state.nu), std::begin(current_state_array) + 6);
    
    std::array<std::array<double, 12>, HORIZON> ref_path;

    // Execute the active mission's logic in a thread-safe manner
    {
        std::lock_guard<std::mutex> lock(mission_control_mtx_);
        if (mission_running_ && active_mission_) {
            ref_path = active_mission_->step(current_state_array);
        } else {
            // Default behavior: hold current position and a default depth
            for (size_t i = 0; i < HORIZON; i++) {
                ref_path[i] = current_state_array;
                ref_path[i][2] = -1.0; // Desired depth of -1m
            }
        }
    }
    
    // Create and publish the resulting trajectory
    auto mission_msg = std::make_unique<auv_control::msg::MissionTopic>();
    array_to_ros_msg(ref_path, *mission_msg);
    mission_pub_->publish(std::move(mission_msg));
    
    // Publish signal status (can be expanded later)
    auto signal_msg = std::make_unique<auv_control::msg::SignalTopic>();
    signal_pub_->publish(std::move(signal_msg));
}

// --- Service Implementations ---

void MissionSystem::set_mission_callback(const std::shared_ptr<auv_control::srv::SetMission::Request> request,
                                         std::shared_ptr<auv_control::srv::SetMission::Response> response) {
    std::lock_guard<std::mutex> lock(mission_control_mtx_);
    if (mission_running_) {
        active_mission_->terminate();
        mission_running_ = false;
    }

    // Use a switch on the enum from mission_imp.h for clarity
    switch(static_cast<MissionIMP>(request->mission_type)) {
        case MissionIMP::SonarMisTest:
            active_mission_ = std::make_unique<SonarMission>();
            active_mission_->initialize();
            response->success = true;
            response->message = "SonarMisTest mission has been set.";
            RCLCPP_INFO(node_->get_logger(), "Mission set to: SonarMisTest");
            break;
        default:
            active_mission_ = nullptr;
            response->success = false;
            response->message = "Unknown or unsupported mission type.";
            RCLCPP_WARN(node_->get_logger(), "Attempted to set an unknown mission type: %d", request->mission_type);
            break;
    }
}

void MissionSystem::start_mission_callback(const std::shared_ptr<auv_control::srv::Trigger::Request>,
                                           std::shared_ptr<auv_control::srv::Trigger::Response> response) {
    call_gazebo_pause_service(false);
    std::lock_guard<std::mutex> lock(mission_control_mtx_);
    if (active_mission_) {
        mission_running_ = true;
        response->success = true;
        response->message = "Mission started.";
        RCLCPP_INFO(node_->get_logger(), "Mission started.");
    } else {
        response->success = false;
        response->message = "Cannot start, no mission has been set.";
        RCLCPP_WARN(node_->get_logger(), "Attempted to start mission, but none was set.");
    }
}

void MissionSystem::stop_mission_callback(const std::shared_ptr<auv_control::srv::Trigger::Request>,
                                          std::shared_ptr<auv_control::srv::Trigger::Response> response) {
    call_gazebo_pause_service(true);
    std::lock_guard<std::mutex> lock(mission_control_mtx_);
    if (mission_running_) {
        active_mission_->terminate();
        mission_running_ = false;
        response->success = true;
        response->message = "Mission stopped.";
        RCLCPP_INFO(node_->get_logger(), "Mission stopped.");
    } else {
        response->success = false;
        response->message = "Cannot stop, no mission is currently running.";
    }
}

void MissionSystem::get_status_callback(const std::shared_ptr<auv_control::srv::GetStatus::Request>,
                                        std::shared_ptr<auv_control::srv::GetStatus::Response> response) {
    std::lock_guard<std::mutex> lock(mission_control_mtx_);
    response->is_running = mission_running_;
    if (active_mission_) {
        response->mission_name = active_mission_->getMissionName();
    } else {
        response->mission_name = "No active mission";
    }
}


// --- Helper Function ---
void MissionSystem::array_to_ros_msg(const std::array<std::array<double, 12>, HORIZON>& ref_path, 
                                     auv_control::msg::MissionTopic& msg) {
    for (size_t i = 0; i < HORIZON; ++i) {
        std::copy(ref_path[i].begin(), ref_path[i].begin() + 6, msg.trajectory[i].eta_desired.begin());
        std::copy(ref_path[i].begin() + 6, ref_path[i].end(), msg.trajectory[i].nu_desired.begin());
    }
}

void MissionSystem::call_gazebo_pause_service(bool pause) {
    auto client = pause ? pause_client_ : unpause_client_;
    auto service_name = pause ? "/pause_physics" : "/unpause_physics";

    // Wait for the service to be available
    if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Service '%s' not available.", service_name);
        return;
    }

    // Create a request (it's empty for this service type)
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    // Asynchronously call the service
    // We use a lambda to log the result when it comes back
    client->async_send_request(request, 
        [this, service_name](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            // This part runs when the service call is complete
            if (future.valid()) {
                RCLCPP_INFO(node_->get_logger(), "Service call '%s' successful.", service_name);
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Failed to call service '%s'.", service_name);
            }
        });
}