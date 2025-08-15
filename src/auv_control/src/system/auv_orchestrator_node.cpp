#include "auv_control/system/auv_orchestrator_node.hpp"

AuvOrchestratorNode::AuvOrchestratorNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("auv_orchestrator", options) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuvOrchestratorNode::on_configure(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Orchestrator is configuring...");
    auto self = this->shared_from_this();
    control_system_ = std::make_unique<ControlSystem>(self);
    environment_system_ = std::make_unique<EnvironmentSystem>(self);
    mission_system_ = std::make_unique<MissionSystem>(self);
    motion_system_ = std::make_unique<MotionSystem>(shared_from_this(), this->get_logger());

    if (!control_system_->configure() || !environment_system_->configure() ||
        !mission_system_->configure() || !motion_system_->configure()) {
        RCLCPP_FATAL(this->get_logger(), "A subsystem failed to configure. Entering unconfigured state.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "All subsystems configured successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuvOrchestratorNode::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Orchestrator is activating...");
    if (!environment_system_->activate() || !mission_system_->activate() || !motion_system_->activate()) {
        RCLCPP_ERROR(this->get_logger(), "A subsystem failed to activate.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(this->get_logger(), "All subsystems activated successfully.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuvOrchestratorNode::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Orchestrator is deactivating...");
    if (!environment_system_->deactivate() || !mission_system_->deactivate() || !motion_system_->deactivate()) {
        RCLCPP_ERROR(this->get_logger(), "A subsystem failed to deactivate.");
    }
    RCLCPP_INFO(this->get_logger(), "All subsystems deactivated.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AuvOrchestratorNode::on_cleanup(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(this->get_logger(), "Orchestrator is cleaning up...");
    motion_system_->cleanup();
    mission_system_->cleanup();
    environment_system_->cleanup();
    control_system_->cleanup();
    motion_system_.reset();
    mission_system_.reset();
    environment_system_.reset();
    control_system_.reset();
    RCLCPP_INFO(this->get_logger(), "All subsystems cleaned up and destroyed.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// Add this to the bottom of auv_orchestrator_node.cpp

#include "auv_control/system/auv_orchestrator_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  
  // Use a MultiThreadedExecutor to allow all callbacks to run in parallel
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::NodeOptions options;
  
  auto orchestrator_node = std::make_shared<AuvOrchestratorNode>(options);
  
  executor.add_node(orchestrator_node->get_node_base_interface());
  executor.spin(); // This will keep the node alive
  
  rclcpp::shutdown();
  return 0;
}