#ifndef AUV_ORCHESTRATOR_NODE_HPP_
#define AUV_ORCHESTRATOR_NODE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "auv_control/system/control_system.h"
#include "auv_control/system/environment_system.h"
#include "auv_control/system/mission_system.h"
#include "auv_control/system/motion_system.h"
#include <memory>

class AuvOrchestratorNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit AuvOrchestratorNode(const rclcpp::NodeOptions & options);

protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & previous_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

private:
    std::unique_ptr<ControlSystem> control_system_;
    std::unique_ptr<EnvironmentSystem> environment_system_;
    std::unique_ptr<MissionSystem> mission_system_;
    std::unique_ptr<MotionSystem> motion_system_;
};
#endif  // AUV_ORCHESTRATOR_NODE_HPP_