#ifndef NLMPC_H
#define NLMPC_H

#include <vector>
#include <optional>
#include <string>
#include "auv_control/mapping/config.h"
#include "auv_control/control/model_inference.h"
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "auv_control/msg/environment_topic.hpp"
#include "auv_control/msg/mission_topic.hpp"

class NonlinearMPC {
public:
    NonlinearMPC(const rclcpp::Logger& logger, std::string modelPath = "/models/fossen_net_3/fossen_net_scripted.pt", std::string scalerPath = "/models/fossen_net_3/scalers.json");

    std::array<double, 8> solve(const auv_control::msg::EnvironmentTopic& local_env, const auv_control::msg::MissionTopic& local_mission);

    void initialization();
private:
    ModelInference model;
    std::string modelPath;
    std::string scalerPath;
    rclcpp::Logger logger_;
};

#endif // NLMPC_H