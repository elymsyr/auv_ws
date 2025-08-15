#include "auv_control/control/fossennet.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <torch/torch.h>
#include "auv_control/control/model_inference.h"
#include <vector>
#include <optional>
#include <fstream>
#include <array>
#include "auv_control/msg/environment_topic.hpp"
#include "auv_control/msg/mission_topic.hpp"

NonlinearMPC::NonlinearMPC(const rclcpp::Logger& logger, std::string modelPath, std::string scalerPath) : modelPath(modelPath), scalerPath(scalerPath), logger_(logger), model(logger) {}

void NonlinearMPC::initialization() {
    if (torch::cuda::is_available()) {
        torch::globalContext().setBenchmarkCuDNN(true);
    }

    std::string package_share_path;
    try {
        package_share_path = ament_index_cpp::get_package_share_directory("auv_control");
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger_, "Package 'auv_control' not found -> ament_index_cpp::get_package_share_directory: %s", e.what());
        // Handle error, maybe throw an exception or shutdown
        return;
    }

    try {
        RCLCPP_INFO(logger_, "Loading model and scalers...");
        if (!model.loadScalers(package_share_path + scalerPath)) {
            RCLCPP_FATAL(logger_, "Failed to load scalers from: %s", (package_share_path + scalerPath).c_str());
            throw std::runtime_error("Failed to load scalers.");
        }
        if (!model.loadModel(package_share_path + modelPath)) {
            RCLCPP_FATAL(logger_, "Failed to load model from: %s", (package_share_path + modelPath).c_str());
            throw std::runtime_error("Failed to load model.");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Initialization error: %s", e.what());
        throw; // Re-throw the exception
    }
}

std::array<double, 8> NonlinearMPC::solve(const auv_control::msg::EnvironmentTopic& local_env, const auv_control::msg::MissionTopic& local_mission)
{
    std::vector<float> rawInputData;
    rawInputData.reserve(12 + HORIZON * 12);

    for (size_t i = 3; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.eta[i]));
    }
    for (size_t i = 0; i < 6; ++i) {
        rawInputData.push_back(static_cast<float>(local_env.nu[i]));
    }

    for (size_t i = 1; i <= HORIZON; ++i) {
        const auto& point = local_mission.trajectory[i];

        rawInputData.push_back(static_cast<float>(point.eta_desired[0] - local_env.eta[0]));
        rawInputData.push_back(static_cast<float>(point.eta_desired[1] - local_env.eta[1]));
        rawInputData.push_back(static_cast<float>(point.eta_desired[2] - local_env.eta[2]));
        
        for (size_t j = 3; j < 6; ++j) {
            rawInputData.push_back(static_cast<float>(point.eta_desired[j]));
        }
        for (size_t j = 0; j < 6; ++j) {
            rawInputData.push_back(static_cast<float>(point.nu_desired[j]));
        }
    }

    std::vector<float> control_output_float;
    try {
        control_output_float = model.runInference(rawInputData);
    } catch (const std::exception& e) {
        std::cerr << "An error occurred during inference in solve(): " << e.what() << std::endl;
        std::array<double, 8> error_control = {0.0};
        return error_control;
    }

    std::array<double, 8> control_input = {0.0};
    if (control_output_float.size() == control_input.size()) {
        for (size_t i = 0; i < control_input.size(); ++i) {
            control_input[i] = static_cast<double>(control_output_float[i]);
        }
    } else {
        RCLCPP_WARN(logger_, "Model output size (%zu) does not match expected control size (%zu).",
                        control_output_float.size(), control_input.size());
    }

    return control_input;
}