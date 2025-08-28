// model_inference.cpp
#include "auv_control/control/model_inference.h"
#include <iostream>
#include <torch/torch.h>
#include <stdexcept>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

ModelInference::ModelInference(const rclcpp::Logger& logger) : logger_(logger), loaded(false), scalers_loaded(false) {}

ModelInference::~ModelInference() {
    releaseResources();
}

bool ModelInference::loadModel(const std::string& modelPath) {
    try {
        // Check CUDA availability first
        if (!torch::cuda::is_available()) {
            RCLCPP_WARN(logger_, "CUDA is not available");
            throw std::runtime_error("CUDA is not available");
        }
        
        model = torch::jit::load(modelPath, torch::kCUDA);
        model.to(torch::kCUDA);
        loaded = true;
        RCLCPP_INFO(logger_, "Model loaded successfully from: %s", modelPath.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error loading model: %s", e.what());
        loaded = false;
    }
    return loaded;
}

bool ModelInference::loadScalers(const std::string& scalerPath) {
    try {
        std::ifstream f(scalerPath);
        if (!f.is_open()) {
            throw std::runtime_error("Cannot open scaler file: " + scalerPath);
        }
        
        json scaler_data = json::parse(f);
        
        // Load data from JSON into the member vectors
        x_mean = scaler_data["x_mean"].get<std::vector<float>>();
        x_std = scaler_data["x_std"].get<std::vector<float>>();
        y_mean = scaler_data["y_mean"].get<std::vector<float>>();
        y_std = scaler_data["y_std"].get<std::vector<float>>();

        // Validate sizes
        if (x_mean.size() != 25 || x_std.size() != 25 || y_mean.size() != 8 || y_std.size() != 8) {
            throw std::runtime_error("Scaler dimensions do not match expected model I/O.");
        }
        
        scalers_loaded = true;
        RCLCPP_INFO(logger_, "Scalers loaded successfully from: %s", scalerPath.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error loading scalers: %s", e.what());
        scalers_loaded = false;
    }
    return scalers_loaded;
}

std::vector<float> ModelInference::normalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std) {
    std::vector<float> normalized_data;
    normalized_data.reserve(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        normalized_data.push_back((data[i] - mean[i]) / std[i]);
    }
    return normalized_data;
}

std::vector<float> ModelInference::denormalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std) {
    std::vector<float> denormalized_data;
    denormalized_data.reserve(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        denormalized_data.push_back((data[i] * std[i]) + mean[i]);
    }
    return denormalized_data;
}

std::vector<float> ModelInference::runInference(const std::vector<float>& rawInputData) {
    if (!loaded) {
        throw std::runtime_error("Model not loaded!");
    }
    if (!scalers_loaded) {
        throw std::runtime_error("Scalers not loaded!");
    }
    
    // Validate input size
    if (rawInputData.size() != 25) {
        std::ostringstream err;
        err << "Invalid input size. Expected 25 elements, got " << rawInputData.size();
        throw std::invalid_argument(err.str());
    }
    
    try {
        std::vector<float> normalized_input = normalize(rawInputData, x_mean, x_std);

        torch::Tensor input_tensor = torch::tensor(normalized_input, torch::kFloat32)
                                   .view({1, 25})
                                   .to(torch::kCUDA);
        
        auto output_tensor = model.forward({input_tensor}).toTensor()
                               .to(torch::kCPU)
                               .contiguous();
        
        float* output_ptr = output_tensor.data_ptr<float>();
        std::vector<float> normalized_output(output_ptr, output_ptr + output_tensor.numel());

        std::vector<float> final_output = denormalize(normalized_output, y_mean, y_std);

        return final_output;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Inference error: %s", e.what());
        throw;
    }
}

void ModelInference::releaseResources() {
    loaded = false;
    scalers_loaded = false;
}