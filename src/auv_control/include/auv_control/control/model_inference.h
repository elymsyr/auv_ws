#ifndef MODEL_INFERENCE_H
#define MODEL_INFERENCE_H

#include <torch/script.h>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"

class ModelInference {
public:
    ModelInference(const rclcpp::Logger& logger);
    ~ModelInference();

    bool loadModel(const std::string& modelPath);
    bool loadScalers(const std::string& scalerPath);
    std::vector<float> runInference(const std::vector<float>& inputData);
    void releaseResources();
    torch::Tensor preprocess_input(const std::vector<float>& input_data);

private:
    torch::jit::script::Module model;
    bool loaded;
    rclcpp::Logger logger_;

    std::vector<float> x_mean, x_std, y_mean, y_std;
    bool scalers_loaded;

    // Helper functions for normalization
    std::vector<float> normalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std);
    std::vector<float> denormalize(const std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& std);
};

#endif // MODEL_INFERENCE_H