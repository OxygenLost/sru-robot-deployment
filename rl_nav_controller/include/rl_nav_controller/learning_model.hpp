#pragma once

#include <array>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <Eigen/Core>

#include "rl_nav_controller/constants.hpp"

struct PredictResult {
    std::array<float, 3> cmd_vel;
    std::array<float, 3> raw_action;
    Eigen::Vector3f target_vec_b;
};

class LearningModel {
public:
    LearningModel(const std::filesystem::path& vae_model_path,
                  const std::filesystem::path& policy_model_path);

    std::vector<float> depthPreprocess(const cv::Mat& depth_image);

    std::pair<std::array<float, 4>, Eigen::Vector3f> normalizeTargetPosition(
        const std::array<double, 3>& target_pos_w,
        const std::array<double, 3>& robot_pos_w,
        const std::array<double, 4>& robot_orientation_w);

    PredictResult predict(
        const std::vector<double>& linear_vel,
        const std::vector<double>& angular_vel,
        const std::vector<double>& gravity_vector,
        const std::vector<double>& last_action,
        const std::array<double, 3>& target_pos_w,
        const std::array<double, 3>& robot_pos_w,
        const std::array<double, 4>& robot_orientation_w,
        const cv::Mat& depth_image,
        bool is_reset = false);

    void resetHiddenState();

private:
    void fakeRunOnce();

    Ort::Env env_;
    std::unique_ptr<Ort::Session> vae_session_;
    std::unique_ptr<Ort::Session> policy_session_;

    // VAE input/output names
    std::string vae_input_name_;
    std::string vae_output_name_;

    // Policy input/output names
    std::vector<std::string> policy_input_names_;
    std::vector<std::string> policy_output_names_;

    // LSTM hidden states
    std::vector<float> h_state_;
    std::vector<float> c_state_;

    // Policy output scaling
    std::array<float, 3> policy_scale_;
};
