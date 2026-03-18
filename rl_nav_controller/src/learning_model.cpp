#include "rl_nav_controller/learning_model.hpp"
#include "rl_nav_controller/quat_utils.hpp"

#include <cmath>
#include <cstring>
#include <iostream>
#include <numeric>

#include <opencv2/imgproc.hpp>

using namespace rl_nav_constants;

LearningModel::LearningModel(const std::filesystem::path& vae_model_path,
                             const std::filesystem::path& policy_model_path)
    : env_(ORT_LOGGING_LEVEL_WARNING, "rl_nav"),
      h_state_(kLstmHiddenDim, 0.0f),
      c_state_(kLstmHiddenDim, 0.0f)
{
    policy_scale_ = kPolicyScale;

    // Session options
    Ort::SessionOptions sess_options;
    sess_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    sess_options.SetIntraOpNumThreads(4);

    // Load VAE encoder
    vae_session_ = std::make_unique<Ort::Session>(env_, vae_model_path.c_str(), sess_options);

    // Load policy network
    policy_session_ = std::make_unique<Ort::Session>(env_, policy_model_path.c_str(), sess_options);

    // Get VAE input/output names
    Ort::AllocatorWithDefaultOptions allocator;
    vae_input_name_ = vae_session_->GetInputNameAllocated(0, allocator).get();
    vae_output_name_ = vae_session_->GetOutputNameAllocated(0, allocator).get();

    // Get policy input/output names
    size_t num_policy_inputs = policy_session_->GetInputCount();
    for (size_t i = 0; i < num_policy_inputs; i++) {
        policy_input_names_.push_back(policy_session_->GetInputNameAllocated(i, allocator).get());
    }
    size_t num_policy_outputs = policy_session_->GetOutputCount();
    for (size_t i = 0; i < num_policy_outputs; i++) {
        policy_output_names_.push_back(policy_session_->GetOutputNameAllocated(i, allocator).get());
    }

    std::cout << "\033[92mONNX models loaded successfully.\033[0m" << std::endl;
    std::cout << "VAE input: " << vae_input_name_ << ", output: " << vae_output_name_ << std::endl;
    std::cout << "Policy inputs:";
    for (const auto& name : policy_input_names_) std::cout << " " << name;
    std::cout << std::endl;
    std::cout << "Policy outputs:";
    for (const auto& name : policy_output_names_) std::cout << " " << name;
    std::cout << std::endl;

    fakeRunOnce();
    std::cout << "\033[92mLearning model is ready (ONNX Runtime C++).\033[0m" << std::endl;
}

std::vector<float> LearningModel::depthPreprocess(const cv::Mat& depth_image) {
    // Resize to (40, 64) using bilinear interpolation
    cv::Mat resized;
    cv::resize(depth_image, resized, cv::Size(kDepthTargetWidth, kDepthTargetHeight), 0, 0, cv::INTER_LINEAR);

    // Ensure float32
    cv::Mat resized_f32;
    resized.convertTo(resized_f32, CV_32F);

    // Create input tensor data: [1, 1, 40, 64]
    std::vector<float> input_data(kDepthTargetHeight * kDepthTargetWidth);
    std::memcpy(input_data.data(), resized_f32.ptr<float>(), input_data.size() * sizeof(float));

    // Create ONNX tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::array<int64_t, 4> input_shape = {1, 1, kDepthTargetHeight, kDepthTargetWidth};
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, input_data.data(), input_data.size(), input_shape.data(), input_shape.size());

    // Run VAE encoder
    const char* input_names[] = {vae_input_name_.c_str()};
    const char* output_names[] = {vae_output_name_.c_str()};
    auto output_tensors = vae_session_->Run(
        Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);

    // Extract output and flatten: [1, 64, 5, 8] -> [2560]
    float* output_data = output_tensors[0].GetTensorMutableData<float>();
    std::vector<float> embedding(kDepthEmbeddingDim);
    std::memcpy(embedding.data(), output_data, kDepthEmbeddingDim * sizeof(float));

    return embedding;
}

std::pair<std::array<float, 4>, Eigen::Vector3f> LearningModel::normalizeTargetPosition(
    const std::array<double, 3>& target_pos_w,
    const std::array<double, 3>& robot_pos_w,
    const std::array<double, 4>& robot_orientation_w)
{
    Eigen::Vector3f t_target(
        static_cast<float>(target_pos_w[0]),
        static_cast<float>(target_pos_w[1]),
        static_cast<float>(target_pos_w[2]));
    Eigen::Vector3f t_robot(
        static_cast<float>(robot_pos_w[0]),
        static_cast<float>(robot_pos_w[1]),
        static_cast<float>(robot_pos_w[2]));
    Eigen::Vector4f q_robot(
        static_cast<float>(robot_orientation_w[0]),
        static_cast<float>(robot_orientation_w[1]),
        static_cast<float>(robot_orientation_w[2]),
        static_cast<float>(robot_orientation_w[3]));

    // subtract_frame_transforms: T_12 = T_01^{-1} * T_02
    auto [inv_pos, inv_rot] = rl_nav_quat::subtract_frame_transforms(t_robot, q_robot);

    // Transform target point to robot frame
    Eigen::Vector3f target_vec_b = rl_nav_quat::transform_point(t_target, inv_pos, inv_rot);

    // Compute normalized direction + log distance
    float dist = target_vec_b.norm() + 1e-6f;
    Eigen::Vector3f target_dir = target_vec_b / dist;
    float dist_log = std::log(dist + 1.0f);

    std::array<float, 4> target_pos_log = {target_dir(0), target_dir(1), target_dir(2), dist_log};

    return {target_pos_log, target_vec_b};
}

void LearningModel::resetHiddenState() {
    std::fill(h_state_.begin(), h_state_.end(), 0.0f);
    std::fill(c_state_.begin(), c_state_.end(), 0.0f);
}

PredictResult LearningModel::predict(
    const std::vector<double>& linear_vel,
    const std::vector<double>& angular_vel,
    const std::vector<double>& gravity_vector,
    const std::vector<double>& last_action,
    const std::array<double, 3>& target_pos_w,
    const std::array<double, 3>& robot_pos_w,
    const std::array<double, 4>& robot_orientation_w,
    const cv::Mat& depth_image,
    bool is_reset)
{
    if (is_reset) {
        resetHiddenState();
    }

    // Preprocess depth image -> 2560-dim embedding
    std::vector<float> depth_embedding = depthPreprocess(depth_image);

    // Normalize target position
    auto [target_pos_log, target_vec_b] = normalizeTargetPosition(
        target_pos_w, robot_pos_w, robot_orientation_w);

    // Assemble state input: linear_vel(3) + angular_vel(3) + gravity(3) + last_action(3) + target_pos_log(4) = 16
    std::vector<float> obs(kPolicyInputDim);
    int idx = 0;
    for (int i = 0; i < 3; i++) obs[idx++] = static_cast<float>(linear_vel[i]);
    for (int i = 0; i < 3; i++) obs[idx++] = static_cast<float>(angular_vel[i]);
    for (int i = 0; i < 3; i++) obs[idx++] = static_cast<float>(gravity_vector[i]);
    for (int i = 0; i < 3; i++) obs[idx++] = static_cast<float>(last_action[i]);
    for (int i = 0; i < 4; i++) obs[idx++] = target_pos_log[i];

    // Append depth embedding
    std::memcpy(&obs[kStateDim], depth_embedding.data(), kDepthEmbeddingDim * sizeof(float));

    // Create ONNX tensors
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    std::array<int64_t, 2> obs_shape = {1, kPolicyInputDim};
    Ort::Value obs_tensor = Ort::Value::CreateTensor<float>(
        memory_info, obs.data(), obs.size(), obs_shape.data(), obs_shape.size());

    std::array<int64_t, 3> hidden_shape = {1, 1, kLstmHiddenDim};
    Ort::Value h_in_tensor = Ort::Value::CreateTensor<float>(
        memory_info, h_state_.data(), h_state_.size(), hidden_shape.data(), hidden_shape.size());
    Ort::Value c_in_tensor = Ort::Value::CreateTensor<float>(
        memory_info, c_state_.data(), c_state_.size(), hidden_shape.data(), hidden_shape.size());

    // Build input arrays
    std::vector<Ort::Value> input_tensors;
    input_tensors.push_back(std::move(obs_tensor));
    input_tensors.push_back(std::move(h_in_tensor));
    input_tensors.push_back(std::move(c_in_tensor));

    std::vector<const char*> input_names_cstr;
    for (const auto& name : policy_input_names_) {
        input_names_cstr.push_back(name.c_str());
    }
    std::vector<const char*> output_names_cstr;
    for (const auto& name : policy_output_names_) {
        output_names_cstr.push_back(name.c_str());
    }

    // Run policy inference
    auto output_tensors = policy_session_->Run(
        Ort::RunOptions{nullptr},
        input_names_cstr.data(), input_tensors.data(), input_tensors.size(),
        output_names_cstr.data(), output_names_cstr.size());

    // Extract outputs
    float* actions_data = output_tensors[0].GetTensorMutableData<float>();
    float* h_out_data = output_tensors[1].GetTensorMutableData<float>();
    float* c_out_data = output_tensors[2].GetTensorMutableData<float>();

    // Update LSTM hidden states
    std::memcpy(h_state_.data(), h_out_data, kLstmHiddenDim * sizeof(float));
    std::memcpy(c_state_.data(), c_out_data, kLstmHiddenDim * sizeof(float));

    // Apply tanh + scaling
    PredictResult result;
    for (int i = 0; i < 3; i++) {
        result.cmd_vel[i] = std::tanh(actions_data[i]) * policy_scale_[i];
        result.raw_action[i] = actions_data[i];
    }
    result.target_vec_b = target_vec_b;

    return result;
}

void LearningModel::fakeRunOnce() {
    std::vector<double> linear_vel = {0.0, 0.0, 0.0};
    std::vector<double> angular_vel = {0.0, 0.0, 0.0};
    std::vector<double> gravity_vector = {0.0, 0.0, -1.0};
    std::vector<double> last_action_vec = {0.0, 0.0, 0.0};
    std::array<double, 3> target_pos_w = {1.0, 0.0, 0.0};
    std::array<double, 3> robot_pos_w = {0.0, 0.0, 0.0};
    std::array<double, 4> robot_orientation_w = {1.0, 0.0, 0.0, 0.0};

    // Create a random depth image (600x960)
    cv::Mat depth_image(600, 960, CV_32F);
    cv::randu(depth_image, 0.0, 1.0);

    auto result = predict(
        linear_vel, angular_vel, gravity_vector, last_action_vec,
        target_pos_w, robot_pos_w, robot_orientation_w,
        depth_image, true);

    std::cout << "Predicted cmd_vel: linear_x=" << result.cmd_vel[0]
              << ", linear_y=" << result.cmd_vel[1]
              << ", angular_z=" << result.cmd_vel[2] << std::endl;

    // Reset hidden state after warmup
    resetHiddenState();
}
