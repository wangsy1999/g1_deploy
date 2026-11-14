#pragma once

#include "g1_config_utils.h"
#include "g1_joint_mapping.h"
#include "g1_observation_history.h"
#include "g1_onnx_utils.h"

// G1 velocity interface from unitreerobotics/unitree_rl_lab (Apache-2.0).
// No Isaac runtime is used. Arrays from DDS are reordered before inference.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <yaml-cpp/yaml.h>

namespace g1_policy {
constexpr int kJoints = 29;
constexpr int kFrame = 96;
constexpr int kHistory = 5;
constexpr int kObservation = kFrame * kHistory;
using Joints = std::array<float, kJoints>;
using Velocity = std::array<float, 3>; // vx、vy (m/s)、wz (rad/s)
using Frame = std::array<float, kFrame>;
using Observation = std::array<float, kObservation>;
using Bounds = std::array<float, 2>; // 下限、上限

struct State {
    Joints q{};  // 电机 DDS 顺序的实测关节角度，单位 rad
    Joints dq{}; // 电机 DDS 顺序的实测关节角速度，单位 rad/s
    std::array<float, 4> quaternion{1, 0, 0, 0}; // w,x,y,z
    std::array<float, 3> omega{}; // IMU 测得的机体坐标系角速度，单位 rad/s
};

class Policy {
public:
    explicit Policy(const std::string& config)
        : env_(ORT_LOGGING_LEVEL_WARNING, "g1_locomotion") {
        const auto cfg = YAML::LoadFile(config);
        const auto model_path = g1_config::ResolvePath(config, cfg["policy_path"].as<std::string>());
        // 先读懂模型约定：关节顺序、动作转换、观测布局和速度范围。
        LoadJointParameters(cfg);
        LoadActionParameters(cfg);
        LoadObservationParameters(cfg);
        LoadVelocityLimits(cfg);
        LoadNetwork(model_path);
        Reset();
    }

    // 每次推理只做三件事：整理机器人状态 → 预测动作 → 换算电机关节目标。
    Joints Step(const State& state, const Velocity& velocity) {
        auto obs = Observe(state, velocity);
        const auto actions = PredictActions(obs);
        return ProcessActions(actions);
    }

    // 把当前状态整理成 96 个数，再更新五帧历史。
    // 输入模型时按配置排列历史，时间顺序始终为旧帧到新帧。
    Observation Observe(const State& state, const Velocity& velocity) {
        const auto frame = BuildObservationFrame(state, velocity);
        UpdateHistory(frame);
        return PackObservationHistory();
    }

    // 网络给出的是动作值；按配置缩放、加偏置、限幅后，才是 DDS 顺序的目标角度。
    // 同时记录本次动作，供下一次推理作为“上一次动作”输入。
    Joints ProcessActions(const Joints& actions) {
        Joints target{};
        for (int joint = 0; joint < kJoints; ++joint) {
            if (!std::isfinite(actions[joint])) {
                throw std::runtime_error("Non-finite policy action");
            }
            const float applied = std::clamp(actions[joint], -raw_clip_, raw_clip_);
            // 按训练配置决定下次反馈原始动作还是限幅后的动作，均保持策略关节顺序。
            last_action_[joint] = clip_action_feedback_ ? applied : actions[joint];

            const int motor = motor_ids_[joint];
            const float position = action_offset_[joint] + action_scale_[joint] * applied;
            if (!std::isfinite(position)) {
                throw std::runtime_error("Non-finite policy target");
            }
            target[motor] = std::clamp(position, target_clip_[joint][0], target_clip_[joint][1]);
        }
        return target;
    }

    // 新一轮控制从零动作反馈开始；首次观测会填满历史，覆盖上轮状态。
    void Reset() {
        last_action_.fill(0);
        history_ready_ = false;
    }

    const char* Interface() const {
        return input_names_.size() == 2 ? "float32 [1,96] + [1,480] -> [1,29]"
                                       : "float32 [1,480] -> [1,29]";
    }

    Joints DefaultDds() const {
        return g1_joints::ToMotorOrder(default_pos_, motor_ids_);
    }

    void ClampVelocity(Velocity& command) const {
        for (int axis = 0; axis < 3; ++axis) {
            command[axis] = std::clamp(command[axis], velocity_limits_[axis][0], velocity_limits_[axis][1]);
        }
    }

    // 摇杆三个轴分别对应前进、侧移和转向；正负方向可配置不同速度上限。
    Velocity GamepadVelocity(const Velocity& normalized) const {
        Velocity velocity{};
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(normalized[axis])) {
                throw std::runtime_error("Invalid gamepad velocity");
            }
            const float input = std::clamp(normalized[axis], -1.f, 1.f);
            const float speed_limit = input < 0 ? -velocity_limits_[axis][0] : velocity_limits_[axis][1];
            velocity[axis] = input * speed_limit;
        }
        return velocity;
    }

    // 控制器直接读取：PD 增益采用 DDS 电机顺序，推理周期 dt 的单位为秒。
    Joints kp{}, kd{};
    double dt = 0.02;

private:
    // 一帧的排列：角速度(3)、重力(3)、速度指令(3)、关节位置(29)、关节速度(29)、上次动作(29)。
    static constexpr int kAngularVelocityOffset = 0;
    static constexpr int kGravityOffset = 3;
    static constexpr int kCommandOffset = 6;
    static constexpr int kJointPositionOffset = 9;
    static constexpr int kJointVelocityOffset = kJointPositionOffset + kJoints;
    static constexpr int kPreviousActionOffset = kJointVelocityOffset + kJoints;

    // 推理细节按执行顺序排列：构造当前帧、更新历史、打包历史、运行模型。
    Frame BuildObservationFrame(const State& state, const Velocity& velocity) const {
        Frame frame{};

        // 先归一化姿态四元数，再计算机器人自身坐标系中的重力方向。
        float norm = 0;
        for (float component : state.quaternion) {
            norm += component * component;
        }
        if (!std::isfinite(norm) || norm < 1e-8f) throw std::runtime_error("Invalid IMU quaternion");
        norm = std::sqrt(norm);
        const float w = state.quaternion[0] / norm;
        const float x = state.quaternion[1] / norm;
        const float y = state.quaternion[2] / norm;
        const float z = state.quaternion[3] / norm;

        for (int i = 0; i < 3; ++i) frame[kAngularVelocityOffset + i] = state.omega[i];
        // 将世界坐标系的单位重力 (0, 0, -1) 逆旋转到机体坐标系。
        frame[kGravityOffset] = 2 * (w * y - x * z);
        frame[kGravityOffset + 1] = -2 * (y * z + w * x);
        frame[kGravityOffset + 2] = 2 * (x * x + y * y) - 1;
        for (int i = 0; i < 3; ++i) frame[kCommandOffset + i] = velocity[i];

        // DDS 状态按电机编号存放；模型输入和动作反馈按训练时的关节顺序存放。
        for (int joint = 0; joint < kJoints; ++joint) {
            const int motor = motor_ids_[joint];
            frame[kJointPositionOffset + joint] = state.q[motor] - default_pos_[joint];
            frame[kJointVelocityOffset + joint] = state.dq[motor];
            frame[kPreviousActionOffset + joint] = last_action_[joint];
        }

        // 和训练一致：所有观测项先缩放，再限幅。
        for (int i = 0; i < kFrame; ++i) {
            frame[i] *= obs_scale_[i];
            if (!std::isfinite(frame[i])) throw std::runtime_error("Non-finite observation");
            frame[i] = std::clamp(frame[i], obs_clip_[i][0], obs_clip_[i][1]);
        }
        return frame;
    }

    void UpdateHistory(const Frame& frame) {
        current_frame_ = frame;
        g1_inference::PushHistory(history_, frame, history_ready_);
    }

    Observation PackObservationHistory() const {
        Observation observation{};
        if (time_major_history_) {
            // time: [完整旧帧, …, 完整新帧]。
            auto output = observation.begin();
            for (const auto& frame : history_)
                output = std::copy(frame.begin(), frame.end(), output);
        } else {
            // term: [五帧角速度, 五帧重力, …, 五帧上次动作]。
            const std::array<int, 6> sizes{3, 3, 3, kJoints, kJoints, kJoints};
            g1_inference::PackTermMajor(history_, sizes, observation.begin());
        }
        return observation;
    }

    Joints PredictActions(Observation& observation) {
        // 按模型声明的输入顺序绑定缓冲区；双输入模型还需要独立的当前帧。
        std::vector<g1_onnx::InputTensor> inputs;
        for (std::size_t i = 0; i < input_names_.size(); ++i) {
            float* data = input_sizes_[i] == kFrame ? current_frame_.data() : observation.data();
            inputs.push_back({input_names_[i].c_str(), data, static_cast<std::size_t>(input_sizes_[i]),
                              {1, input_sizes_[i]}});
        }
        return g1_onnx::Run<kJoints>(*session_, inputs, output_name_);
    }

    // 启动时读取配置并检查模型接口；不参与每一步推理。
    void LoadJointParameters(const YAML::Node& cfg) {
        motor_ids_ = g1_config::ReadArray<int, kJoints>(cfg["joint_ids_map"]);
        g1_joints::ValidatePermutation(motor_ids_);
        default_pos_ = g1_config::ReadArray<float, kJoints>(cfg["default_joint_pos"]);
        kp = g1_config::ReadArray<float, kJoints>(cfg["stiffness"]); // upstream stores gains in DDS order
        kd = g1_config::ReadArray<float, kJoints>(cfg["damping"]);
        for (int i = 0; i < kJoints; ++i)
            if (kp[i] < 0 || kd[i] < 0) throw std::runtime_error("Negative PD gains");
        dt = cfg["step_dt"].as<double>();
        if (!std::isfinite(dt) || dt < 0.002) throw std::runtime_error("Invalid policy step_dt");
    }

    void LoadActionParameters(const YAML::Node& cfg) {
        const auto action = cfg["actions"]["JointPositionAction"];
        action_scale_ = g1_config::ReadArray<float, kJoints>(action["scale"]);
        action_offset_ = g1_config::ReadArray<float, kJoints>(action["offset"]);
        if (!action["joint_ids"].IsNull())
            throw std::runtime_error("This interface requires full-joint actions");
        raw_clip_ = action["raw_clip"].as<float>(std::numeric_limits<float>::max());
        clip_action_feedback_ = action["clip_action_feedback"].as<bool>(false);
        if (!std::isfinite(raw_clip_) || raw_clip_ <= 0)
            throw std::runtime_error("Invalid raw action clip");
        for (int i = 0; i < kJoints; ++i) {
            target_clip_[i] = {-std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            if (!action["clip"].IsNull()) {
                if (!action["clip"].IsSequence() || action["clip"].size() != kJoints)
                    throw std::runtime_error("Expected 29 joint target clip bounds");
                target_clip_[i] = g1_config::ReadArray<float, 2>(action["clip"][i]);
                if (target_clip_[i][0] > target_clip_[i][1])
                    throw std::runtime_error("Reversed joint target clip bounds");
            }
        }
    }

    void LoadObservationParameters(const YAML::Node& cfg) {
        const auto order = cfg["history_order"].as<std::string>("term");
        if (order != "time" && order != "term")
            throw std::runtime_error("Expected history_order time or term");
        time_major_history_ = order == "time";
        const auto observations = cfg["observations"];
        const std::array<const char*, 6> names{"base_ang_vel", "projected_gravity",
            "velocity_commands", "joint_pos_rel", "joint_vel_rel", "last_action"};
        const std::array<int, 6> sizes{3, 3, 3, 29, 29, 29};
        if (observations.size() != names.size()) throw std::runtime_error("Unexpected observation terms");
        int term = 0, index = 0;
        for (const auto& entry : observations) {
            if (entry.first.as<std::string>() != names[term])
                throw std::runtime_error("Unexpected observation order");
            const auto node = entry.second;
            if (node["history_length"].as<int>() != kHistory ||
                !node["scale"].IsSequence() || node["scale"].size() != std::size_t(sizes[term]))
                throw std::runtime_error("Expected five-frame observations");
            Bounds clip{-std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
            if (!node["clip"].IsNull()) clip = g1_config::ReadArray<float, 2>(node["clip"]);
            if (clip[0] > clip[1]) throw std::runtime_error("Reversed observation clip bounds");
            for (int i = 0; i < sizes[term]; ++i) {
                float value = node["scale"][i].as<float>();
                if (!std::isfinite(value)) throw std::runtime_error("Non-finite observation scale");
                obs_clip_[index] = clip;
                obs_scale_[index++] = value;
            }
            ++term;
        }
    }

    void LoadVelocityLimits(const YAML::Node& cfg) {
        const auto ranges = cfg["commands"]["base_velocity"]["ranges"];
        const std::array<const char*, 3> velocity_names{"lin_vel_x", "lin_vel_y", "ang_vel_z"};
        for (int i = 0; i < 3; ++i) {
            velocity_limits_[i] = g1_config::ReadArray<float, 2>(ranges[velocity_names[i]]);
            if (velocity_limits_[i][0] > 0 || velocity_limits_[i][1] < 0)
                throw std::runtime_error("Command limits must include zero");
        }
    }

    void LoadNetwork(const std::string& model_path) {
        auto options = g1_onnx::CpuSessionOptions();
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options);
        const auto count = session_->GetInputCount();
        if ((count != 1 && count != 2) || session_->GetOutputCount() != 1)
            throw std::runtime_error("Expected history input, or current observation and history inputs, and one output");
        bool has_frame = false, has_history = false;
        for (std::size_t i = 0; i < count; ++i) {
            const auto shape = g1_onnx::TensorShape(*session_, true, i);
            if (shape.size() != 2 || shape[0] != 1 ||
                (shape[1] != kFrame && shape[1] != kObservation))
                throw std::runtime_error("Expected float32 inputs [1,96] and/or [1,480]");
            const bool frame = shape[1] == kFrame;
            if ((frame && has_frame) || (!frame && has_history))
                throw std::runtime_error("Duplicate policy input shape");
            has_frame = has_frame || frame;
            has_history = has_history || !frame;
            input_sizes_.push_back(shape[1]);
            input_names_.emplace_back(g1_onnx::TensorName(*session_, true, i));
        }
        if (!has_history || (count == 2 && !has_frame))
            throw std::runtime_error("Policy requires a history input");
        if (g1_onnx::TensorShape(*session_, false, 0) != std::vector<int64_t>({1, kJoints}))
            throw std::runtime_error("Expected float32 output [1,29]");
        output_name_ = g1_onnx::TensorName(*session_, false, 0);
    }

    // 模型及输入输出接口。env_ 必须比 session_ 更早构造、更晚销毁。
    Ort::Env env_;
    std::unique_ptr<Ort::Session> session_;
    std::vector<std::string> input_names_;
    std::vector<int64_t> input_sizes_;
    std::string output_name_;

    // 关节参数按策略顺序保存；motor_ids_[joint] 给出对应的 DDS 电机编号。
    std::array<int, kJoints> motor_ids_{};
    Joints default_pos_{};
    Joints action_scale_{};
    Joints action_offset_{};
    std::array<Bounds, kJoints> target_clip_{};
    float raw_clip_ = std::numeric_limits<float>::max();
    bool clip_action_feedback_ = false;

    // 观测缩放、限幅及历史布局，均须与训练配置一致。
    Frame obs_scale_{};
    std::array<Bounds, kFrame> obs_clip_{};
    std::array<Bounds, 3> velocity_limits_{};
    bool time_major_history_ = false;

    // 每步更新的策略状态。Reset 后首次观测会重新填满整个历史窗口。
    Frame current_frame_{};
    std::array<Frame, kHistory> history_{};
    Joints last_action_{};
    bool history_ready_ = false;
};
} // namespace g1_policy
