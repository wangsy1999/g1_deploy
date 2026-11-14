#pragma once

#include "g1_config_utils.h"
#include "g1_joint_mapping.h"
#include "g1_observation_history.h"
#include "g1_onnx_utils.h"
#include <onnxruntime_cxx_api.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <limits>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

namespace g1_parkour {
inline constexpr int kJointCount = 29;
inline constexpr int kFrameSize = 96;
inline constexpr int kHistoryFrames = 8;
inline constexpr int kDepthHeight = 18;
inline constexpr int kDepthWidth = 32;
inline constexpr int kProprioceptionSize = kFrameSize * kHistoryFrames; // 768
inline constexpr int kDepthFeatureSize = 128;
inline constexpr int kActorInputSize = kProprioceptionSize + kDepthFeatureSize; // 896
using Joints = std::array<float, kJointCount>;
using Image = std::array<float, kDepthHeight * kDepthWidth>;
using ProprioceptionFrame = std::array<float, kFrameSize>;
using Velocity = std::array<float, 3>; // vx、vy (m/s)、wz (rad/s)
using TerrainFeatures = std::array<float, kDepthFeatureSize>;

// FNV-1a 的固定常量，用于计算关节名称、顺序和方向的配置指纹。
inline constexpr uint32_t kFnvOffsetBasis = 2166136261u;
inline constexpr uint32_t kFnvPrime = 16777619u;

struct Observation {
    uint64_t sequence = 0;
    double time = 0;
    Joints q{}, dq{}; // policy joint order
    std::array<float,3> omega{}, gravity{};
    float height = 0;
    std::shared_ptr<const std::vector<float>> raw_depth;
    bool depth_normalized=false;
    std::array<float,36*64> depth{}; // image-plane metres
};

// 状态历史按观测项排列，深度历史按完整图像排列；两者都从旧到新。
struct NetworkInputs {
    std::vector<float> proprioception;
    std::vector<float> depth_history;
};

class Policy {
public:
    explicit Policy(const std::string& config) {
        const auto settings = YAML::LoadFile(config);
        LoadJointParameters(settings);
        LoadActionParameters(settings);
        LoadHistoryTiming(settings);
        BuildJointMapping(settings);
        LoadCameraProfile(config);
        LoadNetworks(settings, config);
    }

    // 每次推理：整理状态和图像 → 理解前方地形 → 预测动作 → 换算电机关节目标。
    Joints Step(const Observation& observation, const Velocity& command) {
        auto inputs = Observe(observation, command);
        const auto terrain_features = EncodeDepth(inputs.depth_history);
        auto actor_input = CombineStateAndTerrain(std::move(inputs.proprioception), terrain_features);
        const auto predicted_actions = PredictActions(actor_input);
        return ProcessActions(predicted_actions);
    }

    // 整理本次状态和深度图，更新历史，再按模型训练时的顺序打包。
    NetworkInputs Observe(const Observation& observation, const Velocity& command) {
        const auto frame = BuildProprioception(observation, command);
        const auto image = PrepareDepthImage(observation);
        const bool uses_camera_history = observation.depth_normalized || bool(observation.raw_depth);
        UpdateHistory(frame, image, uses_camera_history);
        return {PackProprioceptionHistory(), PackDepthHistory(uses_camera_history)};
    }

    // 动作值 × 缩放 + 默认姿态，再按关节方向和 DDS 顺序转换成目标角度。
    // 先对网络动作限幅并保存反馈，再计算目标角度；最后应用电机目标限幅。
    Joints ProcessActions(const Joints& actions) {
        Joints target{};
        for (int joint = 0; joint < kJointCount; ++joint) {
            if (!std::isfinite(actions[joint])) {
                throw std::runtime_error("Non-finite Parkour action");
            }
            const float applied = std::clamp(actions[joint], -raw_clip_, raw_clip_);
            // 反馈仍为策略顺序的动作值，不包含角度缩放、偏置、方向和目标限幅。
            last_action_[joint] = clip_action_feedback_ ? applied : actions[joint];

            const int motor = ids[joint];
            target[motor] = sign[joint] * (neutral[joint] + scale[joint] * applied);
            if (!std::isfinite(target[motor])) throw std::runtime_error("Non-finite target");
            target[motor] = std::clamp(target[motor], target_lower_[joint], target_upper_[joint]);
        }
        return target;
    }

    void Reset() {
        history_.clear();
        images_.clear();
        last_action_.fill(0);
    }

    Joints DefaultDds() const {
        return g1_joints::ToMotorOrder(neutral, ids, sign);
    }

    std::array<int, kJointCount> ids{}; // policy joint index -> DDS motor index
    Joints neutral{}, kp{}, kd{}, scale{}, sign{}; // policy joint order
    uint32_t signature = kFnvOffsetBasis; // 校验仿真观测中的关节配置
    double dt = .02;

private:
    // 一帧依次为角速度、重力、速度指令、关节位置、关节速度、上次动作。
    static constexpr int kSimulationDepthFrames = 37;
    static constexpr int kSimulationDepthStride = 5;
    static constexpr int kAngularVelocityOffset = 0;
    static constexpr int kGravityOffset = 3;
    static constexpr int kCommandOffset = 6;
    static constexpr int kJointPositionOffset = 9;
    static constexpr int kJointVelocityOffset = kJointPositionOffset + kJointCount;
    static constexpr int kPreviousActionOffset = kJointVelocityOffset + kJointCount;

    // 每步推理的细节：构造状态与图像、更新历史、打包输入、执行两个网络。
    ProprioceptionFrame BuildProprioception(const Observation& observation,
                                            const Velocity& command) const {
        ProprioceptionFrame frame{};
        for (int axis = 0; axis < 3; ++axis) {
            frame[kAngularVelocityOffset + axis] = observation.omega[axis] * .25f;
            frame[kGravityOffset + axis] = observation.gravity[axis];
            frame[kCommandOffset + axis] = command[axis];
        }
        for (int joint = 0; joint < kJointCount; ++joint) {
            frame[kJointPositionOffset + joint] = observation.q[joint] - neutral[joint];
            frame[kJointVelocityOffset + joint] = observation.dq[joint] * .05f;
            frame[kPreviousActionOffset + joint] = last_action_[joint];
        }
        for (float value : frame)
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite proprioception");
        return frame;
    }

    Image PrepareDepthImage(const Observation& observation) const {
        if (observation.raw_depth) return PreprocessRaw(*observation.raw_depth);
        if (!observation.depth_normalized) return Preprocess(observation.depth);

        Image image{};
        std::copy_n(observation.depth.begin(), image.size(), image.begin());
        for (float value : image)
            if (!std::isfinite(value) || value < 0 || value > 1)
                throw std::runtime_error("Invalid normalized depth input");
        return image;
    }

    void UpdateHistory(const ProprioceptionFrame& frame, const Image& image, bool uses_camera_history) {
        g1_inference::PushHistory(history_, frame, kHistoryFrames);
        const int image_count = uses_camera_history ? camera_fps_ : kSimulationDepthFrames;
        g1_inference::PushHistory(images_, image, image_count);
    }

    std::vector<float> PackProprioceptionHistory() const {
        const std::array<int, 6> sizes{3, 3, 3, kJointCount, kJointCount, kJointCount};
        std::vector<float> input;
        input.reserve(kProprioceptionSize);
        g1_inference::PackTermMajor(history_, sizes, std::back_inserter(input));
        return input;
    }

    std::vector<float> PackDepthHistory(bool uses_camera_history) const {
        const int stride = uses_camera_history ? real_stride_ : kSimulationDepthStride;
        std::vector<float> input;
        input.reserve(kHistoryFrames * kDepthHeight * kDepthWidth);
        // 取八张从旧到新的图像，最新一张也要遵守配置中的延迟。
        for (int offset = (kHistoryFrames - 1) * stride; offset >= 0; offset -= stride) {
            const auto& image = images_[images_.size() - 1 - delay_ - offset];
            input.insert(input.end(), image.begin(), image.end());
        }
        return input;
    }

    // 把八张深度图交给编码器，提取 128 个地形特征。
    TerrainFeatures EncodeDepth(std::vector<float>& depth_history) {
        return g1_onnx::Run<kDepthFeatureSize>(*encoder_, depth_history, {1, kHistoryFrames, kDepthHeight, kDepthWidth},
                          encoder_in_, encoder_out_);
    }

    // 动作网络同时需要“机器人最近的状态”和“深度网络提取的地形特征”。
    static std::vector<float> CombineStateAndTerrain(
            std::vector<float> state_history,
            const TerrainFeatures& terrain_features) {
        // 固定顺序：先放 768 个状态历史值，再放 128 个地形特征值。
        state_history.insert(state_history.end(), terrain_features.begin(), terrain_features.end());
        return state_history;
    }

    Joints PredictActions(std::vector<float>& actor_input) {
        return g1_onnx::Run<kJointCount>(*actor_, actor_input, {1, kActorInputSize}, actor_in_, actor_out_);
    }

public:
    // 深度预处理也供离线检查使用；实机原始图与仿真旧协议分别处理。
    // Simulation/legacy input: 36x64 metres -> cropped, blurred 18x32 values in [0, 1].
    static Image Preprocess(const std::array<float, 36 * 64>& depth) {
        for (float value : depth)
            if (std::isnan(value) || value < 0) throw std::runtime_error("Invalid raw depth");

        const double side_weight = std::exp(-.5);
        const double weight_sum = 1 + 2 * side_weight;
        const std::array<double, 3> weights{side_weight / weight_sum, 1 / weight_sum, side_weight / weight_sum};
        auto reflect = [](int index, int size) {
            return index < 0 ? -index : (index >= size ? 2 * size - 2 - index : index);
        };

        Image image{};
        for (int y = 0; y < kDepthHeight; ++y) {
            for (int x = 0; x < kDepthWidth; ++x) {
                double value = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        const int source_y = reflect(y + dy, kDepthHeight) + 18;
                        const int source_x = reflect(x + dx, kDepthWidth) + 16;
                        const float distance = depth[source_y * 64 + source_x];
                        // An infinite no-hit remains far after blur and depth clipping.
                        value += weights[dy + 1] * weights[dx + 1] *
                                 (std::isinf(distance) ? 1e30 : double(distance));
                    }
                }
                image[y * kDepthWidth + x] = std::clamp(value, 0., 2.5) / 2.5;
            }
        }
        return image;
    }

    // RealSense input: resize -> crop -> fill holes -> optional mask/blur -> normalize.
    Image PreprocessRaw(const std::vector<float>& depth) const {
        if (depth.size() != 480 * 270 || !raw_profile_.IsMap())
            throw std::runtime_error("Raw depth requires 480x270 and sibling realsense_parkour.json");
        for (float value : depth)
            if (!std::isfinite(value) || value < 0) throw std::runtime_error("Invalid raw depth");

        const auto profile = raw_profile_;
        cv::Mat source(270, 480, CV_32FC1, const_cast<float*>(depth.data()));
        cv::Mat image;
        const cv::Size resized_size(profile["resize"][0].as<int>(), profile["resize"][1].as<int>());
        cv::resize(source, image, resized_size, 0, 0, cv::INTER_NEAREST);

        const auto crop = g1_config::ReadArray<float, 4>(profile["crop"]);
        const int top = crop[0], bottom = crop[1], left = crop[2], right = crop[3];
        if (top < 0 || bottom < 0 || left < 0 || right < 0 ||
            top + bottom >= image.rows || left + right >= image.cols)
            throw std::runtime_error("Invalid raw depth crop");
        image = image(cv::Rect(left, top, image.cols - left - right, image.rows - top - bottom)).clone();

        cv::Mat filled;
        cv::inpaint(image, image < profile["inpaint_below_m"].as<float>(), filled,
                    profile["inpaint_radius"].as<double>(), cv::INPAINT_NS);
        image = filled;
        if (profile["blind_spot_crop"]) {
            const auto blind = g1_config::ReadArray<float, 4>(profile["blind_spot_crop"]);
            for (int y = 0; y < image.rows; ++y) {
                for (int x = 0; x < image.cols; ++x) {
                    if (y < blind[0] || y >= image.rows - blind[1] ||
                        x < blind[2] || x >= image.cols - blind[3])
                        image.at<float>(y, x) = 0;
                }
            }
        }
        const int kernel = profile["gaussian_kernel"] ? profile["gaussian_kernel"].as<int>() : 0;
        if (kernel) {
            const double sigma = profile["gaussian_sigma"].as<double>();
            cv::GaussianBlur(image, image, cv::Size(kernel, kernel), sigma, sigma);
        }
        if (image.rows != kDepthHeight || image.cols != kDepthWidth)
            throw std::runtime_error("Model requires 18x32 depth");

        const auto depth_range = g1_config::ReadArray<float, 2>(profile["depth_range"]);
        const auto output_range = g1_config::ReadArray<float, 2>(profile["output_range"]);
        if (depth_range[1] <= depth_range[0] || output_range[0] != 0 || output_range[1] != 1)
            throw std::runtime_error("Invalid raw depth normalization");
        Image result{};
        for (int y = 0; y < kDepthHeight; ++y) {
            for (int x = 0; x < kDepthWidth; ++x) {
                const float value = (std::clamp(image.at<float>(y, x), depth_range[0], depth_range[1]) - depth_range[0]) /
                                    (depth_range[1] - depth_range[0]);
                if (!std::isfinite(value)) throw std::runtime_error("Non-finite processed depth");
                result[y * kDepthWidth + x] = value;
            }
        }
        return result;
    }

private:
    // 启动配置与模型检查，不参与每一步推理。
    void LoadJointParameters(const YAML::Node& settings) {
        neutral = g1_config::ReadArray<float, kJointCount>(settings["default_joint_pos"]);
        kp = g1_config::ReadArray<float, kJointCount>(settings["kp"]);
        kd = g1_config::ReadArray<float, kJointCount>(settings["kd"]);
        sign = g1_config::ReadArray<float, kJointCount>(settings["joint_sign"]);
    }

    void LoadActionParameters(const YAML::Node& settings) {
        scale = g1_config::ReadArray<float, kJointCount>(settings["action_scale"]);
        // 旧配置未提供这两项时，保留原来的原始动作反馈行为。
        raw_clip_ = settings["raw_clip"].as<float>(std::numeric_limits<float>::max());
        clip_action_feedback_ = settings["clip_action_feedback"].as<bool>(false);
        if (!std::isfinite(raw_clip_) || raw_clip_ <= 0) {
            throw std::runtime_error("Invalid Parkour raw action clip");
        }
        target_lower_ = g1_config::ReadArray<float, kJointCount>(settings["joint_clip_lower"]);
        target_upper_ = g1_config::ReadArray<float, kJointCount>(settings["joint_clip_upper"]);
        for (int joint = 0; joint < kJointCount; ++joint) {
            if (target_lower_[joint] > target_upper_[joint]) {
                throw std::runtime_error("Reversed Parkour target bounds");
            }
        }
    }

    void LoadHistoryTiming(const YAML::Node& settings) {
        delay_ = settings["depth_delay_frames"].as<int>();
        dt = settings["step_dt"].as<double>();
        camera_fps_ = settings["camera_depth_fps"] ? settings["camera_depth_fps"].as<int>() : 60;
        const int training_fps = settings["training_camera_fps"] ? settings["training_camera_fps"].as<int>() : 50;
        const int frame_skip = settings["depth_history_skip_frames"] ? settings["depth_history_skip_frames"].as<int>() : 5;
        if (camera_fps_ <= 0 || training_fps <= 0 || frame_skip <= 0)
            throw std::runtime_error("Invalid camera history configuration");
        if ((delay_ != 0 && delay_ != 1) || !std::isfinite(dt) || std::abs(dt - .02) > 1e-8)
            throw std::runtime_error("Expected 20ms parkour observations and depth delay 0 or 1");
        // History advances once per policy step, independently of camera capture FPS.
        real_stride_ = int(std::lround(double(frame_skip) / training_fps / dt));
        if (real_stride_ < 1 || (kHistoryFrames - 1) * real_stride_ + delay_ >= camera_fps_)
            throw std::runtime_error("Camera history cannot fit eight frames");
    }

    void BuildJointMapping(const YAML::Node& cfg) {
        const auto names = cfg["joint_names"].as<std::vector<std::string>>();
        ids = g1_joints::FromNames(names);
        for (int joint = 0; joint < kJointCount; ++joint) {
            if (sign[joint] != 1 && sign[joint] != -1) {
                throw std::runtime_error("joint_sign must be +1 or -1");
            }
            // 与发送端保持相同的字符串格式和哈希算法；逐字节累积，32 位自然回绕。
            const auto entry = names[joint] + ":" + std::to_string(static_cast<int>(sign[joint])) + "\n";
            for (unsigned char byte : entry) {
                signature = (signature ^ byte) * kFnvPrime;
            }
            if (kp[joint] <= 0 || kd[joint] < 0) {
                throw std::runtime_error("Invalid PD gains");
            }
        }
    }

    void LoadCameraProfile(const std::string& config) {
        const auto path = std::filesystem::path(config).parent_path() / "realsense_parkour.json";
        if (std::filesystem::exists(path))
            raw_profile_ = YAML::LoadFile(path.string())["preprocessing"];
    }

    void LoadNetworks(const YAML::Node& settings, const std::string& config) {
        auto model_path = [&](const char* key) {
            return g1_config::ResolvePath(config, settings["models"][key].as<std::string>());
        };
        auto options = g1_onnx::CpuSessionOptions();
        encoder_ = std::make_unique<Ort::Session>(env_, model_path("depth_encoder").c_str(), options);
        actor_ = std::make_unique<Ort::Session>(env_, model_path("actor").c_str(), options);
        g1_onnx::ValidateSingleInputNetwork(*encoder_, {1, kHistoryFrames, kDepthHeight, kDepthWidth},
                        {1, kDepthFeatureSize}, encoder_in_, encoder_out_);
        g1_onnx::ValidateSingleInputNetwork(*actor_, {1, kActorInputSize}, {1, kJointCount}, actor_in_, actor_out_);
    }

    // 模型及输入输出名称；env_ 的生命周期覆盖两个 session。
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "g1_parkour"};
    std::unique_ptr<Ort::Session> encoder_, actor_;
    std::string encoder_in_, encoder_out_, actor_in_, actor_out_;

    // 动作处理配置：raw_clip_ 限制动作值，target_* 限制最终电机角度。
    float raw_clip_ = std::numeric_limits<float>::max();
    bool clip_action_feedback_ = false;
    Joints target_lower_{}, target_upper_{};

    // 深度历史采样与预处理配置。
    int delay_ = 0;
    int camera_fps_ = 60, real_stride_ = 5;
    YAML::Node raw_profile_;

    // 推理线程独占的历史；Reset 清空状态、图像及上一次动作。
    std::deque<ProprioceptionFrame> history_;
    std::deque<Image> images_;
    Joints last_action_{};

};

// 共享内存观测格式：小端、32 字节帧头及 2369 个 float32。
inline Observation Decode(const unsigned char* bytes,std::size_t size,uint32_t signature) {
    constexpr std::size_t count=29+29+3+3+1+36*64;
    if(size!=32+count*4 || std::memcmp(bytes,"G1PKR001",8)!=0) throw std::runtime_error("Invalid observation packet");
    uint32_t order=0,n=0; Observation result;
    std::memcpy(&result.sequence,bytes+8,8);std::memcpy(&result.time,bytes+16,8);
    std::memcpy(&order,bytes+24,4);std::memcpy(&n,bytes+28,4);
    if(order!=signature || n!=count || !std::isfinite(result.time)) throw std::runtime_error("Observation joint order/count/time mismatch");
    std::array<float,count> values{};std::memcpy(values.data(),bytes+32,count*4);
    std::copy_n(values.begin(),29,result.q.begin());std::copy_n(values.begin()+29,29,result.dq.begin());
    std::copy_n(values.begin()+58,3,result.omega.begin());std::copy_n(values.begin()+61,3,result.gravity.begin());
    result.height=values[64];std::copy_n(values.begin()+65,36*64,result.depth.begin());
    for(int i=0;i<65;++i) if(!std::isfinite(values[i])) throw std::runtime_error("Non-finite observation state");
    for(float d:result.depth) if(std::isnan(d)||d<0) throw std::runtime_error("Invalid observation depth");
    return result;
}
} // namespace g1_parkour
