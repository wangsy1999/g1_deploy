#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace g1_camera {
using Clock = std::chrono::steady_clock;
constexpr int kWidth = 480;
constexpr int kHeight = 270;
constexpr int kFps = 60;

struct DepthFrame {
    uint64_t sequence = 0;
    double captured_at = 0; // 映射到本机 steady_clock 的采集时间，单位秒
    std::shared_ptr<const std::vector<float>> raw_depth; // 图像平面 Z，单位米
};
struct Sample {
    DepthFrame frame;
    Clock::time_point received_at;
};

// 使用接收偏移的最小值映射设备时钟：延迟帧不能通过重新打时间戳来续期。
class CaptureClock {
    bool initialized_ = false;
    double offset_ = 0, last_sensor_ = 0, last_received_ = 0;
public:
    double Map(double sensor_time, double received_at) {
        if (!std::isfinite(sensor_time) || !std::isfinite(received_at)) {
            throw std::runtime_error("Non-finite RealSense capture clock");
        }
        if (initialized_ && sensor_time <= last_sensor_) {
            throw std::runtime_error("RealSense capture clock restarted or stopped");
        }
        if (initialized_ && received_at < last_received_) {
            throw std::runtime_error("Host monotonic clock moved backwards");
        }
        const double candidate = received_at - sensor_time;
        offset_ = initialized_ ? std::min(offset_, candidate) : candidate;
        last_sensor_ = sensor_time;
        last_received_ = received_at;
        initialized_ = true;
        return std::min(received_at, sensor_time + offset_);
    }
};

// 从带行间距的 Z16 帧复制为独立米制图，避免借用 SDK 缓冲区跨线程传递。
inline std::vector<float> DepthMeters(const void* data, int width, int height,
                                      std::size_t stride, float depth_scale) {
    if (!data || width != kWidth || height != kHeight || stride < kWidth * sizeof(uint16_t) ||
        !std::isfinite(depth_scale) || depth_scale <= 0) {
        throw std::runtime_error("Invalid RealSense raw depth layout/scale");
    }
    std::vector<float> result(kWidth * kHeight);
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (int row = 0; row < kHeight; ++row) {
        for (int column = 0; column < kWidth; ++column) {
            uint16_t value;
            std::memcpy(&value, bytes + row * stride + column * sizeof(value), sizeof(value));
            const float meters = value * depth_scale;
            if (!std::isfinite(meters)) throw std::runtime_error("Invalid RealSense depth");
            result[row * kWidth + column] = meters;
        }
    }
    return result;
}

// 实机控制器内的独立 USB 采集线程。仅保留最新帧，不在 500 Hz 控制线程等待 USB。
// SDK 实现在 .cpp，仿真和纯策略测试不需要包含 librealsense 头文件。
class RealSenseCamera {
public:
    RealSenseCamera();
    ~RealSenseCamera();
    RealSenseCamera(const RealSenseCamera&) = delete;
    RealSenseCamera& operator=(const RealSenseCamera&) = delete;
    static bool Supported();
    std::optional<Sample> Latest(); // 无新帧返回空；采集异常在此交给控制器处理。
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace g1_camera
