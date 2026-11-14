#include "g1_depth_camera.h"

#ifdef G1_WITH_REALSENSE
#include <librealsense2/rs.hpp>
#include <atomic>
#include <exception>
#include <mutex>
#include <thread>

namespace g1_camera {
struct RealSenseCamera::Impl {
    rs2::pipeline pipeline;
    std::atomic<bool> running{true};
    std::mutex mutex;
    std::optional<Sample> latest;
    std::exception_ptr error;
    std::thread worker;

    Impl() {
        rs2::config config;
        config.enable_stream(RS2_STREAM_DEPTH, kWidth, kHeight, RS2_FORMAT_Z16, kFps);
        // SDK 默认可能再启用 RGB；显式只请求深度，不做彩色对齐或设备滤波。
        const auto profile = pipeline.start(config);
        try {
            const auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
            if (stream.width() != kWidth || stream.height() != kHeight || stream.fps() != kFps ||
                stream.format() != RS2_FORMAT_Z16) {
                throw std::runtime_error("RealSense requires 480x270 Z16 at 60Hz");
            }
            const auto intrinsics = stream.get_intrinsics();
            for (float coefficient : intrinsics.coeffs) {
                if (!std::isfinite(coefficient) || std::abs(coefficient) > 1e-8f) {
                    throw std::runtime_error("RealSense depth must use a rectified profile");
                }
            }
            worker = std::thread([this] { Run(); });
        } catch (...) {
            try { pipeline.stop(); } catch (...) {}
            throw;
        }
    }

    ~Impl() {
        running = false;
        if (worker.joinable()) worker.join();
        try { pipeline.stop(); } catch (...) {} // 析构不得掩盖控制器正在处理的异常。
    }

    void Run() {
        try {
            CaptureClock clock;
            uint64_t sequence = 0, last_number = 0;
            bool have_frame = false;
            rs2_timestamp_domain timestamp_domain{};
            auto last_received = Clock::now();
            while (running) {
                rs2::frameset frames;
                if (!pipeline.try_wait_for_frames(&frames, 100)) {
                    if (Clock::now() - last_received >= std::chrono::seconds(1)) {
                        throw std::runtime_error("RealSense depth capture timed out");
                    }
                    continue;
                }
                const auto received_at = Clock::now();
                const auto depth = frames.get_depth_frame();
                if (!depth) throw std::runtime_error("RealSense returned no depth frame");
                const auto number = depth.get_frame_number();
                if (have_frame && number == last_number) {
                    if (received_at - last_received >= std::chrono::seconds(1)) {
                        throw std::runtime_error("RealSense depth frame sequence stopped");
                    }
                    continue;
                }
                if (have_frame && (number < last_number || depth.get_frame_timestamp_domain() != timestamp_domain)) {
                    throw std::runtime_error("RealSense stream restarted; restart controller");
                }
                timestamp_domain = depth.get_frame_timestamp_domain();
                const double received_seconds = std::chrono::duration<double>(received_at.time_since_epoch()).count();
                DepthFrame output;
                output.sequence = ++sequence;
                output.captured_at = clock.Map(depth.get_timestamp() * .001, received_seconds);
                output.raw_depth = std::make_shared<const std::vector<float>>(DepthMeters(
                    depth.get_data(), depth.get_width(), depth.get_height(), depth.get_stride_in_bytes(), depth.get_units()));
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    latest = Sample{std::move(output), received_at};
                }
                last_number = number;
                last_received = received_at;
                have_frame = true;
            }
        } catch (...) {
            std::lock_guard<std::mutex> lock(mutex);
            error = std::current_exception();
        }
    }
};
RealSenseCamera::RealSenseCamera() : impl_(std::make_unique<Impl>()) {}
RealSenseCamera::~RealSenseCamera() = default;
bool RealSenseCamera::Supported() { return true; }
std::optional<Sample> RealSenseCamera::Latest() {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    if (impl_->error) std::rethrow_exception(impl_->error);
    auto result = std::move(impl_->latest);
    impl_->latest.reset();
    return result;
}
} // namespace g1_camera
#else
namespace g1_camera {
struct RealSenseCamera::Impl {};
RealSenseCamera::RealSenseCamera() {
    throw std::runtime_error("Native RealSense support unavailable: install librealsense2 development files and rebuild");
}
RealSenseCamera::~RealSenseCamera() = default;
bool RealSenseCamera::Supported() { return false; }
std::optional<Sample> RealSenseCamera::Latest() { return std::nullopt; }
} // namespace g1_camera
#endif
