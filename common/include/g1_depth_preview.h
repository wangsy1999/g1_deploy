#pragma once
// Best-effort loopback preview. The controller only offers immutable frames;
// resizing/serialization/UDP run in a separate thread and never own the camera.
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

namespace g1_camera {
class DepthPreviewPublisher {
    using Clock = std::chrono::steady_clock;
    static constexpr int width = 64, height = 36;
    struct Frame {
        std::shared_ptr<const std::vector<float>> raw;
        std::array<float, width * height> small{};
        uint64_t sequence = 0;
        double captured_at = 0;
        bool simulation = false;
    };
    int socket_ = -1;
    sockaddr_in destination_{};
    std::atomic<bool> running_{false};
    std::mutex mutex_;
    std::condition_variable changed_;
    Frame pending_;
    bool have_pending_ = false;
    Clock::time_point next_offer_{}; // Only the control thread calls Offer().
    std::thread worker_;
    const uint64_t session_ = static_cast<uint64_t>(Clock::now().time_since_epoch().count());

    static void Integer(unsigned char* out, uint64_t value, int size) {
        for (int i = 0; i < size; ++i) out[i] = static_cast<unsigned char>(value >> (8 * i));
    }
    void Run() noexcept {
        while (running_) {
            Frame frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait_for(lock, std::chrono::milliseconds(100), [&] {return have_pending_ || !running_;});
                if (!running_) return;
                if (!have_pending_) continue;
                frame = std::move(pending_);
                have_pending_ = false;
            }
            if (frame.raw && frame.raw->size() != 480 * 270) continue;
            std::array<unsigned char, 40 + width * height * sizeof(float)> packet{};
            std::memcpy(packet.data(), "G1DPV001", 8);
            Integer(packet.data() + 8, session_, 8);
            Integer(packet.data() + 16, frame.sequence, 8);
            uint64_t time_bits;
            std::memcpy(&time_bits, &frame.captured_at, 8);
            Integer(packet.data() + 24, time_bits, 8);
            Integer(packet.data() + 32, width, 2);
            Integer(packet.data() + 34, height, 2);
            packet[36] = frame.simulation ? 1 : 0;
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    // Same nearest-neighbor sampling as policy's 480x270 -> 64x36 resize.
                    const float value = frame.raw ? (*frame.raw)[(y * 270 / height) * 480 + x * 480 / width]
                                                  : frame.small[y * width + x];
                    uint32_t bits;
                    std::memcpy(&bits, &value, 4);
                    Integer(packet.data() + 40 + (y * width + x) * 4, bits, 4);
                }
            }
            // A slow/absent receiver only loses preview packets, never policy observations.
            (void)sendto(socket_, packet.data(), packet.size(), MSG_DONTWAIT | MSG_NOSIGNAL,
                         reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
        }
    }
public:
    DepthPreviewPublisher() noexcept {
        const char* setting = std::getenv("G1_DEPTH_PREVIEW_PORT");
        char* end = nullptr;
        const long port = setting ? std::strtol(setting, &end, 10) : 9871;
        if (port < 1 || port > 65535 || (setting && (end == setting || *end))) return;
        socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (socket_ < 0) return;
        destination_.sin_family = AF_INET;
        destination_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        destination_.sin_port = htons(static_cast<uint16_t>(port));
        running_ = true;
        try { worker_ = std::thread([this] { Run(); }); }
        catch (...) { running_ = false; close(socket_); socket_ = -1; }
    }
    ~DepthPreviewPublisher() {
        running_ = false;
        changed_.notify_one();
        if (worker_.joinable()) worker_.join();
        if (socket_ >= 0) close(socket_);
    }
    DepthPreviewPublisher(const DepthPreviewPublisher&) = delete;
    DepthPreviewPublisher& operator=(const DepthPreviewPublisher&) = delete;

    void Offer(std::shared_ptr<const std::vector<float>> raw, const std::array<float, width * height>* small,
               uint64_t sequence, double captured_at) {
        if (!running_) return;
        const auto now = Clock::now();
        if (now < next_offer_) return;
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock()) return;
        pending_.raw = std::move(raw);
        if (small) pending_.small = *small;
        pending_.simulation = small != nullptr;
        pending_.sequence = sequence;
        pending_.captured_at = captured_at;
        have_pending_ = true;
        next_offer_ = now + std::chrono::microseconds(16000);
        lock.unlock();
        changed_.notify_one();
    }
};
} // namespace g1_camera
