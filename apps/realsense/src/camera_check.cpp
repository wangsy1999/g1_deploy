#include "g1_depth_camera.h"
#include <iostream>
#include <thread>

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "g1_camera_check: capture 60 fresh USB depth frames using C++; no DDS or robot commands\n";
        return 0;
    }
    if (argc != 1) { std::cerr << "Use g1_camera_check without arguments\n"; return 2; }
    try {
        g1_camera::RealSenseCamera camera;
        const auto deadline = g1_camera::Clock::now() + std::chrono::seconds(5);
        uint64_t last_sequence = 0;
        int count = 0;
        while (count < 60) {
            if (g1_camera::Clock::now() >= deadline) throw std::runtime_error("Camera check timed out");
            if (const auto sample = camera.Latest()) {
                const double now = std::chrono::duration<double>(g1_camera::Clock::now().time_since_epoch()).count();
                const auto& frame = sample->frame;
                if (frame.sequence <= last_sequence || now - frame.captured_at < -.05 || now - frame.captured_at >= .15 ||
                    !frame.raw_depth || frame.raw_depth->size() != 480 * 270) {
                    throw std::runtime_error("Camera check received stale or invalid depth");
                }
                last_sequence = frame.sequence;
                ++count;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::cout << "C++ RealSense: 60 fresh 480x270 depth frames verified; no robot commands\n";
    } catch (const std::exception& error) {
        std::cerr << "Camera check: " << error.what() << '\n';
        return 1;
    }
}
