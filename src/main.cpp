// G1 29DOF simulation/hardware joint reset: current position -> default pose -> hold.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <cerrno>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <memory>
#include <net/if.h>
#include "g1_hardware.h"
#include "g1_gamepad.h"
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#ifdef G1_WITH_POLICY
#include "g1_locomotion_policy.h"
#endif

using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;
using Clock = std::chrono::steady_clock;

namespace {
constexpr std::size_t kJointCount = 29;
constexpr auto kPeriod = std::chrono::milliseconds(2);  // 500 Hz
volatile std::sig_atomic_t stop_requested = 0;
void Stop(int) { stop_requested = 1; }

// Read single keys without Enter; restore terminal settings on every exit.
class Keyboard {
public:
    Keyboard() {
        if (!isatty(STDIN_FILENO)) return;
        if (tcgetattr(STDIN_FILENO, &original_) != 0)
            throw std::runtime_error("Cannot read terminal settings");
        auto settings = original_;
        settings.c_lflag &= ~(ICANON | ECHO);  // keep Ctrl+C signal handling
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &settings) != 0)
            throw std::runtime_error("Cannot enable keyboard input");
        restore_ = true;
    }
    ~Keyboard() {
        if (restore_) tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
    char Read(bool eof_is_exit=true) {
        pollfd input{STDIN_FILENO, POLLIN, 0};
        const int ready = poll(&input, 1, 0);
        if (ready < 0) {
            if (errno == EINTR) return 0;
            throw std::runtime_error("Cannot poll keyboard input");
        }
        if (!ready) return 0;
        if (input.revents & (POLLERR | POLLNVAL))
            throw std::runtime_error("Keyboard input closed");
        char key = 0;
        const auto count = read(STDIN_FILENO, &key, 1);
        if (count == 0) return eof_is_exit ? ' ' : 0;
        if (count < 0 && errno != EINTR && errno != EAGAIN)
            throw std::runtime_error("Cannot read keyboard input");
        return count > 0 ? key : 0;
    }
private:
    termios original_{};
    bool restore_ = false;
};

struct ResetConfig {
    std::array<float, kJointCount> default_pos{};
    std::array<float, kJointCount> kp{};
    std::array<float, kJointCount> kd{};
};

ResetConfig LoadConfig(const std::string& path) {
    const auto root = YAML::LoadFile(path);
    ResetConfig config;
    const auto ids = root["joint_ids_map"];
    const auto positions = root["default_joint_pos"];
    const auto kp = root["stiffness"];
    const auto kd = root["damping"];
    for (const auto& array : {ids, positions, kp, kd})
        if (!array.IsSequence() || array.size() != kJointCount)
            throw std::runtime_error("Expected 29 joint IDs, default positions and PD gains in locomotion YAML");
    std::array<bool, kJointCount> seen{};
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const int id = ids[i].as<int>();
        if (id < 0 || id >= int(kJointCount) || seen[id])
            throw std::runtime_error("joint_ids_map must be a permutation of 0..28");
        seen[id] = true;
        const float q = positions[i].as<float>();
        config.default_pos[id] = q; // Default positions use policy order.
        config.kp[i] = kp[i].as<float>(); // PD arrays already use DDS order.
        config.kd[i] = kd[i].as<float>();
        if (!std::isfinite(q) || !std::isfinite(config.kp[i]) ||
            !std::isfinite(config.kd[i]) || config.kp[i] < 0 || config.kd[i] < 0)
            throw std::runtime_error("Invalid default position or PD gains in locomotion YAML");
    }
    return config;
}

// SDK wire CRC, excluding the final crc field. memcpy avoids aliasing.
uint32_t CommandCrc(const LowCmd_& command) {
    return g1_hardware::Crc(command);
}
}  // namespace

class JointReset {
public:
    explicit JointReset(const ResetConfig& config, const g1_hardware::Limits* hardware, bool read_only
#ifdef G1_WITH_POLICY
        , g1_policy::Policy* policy, g1_gamepad::Device* gamepad
#endif
    ) : config_(config), hardware_(hardware), read_only_(read_only), publisher_("rt/lowcmd"), target_publisher_("rt/g1/reset_target"), enable_publisher_("rt/g1/reset_enable"), subscriber_("rt/lowstate") {
#ifdef G1_WITH_POLICY
        policy_ = policy;
        gamepad_ = gamepad;
#endif
        if (!read_only_) {
            publisher_.InitChannel();
            target_publisher_.InitChannel();
            enable_publisher_.InitChannel();
        }
        if (hardware_ && !read_only_) {
            command_monitor_ = std::make_unique<unitree::robot::ChannelSubscriber<LowCmd_>>("rt/lowcmd");
            command_monitor_->InitChannel([this](const void*) { command_seen_ = true; }, 1);
        }
        subscriber_.InitChannel(
            std::bind(&JointReset::OnState, this, std::placeholders::_1), 1);
    }
    ~JointReset() { subscriber_.CloseChannel(); if (command_monitor_) command_monitor_->CloseChannel(); }

    int Run(double duration) {
        std::cout << "Waiting for G1 29DOF lowstate (5 s timeout)...\n";
        const auto deadline = Clock::now() + std::chrono::seconds(5);
        LowState_ state{};
        while (!stop_requested && !Snapshot(state)) {
            if (Clock::now() >= deadline) {
                std::cerr << "No valid lowstate. Check scene/robot, DDS domain and interface. " << StateError() << "\n";
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (stop_requested) return 0;
        // C++ bridge publishes 4 for 23DOF, 5 for 29DOF. Python bridge
        // leaves mode_machine at 0, so its scene must be checked by the user.
        if ((hardware_ && state.mode_machine() != 5) || (!hardware_ && state.mode_machine() != 0 && state.mode_machine() != 5)) {
            std::cerr << "Expected 29DOF mode_machine 5, received "
                      << unsigned(state.mode_machine()) << ".\n";
            return 1;
        }
        if (read_only_) {
            std::cout << "Read-only hardware check: valid CRC, G1 mode=5, joints/IMU/motors OK. No control service release or LowCmd publication.\n";
            return 0;
        }
        std::array<float, kJointCount> previous_target{};
        for (std::size_t i = 0; i < kJointCount; ++i) previous_target[i] = state.motor_state()[i].q();
        std::array<float, kJointCount> start_angles{};

        LowCmd_ command{};
        command.mode_pr() = 0;  // pitch/roll joint coordinates, not A/B motors
        command.mode_machine() = state.mode_machine();
        Keyboard keyboard;
        std::cout << "Ready. Command output DISABLED. 9: power on, P: reset (" << duration
                  << " s), Space / Ctrl+C: exit.\n";
        auto start = Clock::now();
        auto next = start;
        bool active = false;
        bool output_enabled = false;
        bool publishing = false;
        bool holding = false;
        bool power_locked = false; // Once reset starts, only restarting the process permits power-on.
#ifdef G1_WITH_POLICY
        bool policy_active = false;
        std::array<float, 3> velocity{};
        g1_policy::Joints policy_target{};
        auto next_policy = Clock::now();
        std::cout << "Sequence: 9 -> P (wait for reset) -> R (policy). P: return to reset; 9 is locked after P.\n"
                  << "W/S: forward +/-0.1 m/s, A/D: lateral +/-0.1 m/s, Q/E: yaw +/-0.1 rad/s, X: zero velocity.\n";
#endif
        auto publish_enable = [&](bool enabled) {
            LowCmd_ status{};
            for (std::size_t i = 0; i < kJointCount; ++i)
                status.motor_cmd()[i].mode() = enabled ? 1 : 0;
            status.crc() = CommandCrc(status);
            enable_publisher_.Write(status);
        };
        auto release = [&]() {
            if (!publishing) return;
            // Hardware: enabled damping for a bounded interval; never assume mode=0 is damping.
            for (std::size_t i = 0; i < kJointCount; ++i) {
                auto& motor = command.motor_cmd()[i];
                motor.mode() = hardware_ ? 1 : 0;
                motor.q() = motor.kp() = motor.tau() = motor.dq() = 0;
                motor.kd() = hardware_ ? hardware_->stop_kd : 0;
            }
            const auto deadline = Clock::now() + std::chrono::duration<double>(hardware_ ? hardware_->damping_seconds : 0);
            do {
                command.crc() = CommandCrc(command);
                if (!publisher_.Write(command) && hardware_)
                    std::cerr << "Damping command DDS publication failed; execution is unconfirmed.\n";
                if (hardware_) std::this_thread::sleep_for(kPeriod);
            } while (Clock::now() < deadline);
            publishing = false;
        };
        int result = 0;
        try {
        while (!stop_requested) {
            if (!Snapshot(state) || state.mode_machine() != command.mode_machine()) {
                std::cerr << "Lowstate stale/invalid or robot mode changed; stopping. " << StateError() << "\n";
                result = 1;
                break;
            }
            char key = 0;
            try {
                key = keyboard.Read(!gamepad_);
                if(gamepad_) {
                    gamepad_->Poll();
                    const char action=gamepad_->state.Action();
                    if(action==' ' || (key==0 && action)) key=action;
                }
            } catch (const std::exception& error) {
                std::cerr << error.what() << '\n';
                result = 1;
                break;
            }
            if (key == ' ') break;
            if (key == '9') {
                if (power_locked) {
                    std::cout << "Power-on locked after reset/policy entry; restart the controller to power on again.\n";
                } else if (output_enabled) {
                    std::cout << "Command output already ENABLED. Press P to reset.\n";
                } else {
                    if (hardware_) {
                        ReleaseMotionService();
                        publishing = true; // Ensure faults after confirmed handover enter damping stop.
                        if (!Snapshot(state)) throw std::runtime_error("No valid state after releasing motion service");
                        for (std::size_t i = 0; i < kJointCount; ++i) previous_target[i] = state.motor_state()[i].q();
                    }
                    output_enabled = true;
                    std::cout << "Command output ENABLED. Press P to reset.\n";
                }
            }
            if ((key == 'p' || key == 'P') && !output_enabled) {
                std::cout << "Reset requires power-on. Press 9 before P.\n";
            }
            if ((key == 'p' || key == 'P') && output_enabled) {
                power_locked = true;
#ifdef G1_WITH_POLICY
                policy_active = false;
                velocity.fill(0);
                if (policy_) policy_->Reset();
#endif
                for (std::size_t i = 0; i < kJointCount; ++i)
                    start_angles[i] = state.motor_state()[i].q();
                start = Clock::now();
                active = true;
                holding = false;
                std::cout << "Reset target generated from current joint angles."
                          << (output_enabled ? " Publishing enabled.\n" : " Output disabled; targets only.\n");
            }
#ifdef G1_WITH_POLICY
            if ((key == 'r' || key == 'R') && !policy_active) {
                bool pose_ready = true;
                const auto neutral = policy_->DefaultDds();
                for (std::size_t i = 0; i < kJointCount; ++i)
                    if (std::abs(state.motor_state()[i].q() - neutral[i]) > 0.35f) pose_ready = false;
                if (!output_enabled || !active || !holding || !pose_ready) {
                    std::cout << "Policy requires power-on and completed reset near the training pose. Follow 9 -> P -> R; wait for reset completion.\n";
                } else if(gamepad_ && (!gamepad_->state.Ready() || !gamepad_->state.Centered() || gamepad_->state.MovingEnabled())) {
                    std::cout<<"Center sticks and release RB before policy entry.\n";
                } else {
                    policy_->Reset();
                    velocity.fill(0);
                    policy_active = true;
                    if(gamepad_) gamepad_->state.BeginPolicy();
                    next_policy = Clock::now();
                    std::cout << "Locomotion policy ON; velocity = [0, 0, 0]. P: reset, Space / Ctrl+C: exit.\n";
                }
            }
            if (policy_active) {
                bool changed = true;
                switch (key) {
                    case 'w': case 'W': velocity[0] += 0.1f; break;
                    case 's': case 'S': velocity[0] -= 0.1f; break;
                    case 'a': case 'A': velocity[1] += 0.1f; break;
                    case 'd': case 'D': velocity[1] -= 0.1f; break;
                    case 'q': case 'Q': velocity[2] += 0.1f; break;
                    case 'e': case 'E': velocity[2] -= 0.1f; break;
                    case 'x': case 'X': velocity.fill(0); break;
                    default: changed = false;
                }
                if(gamepad_) velocity=policy_->GamepadVelocity(gamepad_->state.ControlVelocity(key));
                policy_->ClampVelocity(velocity);
                if (changed) std::cout << "Velocity: " << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << '\n';
                if (Clock::now() >= next_policy) {
                    g1_policy::State input;
                    for (std::size_t i = 0; i < kJointCount; ++i) {
                        input.q[i] = state.motor_state()[i].q();
                        input.dq[i] = state.motor_state()[i].dq();
                    }
                    std::copy_n(state.imu_state().quaternion().begin(), 4, input.quaternion.begin());
                    std::copy_n(state.imu_state().gyroscope().begin(), 3, input.omega.begin());
                    const auto inference_start = Clock::now();
                    policy_target = policy_->Step(input, velocity);
                    if (hardware_ && Clock::now()-inference_start > std::chrono::duration<double>(policy_->dt))
                        throw std::runtime_error("Policy inference exceeded its control period");
                    next_policy += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(policy_->dt));
                    if (next_policy < Clock::now())
                        next_policy = Clock::now() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(policy_->dt));
                }
            }
#endif
            publish_enable(output_enabled);
            if (!active) {
                if (hardware_ && output_enabled) {
                    for (std::size_t i = 0; i < kJointCount; ++i) {
                        auto& motor = command.motor_cmd()[i];
                        motor.mode() = 1;
                        motor.q() = motor.kp() = motor.tau() = motor.dq() = 0;
                        motor.kd() = hardware_->stop_kd;
                    }
                    command.crc() = CommandCrc(command);
                    if (!publisher_.Write(command)) throw std::runtime_error("Damping DDS publication failed");
                    publishing = true;
                }
                next += kPeriod;
                if (next < Clock::now()) next = Clock::now() + kPeriod;
                std::this_thread::sleep_until(next);
                continue;  // do not publish commands until P is pressed after power-on
            }
            const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
            const double u = std::clamp(elapsed / duration, 0.0, 1.0);
            const double blend = u * u * (3.0 - 2.0 * u);
            for (std::size_t i = 0; i < kJointCount; ++i) {
                auto& motor = command.motor_cmd()[i];
                motor.mode() = 1;
#ifdef G1_WITH_POLICY
                if (policy_active) {
                    motor.q() = policy_target[i];
                    motor.kp() = policy_->kp[i];
                    motor.kd() = policy_->kd[i];
                } else {
#endif
                motor.q() = u == 1.0 ? config_.default_pos[i] :
                    static_cast<float>(start_angles[i] +
                        blend * (config_.default_pos[i] - start_angles[i]));
                motor.kp() = config_.kp[i];
                motor.kd() = config_.kd[i];
#ifdef G1_WITH_POLICY
                }
#endif
                motor.dq() = 0;
                motor.tau() = 0;
            }
            if (hardware_) {
                if (!Snapshot(state)) throw std::runtime_error("Hardware state invalid before command publication: " + StateError());
                hardware_->LimitCommand(command, state, previous_target, 0.002);
            }
            command.crc() = CommandCrc(command);
            // Telemetry only: the simulator does not subscribe to this topic.
            target_publisher_.Write(command);
            if (output_enabled) {
                if (!publisher_.Write(command)) throw std::runtime_error("LowCmd DDS publication failed");
                publishing = true;
            }
            if (u == 1.0 && !holding) {
                std::cout << "Target interpolation complete."
                          << (output_enabled ? " Holding default pose.\n" : " Output disabled; target only.\n");
                holding = true;
            }
            next += kPeriod;
            if (next < Clock::now()) next = Clock::now() + kPeriod;
            std::this_thread::sleep_until(next);
        }
        } catch (const std::exception& error) {
            std::cerr << "Controller error: " << error.what() << '\n';
            result = 1;
        }
        release();
        publish_enable(false);
        return result;
    }

private:
    g1_gamepad::Device* gamepad_ = nullptr;
#ifdef G1_WITH_POLICY
    g1_policy::Policy* policy_ = nullptr;
#endif
    void OnState(const void* message) {
        const auto& state = *static_cast<const LowState_*>(message);
        std::lock_guard<std::mutex> lock(mutex_);
        valid_ = false;
        if (hardware_fault_) return;
        if (hardware_) {
            try { hardware_->CheckState(state); }
            catch (const std::exception& error) { state_error_ = error.what(); hardware_fault_ = true; return; }
        }
        for (std::size_t i = 0; i < kJointCount; ++i)
            if (!std::isfinite(state.motor_state()[i].q()) || !std::isfinite(state.motor_state()[i].dq())) return;
        state_ = state;
        received_at_ = Clock::now();
        valid_ = true;
    }
    bool Snapshot(LowState_& state) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!valid_ || Clock::now() - received_at_ > std::chrono::duration<double, std::milli>(hardware_ ? hardware_->timeout_ms : 500))
            return false;
        state = state_;
        return true;
    }

    std::string StateError() { std::lock_guard<std::mutex> lock(mutex_); return state_error_; }
    void ReleaseMotionService() {
        unitree::robot::b2::MotionSwitcherClient client;
        client.SetTimeout(5.0f);
        client.Init();
        bool released = false;
        for (int attempt = 0; attempt < 5 && !stop_requested; ++attempt) {
            std::string form, name;
            if (client.CheckMode(form, name) != 0) throw std::runtime_error("MotionSwitcher CheckMode failed");
            if (name.empty()) { released = true; break; }
            if (client.ReleaseMode() != 0) throw std::runtime_error("MotionSwitcher ReleaseMode failed");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (!released || stop_requested) throw std::runtime_error("Motion service release not confirmed");
        command_seen_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        if (command_seen_) throw std::runtime_error("Another LowCmd publisher is active after service release");
    }
    const ResetConfig config_;
    const g1_hardware::Limits* hardware_ = nullptr;
    bool read_only_ = false;
    bool hardware_fault_ = false; // A hardware validation failure requires restarting the process.
    std::string state_error_;
    std::atomic<bool> command_seen_{false};
    std::unique_ptr<unitree::robot::ChannelSubscriber<LowCmd_>> command_monitor_;
    std::mutex mutex_;
    LowState_ state_{};
    Clock::time_point received_at_{};
    bool valid_ = false;
    unitree::robot::ChannelPublisher<LowCmd_> publisher_;
    unitree::robot::ChannelPublisher<LowCmd_> target_publisher_;
    unitree::robot::ChannelPublisher<LowCmd_> enable_publisher_;
    unitree::robot::ChannelSubscriber<LowState_> subscriber_;
};

int main(int argc, char** argv) {
#ifdef G1_WITH_POLICY
    std::string joystick_path;
    bool check_joystick=false;
    std::vector<char*> positional{argv[0]};
    for(int i=1;i<argc;++i) {
        const std::string argument=argv[i];
        if(argument=="--joystick") {
            if(i+1==argc) {std::cerr<<"--joystick requires a device path\n";return 1;}
            joystick_path=argv[++i];
        } else if(argument=="--check-joystick") check_joystick=true;
        else positional.push_back(argv[i]);
    }
    argc=static_cast<int>(positional.size());
    positional.push_back(nullptr);argv=positional.data();
#endif
    const bool hardware = argc > 1 && (std::string(argv[1]) == "--hardware" || std::string(argv[1]) == "--hardware-read-only");
    const bool read_only = hardware && std::string(argv[1]) == "--hardware-read-only";
    const std::string program = argv[0];
    if (hardware) { --argc; ++argv; }
#ifdef G1_WITH_POLICY
    if(hardware && check_joystick) {
        std::cerr<<"Use --check-joystick without --hardware; input diagnostics never connect to DDS.\n";return 1;
    }
    if(read_only && !joystick_path.empty()) {
        std::cerr<<"Hardware read-only check does not use a gamepad; omit --joystick.\n";return 1;
    }
    if(check_joystick) {
        try {
            if(argc!=1) throw std::runtime_error("Use --check-joystick [--joystick DEVICE] without control arguments");
            g1_gamepad::Device gamepad(joystick_path.empty()?"/dev/input/js0":joystick_path,true);
            std::cout<<"Gamepad: "<<gamepad.Description()<<'\n';
            std::signal(SIGINT,Stop);std::signal(SIGTERM,Stop);
            g1_gamepad::CheckInput(gamepad,stop_requested);return 0;
        } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
    }
    if (argc > 1 && std::string(argv[1]) == "--check-policy") {
        try {
            if (argc > 4) throw std::runtime_error("Usage: --check-policy [model.onnx] [deploy.yaml]");
            g1_policy::Policy policy(argc > 2 ? argv[2] : G1_POLICY_DEFAULT_MODEL,
                                     argc > 3 ? argv[3] : G1_POLICY_DEFAULT_CONFIG);
            g1_policy::State state;
            state.q = policy.DefaultDds();
            const auto target = policy.Step(state, {0, 0, 0});
            std::cout << "Policy OK: " << policy.Interface() << ", dt=" << policy.dt << " s\nDDS targets:";
            for (float q : target) std::cout << ' ' << q;
            std::cout << '\n';
            return 0;
        } catch (const std::exception& error) {
            std::cerr << "Policy check failed: " << error.what() << '\n';
            return 1;
        }
    }
    constexpr int max_args = 6;
#else
    constexpr int max_args = 5;
#endif
    if (argc > max_args || (argc > 1 && std::string(argv[1]) == "--help")) {
        std::cout << "Usage: " << argv[0] << " [reset_seconds=3] [interface=lo] [domain=1] [locomotion.yaml]\n"
                  << "Default: MuJoCo, positive domain. Hardware: prepend --hardware or --hardware-read-only, explicitly set interface and domain 0.\n";
#ifdef G1_WITH_POLICY
        std::cout << "Optional argument after locomotion.yaml: [model.onnx]\n"
                  << "Offline inference: --check-policy [model.onnx] [deploy.yaml]\n"
                  << "F710: --joystick /dev/input/js0; input-only check: --check-joystick [--joystick DEVICE]\n"
                  << "Start enable, A reset, Y run; hold RB + left stick XY/right stick X to move; X zero, B exit.\n";
#endif
        return argc > max_args ? 1 : 0;
    }
    try {
#ifdef G1_WITH_POLICY
        std::unique_ptr<g1_gamepad::Device> gamepad;
        if(!joystick_path.empty()) {
            gamepad=std::make_unique<g1_gamepad::Device>(joystick_path,true);
            std::cout<<"Gamepad: "<<gamepad->Description()<<'\n';
        }
#endif
        std::size_t consumed = 0;
        const std::string duration_text = argc > 1 ? argv[1] : "3";
        const double duration = std::stod(duration_text, &consumed);
        if (consumed != duration_text.size() || !std::isfinite(duration) || duration <= 0)
            throw std::invalid_argument("reset_seconds must be finite and positive");
        if (hardware && argc < 4) throw std::invalid_argument("Hardware usage: " + program + " --hardware[-read-only] 3 <network_interface> 0 [locomotion.yaml] [model.onnx]");
        const std::string domain_text = argc > 3 ? argv[3] : "1";
        const int domain = std::stoi(domain_text, &consumed);
        if (consumed != domain_text.size() || (hardware ? domain != 0 : domain <= 0))
            throw std::invalid_argument("Hardware domain must be 0; simulation domain must be positive");
        if (hardware && (std::string(argv[2]) == "lo" || if_nametoindex(argv[2]) == 0))
            throw std::invalid_argument("Specify an existing non-loopback robot network interface");
        const std::string config_path = argc > 4 ? argv[4] : G1_POLICY_DEFAULT_CONFIG;
        const auto config = LoadConfig(config_path);
        std::cout << "Loaded shared locomotion configuration: " << config_path << '\n';
#ifdef G1_WITH_POLICY
        std::unique_ptr<g1_policy::Policy> policy;
        if (!read_only) {
            policy = std::make_unique<g1_policy::Policy>(argc > 5 ? argv[5] : G1_POLICY_DEFAULT_MODEL, config_path);
            std::cout << "Loaded locomotion policy, period " << policy->dt << " s\n";
        }
#endif
        std::unique_ptr<g1_hardware::Limits> hardware_limits;
        if (hardware) {
            hardware_limits = std::make_unique<g1_hardware::Limits>(YAML::LoadFile(config_path));
            for (int i = 0; i < 29; ++i)
                if (config.default_pos[i] < hardware_limits->lower[i]+0.01f || config.default_pos[i] > hardware_limits->upper[i]-0.01f)
                    throw std::runtime_error("Reset pose outside hardware joint limits");
        }
        std::signal(SIGTERM, Stop);
        std::signal(SIGINT, Stop);
        std::signal(SIGHUP, Stop);
        unitree::robot::ChannelFactory::Instance()->Init(domain, argc > 2 ? argv[2] : "lo");
        JointReset controller(config, hardware_limits.get(), read_only
#ifdef G1_WITH_POLICY
            , policy.get(), gamepad.get()
#endif
        );
        return controller.Run(duration);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
