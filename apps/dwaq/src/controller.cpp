// DWaQ controller: DDS state, reset/hold, policy scheduling and actuator lifecycle.
#include "g1_dwaq_app.h"
#include "g1_loop_scheduler.h"
#include "g1_dds_state_cache.h"
#include "g1_monitor_publisher.h"
#include "g1_joint_mapping.h"
#include "g1_config_utils.h"
#include "g1_keyboard.h"
#include "g1_control_signals.h"
#include "g1_control_runtime.h"
#include "g1_motion_service.h"
#include "g1_control_targets.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <memory>
#include "g1_hardware.h"
#include "g1_motor_channel.h"
#include "g1_async_log.h"
#include "g1_gamepad.h"
#include "g1_unitree_remote.h"

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include "g1_locomotion_policy.h"
#include "g1_policy_worker.h"

using unitree_hg::msg::dds_::LowCmd_;
using unitree_hg::msg::dds_::LowState_;
using Clock = std::chrono::steady_clock;

namespace {
constexpr std::size_t kJointCount = 29;
struct ResetConfig {
    std::array<float, kJointCount> default_pos{};
    std::array<float, kJointCount> kp{};
    std::array<float, kJointCount> kd{};
};

ResetConfig LoadConfig(const std::string& path) {
    const auto root = YAML::LoadFile(path);
    ResetConfig config;
    const auto ids = g1_config::ReadArray<int, kJointCount>(root["joint_ids_map"]);
    const auto positions = g1_config::ReadArray<float, kJointCount>(root["default_joint_pos"]);
    config.kp = g1_config::ReadArray<float, kJointCount>(root["stiffness"]);
    config.kd = g1_config::ReadArray<float, kJointCount>(root["damping"]);
    g1_joints::ValidatePermutation(ids);
    config.default_pos = g1_joints::ToMotorOrder(positions, ids);
    for (std::size_t i = 0; i < kJointCount; ++i) {
        const float q = positions[i];
        if (!std::isfinite(q) || !std::isfinite(config.kp[i]) ||
            !std::isfinite(config.kd[i]) || config.kp[i] < 0 || config.kd[i] < 0)
            throw std::runtime_error("Invalid default position or PD gains in locomotion JSON");
    }
    return config;
}

class DwaqController {
public:
    explicit DwaqController(const ResetConfig& config, const g1_hardware::Limits* hardware, bool read_only, bool unitree_remote
        , g1_policy::Policy* policy, g1_gamepad::Device* gamepad
    ) : config_(config), hardware_(hardware), read_only_(read_only), unitree_remote_(unitree_remote), state_receiver_("rt/lowstate") {
        policy_ = policy;
        gamepad_ = gamepad;
        if (!read_only_) {
            // On hardware, no actuator writer exists until the explicit 9/Start handover.
            if (!hardware_) InitMotorPublisher();
            telemetry_.Start();
        }
        if (hardware_ && !read_only_) {
            command_monitor_ = std::make_unique<g1_control::CommandMonitor>();
        }
        state_receiver_.Start([this](const LowState_& state) { ValidateState(state); },
                              [this] { return hardware_ && (read_only_ || output_armed_); });
    }
    ~DwaqController() { state_receiver_.Close(); command_monitor_.reset(); }

    int Run(double duration) {
        reset_seconds = duration;
        const int ready = WaitForInitialState();
        if (ready || read_only_ || g1_control::StopRequested()) return ready;
        InitializeSession();
        g1_control::LoopScheduler<> cycle(g1_control::kControlPeriod, start);
        next_warning = Clock::now();
        int result = 0;
        try {
            while (!g1_control::StopRequested()) {
                if (!ReadState()) { result = 1; break; }
                char key = 0;
                if (!ReadInput(key)) { result = 1; break; }
                if (key == ' ') break;
                HandleInput(key);
                UpdatePolicy(key);
                PublishTargets();
                cycle.Sleep();
            }
        } catch (const std::exception& error) {
            log.Line() << "Controller error: " << error.what() << '\n';
            result = 1;
        }
        Stop();
        return result;
    }

private:
    int WaitForInitialState() {
        log.Line() << "Waiting for G1 29DOF lowstate (10 s timeout)...\n";
        const auto deadline = Clock::now() + g1_control::kStartupTimeout;
        while (!g1_control::StopRequested() && !Snapshot(state)) {
            if (Clock::now() >= deadline) {
                log.Line() << "No valid lowstate. Check scene/robot, DDS domain and interface. " << state_receiver_.Error() << "\n";
                return 1;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        if (g1_control::StopRequested()) return 0;
        // C++ bridge publishes 4 for 23DOF, 5 for 29DOF. Python bridge
        // leaves mode_machine at 0, so its scene must be checked by the user.
        if ((hardware_ && state.mode_machine() != 5) || (!hardware_ && state.mode_machine() != 0 && state.mode_machine() != 5)) {
            log.Line() << "Expected 29DOF mode_machine 5, received "
                      << unsigned(state.mode_machine()) << ".\n";
            return 1;
        }
        if (read_only_) {
            log.Line() << "Read-only hardware check: valid CRC, G1 mode=5, joints/IMU/motors OK. No control service release or LowCmd publication.\n";
            return 0;
        }
        return 0;
    }

    void InitializeSession() {
        command.mode_pr() = 0;  // pitch/roll joint coordinates, not A/B motors
        command.mode_machine() = state.mode_machine();
        keyboard = std::make_unique<g1_control::Keyboard>();
        control_input = gamepad_ ? &gamepad_->state : unitree_remote_ ? &remote_.state : nullptr;
        if (unitree_remote_) log.Line() << "Unitree remote: Start=real handover, X=reset, A=policy, Select=stop/exit. Direct stick velocity; R1 not required.\n";
        log.Line() << "Ready. DRY RUN / command output OFF. P: reset preview (" << reset_seconds
                  << " s), R: policy preview; 9/Start: real handover, then P required again. Space / Ctrl+C: exit.\n";
        start = Clock::now();
        if(policy_) policy_worker=std::make_unique<PolicyWorker>(
            [&](const PolicyObservation& input,const auto& velocity) {return policy_->Step(input.state,velocity);},
            [&] {policy_->Reset();},false,
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(policy_->dt)));
        log.Line() << "Dry sequence: P -> wait -> R, no 9. Real sequence: 9 -> P -> wait -> R.\n"
                  << "W/S: forward +/-0.1 m/s, A/D: lateral +/-0.1 m/s, Q/E: yaw +/-0.1 rad/s, X: zero velocity.\n";
    }

    bool ReadState() {
        if(output_enabled && command_monitor_ && command_monitor_->Conflict())
            throw std::runtime_error("Another LowCmd writer published after handover");
        if (!Snapshot(state) || state.mode_machine() != command.mode_machine()) {
            log.Line() << "Lowstate stale/invalid or robot mode changed; stopping. " << state_receiver_.Error() << "\n";
            return false;
        }
        if (hardware_ && !output_enabled && Clock::now() >= next_warning) {
            std::string warning;
            try { CheckHardwareState(state); }
            catch (const std::exception& error) { warning = error.what(); }
            if (warning != preview_warning) {
                log.Line() << (warning.empty() ? "Hardware checks recovered.\n" :
                    "Dry-run warning; real output blocked: " + warning + "\n");
                preview_warning = warning;
            }
            next_warning = Clock::now() + std::chrono::seconds(1);
        }
        return true;
    }

    bool ReadInput(char& key) {
        try {
            key = keyboard->Read(!control_input);
            if(gamepad_) {
                gamepad_->Poll();
            }
            if (unitree_remote_) remote_.Update(state.wireless_remote());
            if (control_input) {
                key = g1_control::SelectAction(key, control_input->Action());
            }
        } catch (const std::exception& error) {
            log.Line() << error.what() << '\n';
            return false;
        }
        return true;
    }

    void HandleInput(char key) {
        if (key == '9' && !output_enabled) TakeOver();
        if (key == 'p' || key == 'P') BeginReset();
        if ((key == 'r' || key == 'R') && !policy_active) TryStartPolicy();
    }

    void TakeOver() {
        if (unitree_remote_ && !g1_control::InputReady(control_input)) {
            log.Line() << "Center Unitree sticks before Start/9 handover.\n";
        } else {
            if (hardware_) {
                // Validate current real state before changing the official service.
                if (!Snapshot(state)) throw std::runtime_error("No fresh state before handover");
                CheckHardwareState(state);
                InitMotorPublisher();
                g1_control::ReleaseMotionService();
                publishing = true; // Ensure faults after confirmed handover enter damping stop.
                output_armed_ = true;
                // Start damping immediately after confirmed service release.
                for(auto& motor:command.motor_cmd()) motor={};
                g1_control::SetDampingTargets(command, true, hardware_->stop_kd);
                command.crc()=g1_hardware::Crc(command);
                if(!WriteMotor(command)) throw std::runtime_error("Initial damping publication failed");
                command_monitor_->Arm(*publisher_);
                if (!Snapshot(state)) throw std::runtime_error("No valid state after releasing motion service");
                CheckHardwareState(state);
            }
            // Never apply an old dry-run target when enabling real output.
            active = false;
            holding = false;
            policy_active = false;
            velocity.fill(0);
            if (policy_worker) policy_worker->Pause();
            output_enabled = true;
            log.Line() << "Command output ENABLED. Dry preview cleared; damping wait. Press P to reset from measured pose.\n";
        }
    }

    void BeginReset() {
        policy_active = false;
        velocity.fill(0);
        if (policy_worker) policy_worker->Pause();
        start_angles = g1_control::JointPositions(state);
        start = Clock::now();
        active = true;
        holding = false;
        log.Line() << "Reset target generated from current joint angles."
                  << (output_enabled ? " Publishing enabled.\n" : " Output disabled; targets only.\n");
    }

    void TryStartPolicy() {
        const bool pose_ready = g1_control::NearPose(g1_control::JointPositions(state), policy_->DefaultDds());
        if (!active || !holding || (output_enabled && !pose_ready)) {
            log.Line() << "Policy requires completed reset. Dry run: P -> wait -> R. Real output also requires measured pose near training pose.\n";
        } else if(!g1_control::InputReady(control_input)) {
            log.Line()<<(unitree_remote_ ? "Center sticks before A/policy entry.\n" : (control_input->HoldRequired() ? "Center sticks and release RB before policy entry.\n" : "Center sticks before policy entry.\n"));
        } else {
            policy_target=policy_->DefaultDds();
            policy_worker->Begin(hardware_ && output_enabled);
            velocity.fill(0);
            policy_active = true;
            if(control_input) control_input->BeginPolicy();
            log.Line() << (output_enabled ? "REAL" : "DRY RUN") << " locomotion policy ON; velocity = [0, 0, 0]. P: reset, Space / Ctrl+C: exit.\n";
        }
    }

    void UpdatePolicy(char key) {
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
            const auto previous_velocity = velocity;
            if(control_input) velocity=policy_->GamepadVelocity(control_input->ControlVelocity(key));
            policy_->ClampVelocity(velocity);
            if (changed || (control_input && velocity != previous_velocity && (Clock::now() >= next_input_log_ || velocity == std::array<float,3>{0,0,0}))) {
                log.Line() << "Velocity: " << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << '\n';
                next_input_log_ = Clock::now() + std::chrono::milliseconds(200);
            }
            PolicyObservation input;
            for (std::size_t i = 0; i < kJointCount; ++i) {
                input.state.q[i] = state.motor_state()[i].q();
                input.state.dq[i] = state.motor_state()[i].dq();
            }
            std::copy_n(state.imu_state().quaternion().begin(), 4, input.state.quaternion.begin());
            std::copy_n(state.imu_state().gyroscope().begin(), 3, input.state.omega.begin());
            policy_worker->Submit(input,velocity);
            if(auto latest=policy_worker->Latest()) policy_target=latest->target;
            // Hold the default pose until the first result. Latest() checks
            // both target and input age, including the first-result deadline.
        }
    }

    void PublishTargets() {
        telemetry_.PublishEnable(output_enabled);
        if (!active) {
            if (hardware_ && output_enabled) {
                g1_control::SetDampingTargets(command, true, hardware_->stop_kd);
                command.crc() = g1_hardware::Crc(command);
                if (!WriteMotor(command)) throw std::runtime_error("Damping DDS publication failed");
                publishing = true;
            }
            return;  // do not publish commands until P is pressed after power-on
        }
        const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
        const double u = g1_control::ResetProgress(elapsed, reset_seconds);
        const auto positions = policy_active ? policy_target :
            g1_control::InterpolateResetPose(start_angles, config_.default_pos, u);
        g1_control::SetPositionTargets(command, positions,
            policy_active ? policy_->kp : config_.kp, policy_active ? policy_->kd : config_.kd);
        if (hardware_ && output_enabled) {
            if (!Snapshot(state)) throw std::runtime_error("Hardware state invalid before command publication: " + state_receiver_.Error());
            hardware_->PrepareCommand(command);
        }
        command.crc() = g1_hardware::Crc(command);
        if (output_enabled) telemetry_.PublishTarget(command);
        else telemetry_.PublishPreview(command);
        if (output_enabled) {
            if (!WriteMotor(command)) throw std::runtime_error("LowCmd DDS publication failed");
            publishing = true;
        }
        if (u == 1.0 && !holding) {
            log.Line() << "Target interpolation complete."
                      << (output_enabled ? " Holding default pose.\n" : " Output disabled; target only.\n");
            holding = true;
        }
    }

    void ReleaseOutput() {
        if (!publishing) return;
        // Hardware: enabled damping for a bounded interval; never assume mode=0 is damping.
        g1_control::SetDampingTargets(command, hardware_ != nullptr,
                                      hardware_ ? hardware_->stop_kd : 0);
        const auto deadline = Clock::now() + std::chrono::duration<double>(hardware_ ? hardware_->damping_seconds : 0);
        do {
            command.crc() = g1_hardware::Crc(command);
            if (!WriteMotor(command) && hardware_)
                log.Line() << "Damping command DDS publication failed; execution is unconfirmed.\n";
            if (hardware_) std::this_thread::sleep_for(g1_control::kControlPeriod);
        } while (Clock::now() < deadline);
        publishing = false;
    }

    void Stop() {
        // Invalidate in-flight work, damp outputs, then join the policy thread.
        if (policy_worker) policy_worker->Pause();
        ReleaseOutput();
        telemetry_.PublishEnable(false);
        policy_worker.reset();
        keyboard.reset();
    }

    void CheckHardwareState(const LowState_& state) const {
        hardware_->CheckState(state, unitree_remote_ ? 1u << 3 : 1u << 8);
    }
    void InitMotorPublisher() {
        if (publisher_) return;
        publisher_ = std::make_unique<g1_control::MotorPublisher>("rt/lowcmd");
        publisher_->InitChannel();
    }
    bool WriteMotor(const LowCmd_& command) {
        if (!publisher_ || (hardware_ && !output_armed_))
            throw std::runtime_error("Actuator publication requires explicit 9/Start handover");
        return publisher_->Write(command);
    }
    g1_gamepad::Device* gamepad_ = nullptr;
    g1_policy::Policy* policy_ = nullptr;
    void ValidateState(const LowState_& state) const {
        if (hardware_) {
            if (read_only_ || output_armed_) CheckHardwareState(state);
            else g1_hardware::CheckTelemetryState(state);
        }
        for (std::size_t i = 0; i < kJointCount; ++i)
            if (!std::isfinite(state.motor_state()[i].q()) || !std::isfinite(state.motor_state()[i].dq()))
                throw std::runtime_error("Non-finite joint state");
    }
    bool Snapshot(LowState_& state) {
        return state_receiver_.ReadFresh(state,
            std::chrono::duration<double, std::milli>(hardware_ ? hardware_->timeout_ms : 500));
    }
    const ResetConfig config_;
    const g1_hardware::Limits* hardware_ = nullptr;
    bool read_only_ = false;
    bool unitree_remote_ = false;
    g1_unitree_remote::Input remote_;
    Clock::time_point next_input_log_{};
    std::atomic<bool> output_armed_{false};
    std::unique_ptr<g1_control::CommandMonitor> command_monitor_;
    std::unique_ptr<g1_control::MotorPublisher> publisher_;
    g1_control::MonitorPublisher telemetry_;
    g1_control::DdsStateCache<LowState_> state_receiver_;
    // Session data belongs to the controller; DDS callbacks only touch the state cache.
    struct PolicyObservation {
        g1_policy::State state;
        uint64_t sequence=0; // No frame gating: DWaQ samples LowState every policy tick.
    };
    using PolicyWorker=g1_inference::PolicyWorker<PolicyObservation,g1_policy::Joints>;
    std::unique_ptr<g1_control::Keyboard> keyboard;
    g1_gamepad::State* control_input = nullptr;
    g1_control::AsyncLog log;
    LowState_ state{};
    LowCmd_ command{};
    std::array<float, kJointCount> start_angles{};
    g1_policy::Joints policy_target{};
    std::array<float, 3> velocity{};
    Clock::time_point start{}, next_warning{};
    std::string preview_warning;
    double reset_seconds = 3;
    bool active = false, output_enabled = false, publishing = false;
    bool holding = false, policy_active = false;
    // Join inference before destroying session data referenced by its callbacks.
    std::unique_ptr<PolicyWorker> policy_worker;

};

}  // namespace

namespace g1_dwaq_app {
namespace {
int CheckPolicy(const Options& options) {
    g1_policy::Policy policy(options.model, options.config);
    g1_policy::State state;
    state.q = policy.DefaultDds();
    const auto target = policy.Step(state, {0, 0, 0});
    std::cout << "Policy OK: " << policy.Interface() << ", dt=" << policy.dt << " s\nDDS targets:";
    for (float q : target) std::cout << ' ' << q;
    std::cout << '\n';
    return 0;
}

int CheckJoystick(const Options& options) {
    g1_gamepad::Device gamepad(options.joystick.empty() ? "/dev/input/js0" : options.joystick, true, false);
    std::cout << "Gamepad: " << gamepad.Description() << '\n';
    g1_gamepad::CheckInput(gamepad, g1_control::stop_requested);
    return 0;
}

int RunController(const Options& options) {
    const bool read_only = options.mode == Mode::ReadOnly;
    std::unique_ptr<g1_gamepad::Device> gamepad;
    if (!options.joystick.empty()) {
        gamepad = std::make_unique<g1_gamepad::Device>(options.joystick, true, false);
        std::cout << "Gamepad: " << gamepad->Description() << '\n';
    }
    const auto config = LoadConfig(options.config);
    std::cout << "Loaded locomotion configuration: " << options.config << '\n';
    std::unique_ptr<g1_policy::Policy> policy;
    if (!read_only) {
        policy = std::make_unique<g1_policy::Policy>(options.model, options.config);
        std::cout << "Loaded locomotion policy, period " << policy->dt << " s\n";
    }
    std::unique_ptr<g1_hardware::Limits> hardware_limits;
    if (options.hardware) {
        hardware_limits = std::make_unique<g1_hardware::Limits>(YAML::LoadFile(options.config));
        for (int i = 0; i < 29; ++i)
            if (config.default_pos[i] < hardware_limits->lower[i] + 0.01f ||
                config.default_pos[i] > hardware_limits->upper[i] - 0.01f)
                throw std::runtime_error("Reset pose outside hardware joint limits");
    }
    unitree::robot::ChannelFactory::Instance()->Init(options.domain, options.interface);
    DwaqController controller(config, hardware_limits.get(), read_only, options.unitree_remote,
                          policy.get(), gamepad.get());
    return controller.Run(options.reset_seconds);
}
}  // namespace

int Run(const Options& options) {
    g1_control::InstallSignalHandlers();
    if (options.mode == Mode::CheckPolicy) return CheckPolicy(options);
    if (options.mode == Mode::CheckJoystick) return CheckJoystick(options);
    return RunController(options);
}
}  // namespace g1_dwaq_app
