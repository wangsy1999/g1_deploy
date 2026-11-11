#pragma once

#include "g1_hybrid_switch.h"
#include "g1_locomotion_policy.h"
#include "g1_parkour_control_policy.h"
#include <memory>

namespace g1_hybrid {
class Policy final : public g1_parkour_app::ControlPolicy {
public:
    using Targets = g1_parkour_app::MotorTargets;
    using Observation = g1_parkour_app::ControlObservation;

    explicit Policy(const std::string& config)
        : selector_(LoadSwitchSettings(config)) {
        const auto cfg = YAML::LoadFile(config);
        dwaq_ = std::make_unique<g1_policy::Policy>(g1_config::ResolvePath(config, cfg["policies"]["dwaq"].as<std::string>()));
        parkour_ = std::make_unique<g1_parkour::Policy>(g1_config::ResolvePath(config, cfg["policies"]["parkour"].as<std::string>()));
        if (std::abs(dwaq_->dt - .02) > 1e-8 || std::abs(parkour_->dt - .02) > 1e-8)
            throw std::runtime_error("Hybrid requires both policies to run at 20ms");
        parkour_kp_ = g1_joints::ToMotorOrder(parkour_->kp, parkour_->ids);
        parkour_kd_ = g1_joints::ToMotorOrder(parkour_->kd, parkour_->ids);
        const auto warmup = cfg["warmup_steps"];
        WarmUp(warmup.IsDefined() ? warmup.as<int>() : 10);
    }

    const char* Name() const override { return "Hybrid (DWaQ stand/backward, Parkour forward)"; }
    const g1_parkour::Policy& CameraPolicy() const override { return *parkour_; }
    Targets DefaultTargets() const override { return {dwaq_->DefaultDds(), dwaq_->kp, dwaq_->kd}; }
    Mode SelectedMode() const { return mode_; } // read only on the inference owner thread

    // 仅在启动、控制线程尚未运行时调用。预热结果不会交给控制循环。
    void WarmUp(int steps) {
        if (steps < 1) throw std::runtime_error("warmup_steps must be a positive integer");
        Reset();
        g1_policy::State standing;
        standing.q = dwaq_->DefaultDds();
        g1_parkour::Observation camera;
        camera.q = parkour_->neutral;
        camera.gravity = {0, 0, -1};
        camera.depth.fill(2.5f);
        try {
            for (int tick = 0; tick < steps; ++tick) {
                // 分别运行 DWaQ、深度编码器和 Parkour 动作网络，不通过模式选择器。
                dwaq_->Step(standing, {0, 0, 0});
                parkour_->Step(camera, {0, 0, 0});
            }
        } catch (...) {
            Reset();
            throw;
        }
        // 正式观测必须重新建立历史，不能混入预热时的虚拟状态和动作。
        Reset();
    }

    void ClampVelocity(std::array<float, 3>& velocity) const override {
        for (float value : velocity)
            if (!std::isfinite(value)) throw std::runtime_error("Non-finite hybrid command");
        dwaq_->ClampVelocity(velocity);
        velocity[0] = std::min(velocity[0], 1.f);
        velocity[1] = 0;
        velocity[2] = std::clamp(velocity[2], -1.f, 1.f);
    }

    // 两套策略都更新历史并推理 → 只选择一套角度和 PD 增益交给控制循环。
    Targets Step(const Observation& observation, const std::array<float, 3>& requested_velocity) override {
        auto velocity = requested_velocity;
        ClampVelocity(velocity);
        mode_ = selector_.Update(velocity[0], dt_);

        // DWaQ 始终使用自己的电机顺序、pelvis IMU 和有符号前后速度。
        const Targets dwaq_targets{
            dwaq_->Step(DwaqState(observation.lowstate), velocity), dwaq_->kp, dwaq_->kd};

        // Parkour 不接受后退指令；后退期间也用最新实测状态和零前进指令维护历史。
        auto parkour_velocity = velocity;
        parkour_velocity[0] = std::clamp(parkour_velocity[0], 0.f, 1.f);
        const Targets parkour_targets{
            parkour_->Step(observation, parkour_velocity), parkour_kp_, parkour_kd_};

        // 未选中的预测不发布；切换不重置历史，也不做角度或增益混合。
        return mode_ == Mode::Dwaq ? dwaq_targets : parkour_targets;
    }

    void Reset() override {
        dwaq_->Reset();
        parkour_->Reset();
        selector_.Reset();
        mode_ = Mode::Dwaq;
    }

    static g1_policy::State DwaqState(const unitree_hg::msg::dds_::LowState_& lowstate) {
        g1_policy::State state;
        for (int motor = 0; motor < 29; ++motor) {
            state.q[motor] = lowstate.motor_state()[motor].q();
            state.dq[motor] = lowstate.motor_state()[motor].dq();
        }
        std::copy_n(lowstate.imu_state().quaternion().begin(), 4, state.quaternion.begin());
        std::copy_n(lowstate.imu_state().gyroscope().begin(), 3, state.omega.begin());
        return state;
    }

private:
    static SwitchSettings LoadSwitchSettings(const std::string& config) {
        const auto cfg = YAML::LoadFile(config)["switching"];
        SwitchSettings settings;
        settings.forward_enter_mps = cfg["forward_enter_mps"].as<float>();
        settings.forward_exit_mps = cfg["forward_exit_mps"].as<float>();
        settings.forward_confirm_seconds = cfg["forward_confirm_seconds"].as<double>();
        settings.Validate();
        return settings;
    }

    static constexpr double dt_ = .02;
    ModeSelector selector_;
    Mode mode_ = Mode::Dwaq;
    std::unique_ptr<g1_policy::Policy> dwaq_;
    std::unique_ptr<g1_parkour::Policy> parkour_;
    g1_parkour::Joints parkour_kp_{}, parkour_kd_{};
};
}  // namespace g1_hybrid
