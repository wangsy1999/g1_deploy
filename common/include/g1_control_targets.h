#pragma once

#include <algorithm>
#include <array>
#include <cmath>

namespace g1_control {
// Callers validate the positive reset duration when parsing their own options.
inline double ResetProgress(double elapsed_seconds, double duration_seconds) {
    return std::clamp(elapsed_seconds / duration_seconds, 0.0, 1.0);
}
inline float InterpolateReset(float start, float target, double progress) {
    if (progress <= 0) return start;
    if (progress >= 1) return target;
    const double blend = progress * progress * (3.0 - 2.0 * progress);
    return static_cast<float>(start + blend * (target - start));
}

template<class State>
std::array<float, 29> JointPositions(const State& state) {
    std::array<float, 29> positions{};
    for (int joint = 0; joint < 29; ++joint) positions[joint] = state.motor_state()[joint].q();
    return positions;
}

template<std::size_t N>
bool NearPose(const std::array<float, N>& measured, const std::array<float, N>& expected, float tolerance = .35f) {
    for (std::size_t joint = 0; joint < N; ++joint)
        if (std::abs(measured[joint] - expected[joint]) > tolerance) return false;
    return true;
}

template<class Input>
bool InputReady(const Input* input) {
    return !input || (input->Ready() && input->Centered() &&
                     !(input->HoldRequired() && input->MovingEnabled()));
}

inline std::array<float, 29> InterpolateResetPose(const std::array<float, 29>& start,
                                                  const std::array<float, 29>& target, double progress) {
    std::array<float, 29> positions{};
    for (int joint = 0; joint < 29; ++joint)
        positions[joint] = InterpolateReset(start[joint], target[joint], progress);
    return positions;
}

// Positions and PD gains are all in DDS motor order at this boundary.
template<class Command>
void SetPositionTargets(Command& command, const std::array<float, 29>& positions,
                        const std::array<float, 29>& kp, const std::array<float, 29>& kd) {
    for (int joint = 0; joint < 29; ++joint) {
        auto& motor = command.motor_cmd()[joint];
        motor.mode() = 1;
        motor.q() = positions[joint];
        motor.kp() = kp[joint];
        motor.kd() = kd[joint];
        motor.dq() = motor.tau() = 0;
    }
}

// Populate only the 29 controlled joints. Preserve packet metadata and other motors.
// CRC calculation, publication and damping duration remain with each controller.
template<class Command>
void SetDampingTargets(Command& command, bool hardware, float stop_kd) {
    for (int i = 0; i < 29; ++i) {
        auto& motor = command.motor_cmd()[i];
        motor.mode() = hardware ? 1 : 0;
        motor.q() = motor.dq() = motor.kp() = motor.tau() = 0;
        motor.kd() = hardware ? stop_kd : 0;
    }
}
}  // namespace g1_control
