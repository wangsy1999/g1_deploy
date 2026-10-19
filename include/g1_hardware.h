#pragma once

#include <array>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>

namespace g1_hardware {
// Same CRC algorithm and wire-object layout used by the official G1 SDK example.
template<class Message> uint32_t Crc(const Message& message) {
    static_assert(sizeof(Message) % 4 == 0);
    const auto* bytes = reinterpret_cast<const unsigned char*>(&message);
    uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < sizeof(Message)/4-1; ++i) {
        uint32_t word;
        std::memcpy(&word, bytes+i*4, 4);
        for (uint32_t mask = 0x80000000u; mask; mask >>= 1) {
            const bool high = (crc & 0x80000000u) != 0;
            crc <<= 1;
            if (high) crc ^= 0x04C11DB7u;
            if (word & mask) crc ^= 0x04C11DB7u;
        }
    }
    return crc;
}
struct Limits {
    std::array<float, 29> lower{}, upper{}, velocity{}, effort{};
    double timeout_ms = 100, max_tilt = 1.0, damping_seconds = 0.5;
    float temperature = 70, target_rate = 10, stop_kd = 1;
    explicit Limits(const YAML::Node& root) {
        const auto cfg = root["hardware"];
        if (!cfg) throw std::runtime_error("Missing hardware configuration");
        auto read = [&](const char* name, auto& result) {
            const auto node = cfg[name];
            if (!node.IsSequence() || node.size() != result.size())
                throw std::runtime_error(std::string("Expected 29 hardware ")+name+" values in DDS order");
            for (int i = 0; i < 29; ++i) {
                result[i] = node[i].as<float>();
                if (!std::isfinite(result[i])) throw std::runtime_error("Non-finite hardware limit");
            }
        };
        read("joint_lower", lower); read("joint_upper", upper);
        read("joint_velocity", velocity); read("joint_effort", effort);
        timeout_ms = cfg["state_timeout_ms"].as<double>();
        max_tilt = cfg["max_tilt_rad"].as<double>();
        temperature = cfg["max_temperature_c"].as<float>();
        target_rate = cfg["target_rate_rad_s"].as<float>();
        stop_kd = cfg["stop_kd"].as<float>();
        damping_seconds = cfg["damping_seconds"].as<double>();
        for (double v : {timeout_ms, max_tilt, double(temperature), double(target_rate), double(stop_kd), damping_seconds})
            if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Invalid hardware protection setting");
        if (timeout_ms > 500 || max_tilt >= 1.57 || damping_seconds > 5)
            throw std::runtime_error("Hardware protection setting outside supported range");
        for (int i = 0; i < 29; ++i)
            if (lower[i]+0.02f >= upper[i] || velocity[i] <= 0 || effort[i] <= 0)
                throw std::runtime_error("Invalid per-joint hardware limits");
    }
    void CheckState(const unitree_hg::msg::dds_::LowState_& state) const {
        if (state.crc() != Crc(state)) throw std::runtime_error("LowState CRC mismatch");
        if (state.mode_machine() != 5) throw std::runtime_error("Hardware requires unlocked G1 29DOF mode_machine=5");
        for (int i = 0; i < 29; ++i) {
            const auto& motor = state.motor_state()[i];
            if (motor.motorstate() != 0) throw std::runtime_error("Motor fault at DDS joint "+std::to_string(i));
            if (!std::isfinite(motor.q()) || !std::isfinite(motor.dq()) || !std::isfinite(motor.tau_est()))
                throw std::runtime_error("Non-finite motor state");
            if (motor.q() < lower[i] || motor.q() > upper[i] || std::abs(motor.dq()) > velocity[i])
                throw std::runtime_error("Motor position/velocity limit at DDS joint "+std::to_string(i));
            for (auto t : motor.temperature())
                if (t >= temperature) throw std::runtime_error("Motor overtemperature at DDS joint "+std::to_string(i));
        }
        double norm = 0;
        for (float v : state.imu_state().quaternion()) norm += double(v)*v;
        if (!std::isfinite(norm) || norm < 0.5 || norm > 1.5) throw std::runtime_error("Invalid hardware IMU quaternion");
        for (float v : state.imu_state().gyroscope())
            if (!std::isfinite(v)) throw std::runtime_error("Invalid hardware gyroscope");
        const auto& q = state.imu_state().quaternion();
        const double upright = 1 - 2*(double(q[1])*q[1]+double(q[2])*q[2])/norm;
        if (upright < std::cos(max_tilt)) throw std::runtime_error("Hardware tilt limit");
        // Official gamepad byte layout: 2 header bytes, then little-endian key bits.
        const auto& remote = state.wireless_remote();
        const unsigned keys = unsigned(remote[2]) | (unsigned(remote[3]) << 8);
        if (keys & (1u << 8)) throw std::runtime_error("Remote A requested damping stop");
    }
    void LimitCommand(unitree_hg::msg::dds_::LowCmd_& cmd,
                      const unitree_hg::msg::dds_::LowState_& state,
                      std::array<float,29>& previous, double dt) const {
        for (int i = 0; i < 29; ++i) {
            auto& motor = cmd.motor_cmd()[i];
            if (!std::isfinite(motor.q())) throw std::runtime_error("Invalid hardware target");
            const float bound = std::min(target_rate, velocity[i])*dt;
            const float target = std::clamp(motor.q(), lower[i]+0.01f, upper[i]-0.01f);
            motor.q() = std::clamp(target, previous[i]-bound, previous[i]+bound);
            const auto& measured = state.motor_state()[i];
            const float predicted = motor.kp()*(motor.q()-measured.q()) - motor.kd()*measured.dq() + motor.tau();
            if (!std::isfinite(predicted) || std::abs(predicted) > effort[i])
                throw std::runtime_error("Estimated PD torque limit at DDS joint "+std::to_string(i));
            previous[i] = motor.q();
        }
    }
};
} // namespace g1_hardware
