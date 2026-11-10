#include "g1_hardware.h"
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

int main() {
    try {
        using Command = unitree_hg::msg::dds_::LowCmd_;
        Command cmd{};
        // Out-of-range finite targets and abrupt updates must pass through exactly.
        for (int i = 0; i < 29; ++i) {
            auto& motor = cmd.motor_cmd()[i];
            motor.q() = i % 2 ? -4.f : 4.f;
            motor.dq() = .5f; motor.kp() = 50.f; motor.kd() = 2.f; motor.tau() = 1.f;
        }
        const Command original = cmd;
        g1_hardware::CheckCommand(cmd);
        if (std::memcmp(&cmd, &original, sizeof(cmd)) != 0)
            throw std::runtime_error("Command validation changed a target");
        YAML::Node root;
        auto hardware = root["hardware"];
        hardware["state_timeout_ms"] = 100;
        hardware["max_tilt_rad"] = 1.;
        hardware["max_temperature_c"] = 70;
        hardware["stop_kd"] = 1.;
        hardware["damping_seconds"] = .5;
        for (int i = 0; i < 29; ++i) {
            hardware["joint_lower"].push_back(-1.f);
            hardware["joint_upper"].push_back(1.f);
        }
        g1_hardware::Limits disabled(root); // Missing switch must preserve current behavior.
        disabled.PrepareCommand(cmd);
        for (int i = 0; i < 29; ++i)
            if (cmd.motor_cmd()[i].q() != original.motor_cmd()[i].q())
                throw std::runtime_error("Default position limits are not disabled");
        hardware["limit_target_position"] = false;
        g1_hardware::Limits explicit_disabled(root);
        explicit_disabled.PrepareCommand(cmd);
        for (int i = 0; i < 29; ++i)
            if (cmd.motor_cmd()[i] != original.motor_cmd()[i])
                throw std::runtime_error("Disabled limits changed a command");
        hardware["limit_target_position"] = true;
        g1_hardware::Limits enabled(root);
        enabled.PrepareCommand(cmd);
        for (int i = 0; i < 29; ++i) {
            const auto& m = cmd.motor_cmd()[i];
            if (m.q() != (i % 2 ? -1.f : 1.f) || m.dq() != .5f ||
                m.kp() != 50.f || m.kd() != 2.f || m.tau() != 1.f)
                throw std::runtime_error("Position limits changed unexpected fields");
        }
        // Even with position limits enabled, a large in-range jump is immediate.
        cmd.motor_cmd()[0].q() = -.9f;
        enabled.PrepareCommand(cmd);
        if (cmd.motor_cmd()[0].q() != -.9f)
            throw std::runtime_error("Target slew rate was limited");
        for (auto invalid : {std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::infinity(),
                             -std::numeric_limits<float>::infinity()}) {
            for (int field = 0; field < 5; ++field) {
                Command bad = original;
                auto& m = bad.motor_cmd()[28];
                float* fields[] = {&m.q(), &m.dq(), &m.kp(), &m.kd(), &m.tau()};
                *fields[field] = invalid;
                bool rejected = false;
                try { enabled.PrepareCommand(bad); }
                catch (const std::runtime_error&) { rejected = true; }
                if (!rejected) throw std::runtime_error("Non-finite command accepted");
            }
        }
        std::cout << "Configurable position limits, unrestricted slew rate and non-finite checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
