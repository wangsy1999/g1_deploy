#include "g1_hardware.h"
#include <functional>
#include <iostream>
#include <limits>

using State = unitree_hg::msg::dds_::LowState_;
void Require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::runtime_error("Expected locomotion YAML");
        const auto cfg = YAML::LoadFile(argv[1]);
        g1_hardware::Limits limits(cfg);
        State state{};
        state.mode_machine() = 5;
        state.imu_state().quaternion() = {1,0,0,0};
        for (int i = 0; i < 29; ++i)
            state.motor_state()[cfg["joint_ids_map"][i].as<int>()].q() = cfg["default_joint_pos"][i].as<float>();
        state.crc() = g1_hardware::Crc(state);
        limits.CheckState(state);
        g1_hardware::CheckTelemetryState(state);
        // Finite measured position/velocity excursions do not cause automatic exits.
        auto preview_state = state;
        preview_state.motor_state()[9].q() = limits.lower[9] - 0.02f;
        preview_state.crc() = g1_hardware::Crc(preview_state);
        g1_hardware::CheckTelemetryState(preview_state);
        limits.CheckState(preview_state);
        auto reject_preview = [&](const std::function<void(State&)>& change, bool recalc = true) {
            auto altered = state; change(altered);
            if (recalc) altered.crc() = g1_hardware::Crc(altered);
            bool rejected = false;
            try { g1_hardware::CheckTelemetryState(altered); } catch (const std::runtime_error&) { rejected = true; }
            Require(rejected, "Invalid telemetry accepted by preview");
        };
        reject_preview([](State& s){ s.crc() ^= 1; }, false);
        reject_preview([](State& s){ s.mode_machine() = 0; });
        reject_preview([](State& s){ s.motor_state()[0].q() = std::numeric_limits<float>::quiet_NaN(); });
        reject_preview([](State& s){ s.imu_state().quaternion() = {0,0,0,0}; });
        reject_preview([](State& s){ s.imu_state().gyroscope()[0] = std::numeric_limits<float>::infinity(); });
        auto reject = [&](const std::function<void(State&)>& change, bool recalc = true) {
            auto altered = state; change(altered);
            if (recalc) altered.crc() = g1_hardware::Crc(altered);
            bool rejected = false;
            try { limits.CheckState(altered); } catch (const std::runtime_error&) { rejected = true; }
            Require(rejected, "Invalid hardware state accepted");
        };
        reject([](State& s){ s.crc() ^= 1; }, false);
        reject([](State& s){ s.mode_machine() = 0; });
        reject([](State& s){ s.motor_state()[28].motorstate() = 1; });
        reject([](State& s){ s.motor_state()[0].temperature()[0] = 100; });
        reject([](State& s){ s.motor_state()[0].dq() = std::numeric_limits<float>::quiet_NaN(); });
        reject([](State& s){ s.imu_state().quaternion() = {0,1,0,0}; });
        reject([](State& s){ s.imu_state().quaternion() = {0,0,0,0}; });
        reject([](State& s){ s.wireless_remote()[3] = 1; }); // A bit
        auto original_remote = state;
        original_remote.wireless_remote()[3] = 1; // A is policy in explicit Unitree remote mode.
        original_remote.crc() = g1_hardware::Crc(original_remote);
        limits.CheckState(original_remote,1u<<3);
        original_remote.wireless_remote()[2] = 1u<<3; // Select must still stop even while A is held.
        original_remote.crc() = g1_hardware::Crc(original_remote);
        bool select_rejected=false;
        try {limits.CheckState(original_remote,1u<<3);}catch(const std::runtime_error&){select_rejected=true;}
        Require(select_rejected,"Unitree Select failed to stop real control");
        reject([](State& s){ s.motor_state()[0].q() = std::numeric_limits<float>::quiet_NaN(); });
        auto outside = state;
        outside.motor_state()[0].q() = 100;
        outside.motor_state()[0].dq() = 100;
        outside.crc() = g1_hardware::Crc(outside);
        limits.CheckState(outside);
        std::array<float,29> previous{};
        unitree_hg::msg::dds_::LowCmd_ command{};
        for (int i = 0; i < 29; ++i) {
            previous[i] = state.motor_state()[i].q();
            command.motor_cmd()[i].q() = 100;
            command.motor_cmd()[i].kp() = 40;
        }
        limits.LimitCommand(command, previous, 0.002);
        for (int i = 0; i < 29; ++i)
            Require(std::abs(command.motor_cmd()[i].q()-state.motor_state()[i].q()) <= 0.02001f, "Target slew limit failed");
        command.motor_cmd()[0].tau() = std::numeric_limits<float>::quiet_NaN();
        bool rejected = false;
        try { limits.LimitCommand(command, previous, 0.002); } catch (const std::runtime_error&) { rejected = true; }
        Require(rejected, "Non-finite command accepted after removing estimated torque check");
        command.motor_cmd()[0].tau() = 0;
        previous[4] = limits.lower[4] - 0.02f;
        command.motor_cmd()[4].q() = previous[4];
        limits.LimitCommand(command, previous, 0.002);
        Require(command.motor_cmd()[4].q() >= limits.lower[4]+0.01f &&
                command.motor_cmd()[4].q() <= limits.upper[4]-0.01f,
                "Outgoing target escaped bounds after an out-of-range initial measurement");
        std::cout << "Hardware CRC, mode, fault, temperature, IMU, remote stop and command bounds passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
