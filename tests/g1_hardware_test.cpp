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
        reject([](State& s){ s.motor_state()[0].q() = 100; });
        reject([](State& s){ s.motor_state()[0].dq() = 100; });
        std::array<float,29> previous{};
        unitree_hg::msg::dds_::LowCmd_ command{};
        for (int i = 0; i < 29; ++i) {
            previous[i] = state.motor_state()[i].q();
            command.motor_cmd()[i].q() = 100;
            command.motor_cmd()[i].kp() = 40;
        }
        limits.LimitCommand(command, state, previous, 0.002);
        for (int i = 0; i < 29; ++i)
            Require(std::abs(command.motor_cmd()[i].q()-state.motor_state()[i].q()) <= 0.02001f, "Target slew limit failed");
        command.motor_cmd()[0].tau() = 10000;
        bool rejected = false;
        try { limits.LimitCommand(command, state, previous, 0.002); } catch (const std::runtime_error&) { rejected = true; }
        Require(rejected, "Estimated torque limit failed");
        std::cout << "Hardware CRC, mode, fault, temperature, IMU, remote stop and command bounds passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
