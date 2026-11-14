#include "g1_parkour_options.h"
#include <iostream>
#include <net/if.h>
#include <stdexcept>

namespace g1_parkour_app {
Options ParseOptions(int argc, char** argv) {
    Options options;
    options.config = G1_PARKOUR_DEFAULT_CONFIG;
    options.program = argv[0];
    g1_control::OptionState state;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (g1_control::ParseRuntimeOption(argc, argv, i, options, state)) continue;
        auto value = [&] { return g1_control::OptionValue(argc, argv, i, arg); };
        if (arg == "--joystick") options.joystick = value();
        else if (arg == "--check-policy") options.mode = Mode::CheckPolicy;
        else if (arg == "--help") {
            options.mode = Mode::Help;
            return options;
        } else throw std::invalid_argument("Unknown argument: " + arg);
    }
    g1_control::ValidateRuntimeOptions(options);
    if (options.hardware && (options.interface == "lo" || !if_nametoindex(options.interface.c_str()) ||
                             !options.joystick.empty()))
        throw std::invalid_argument("Hardware requires a non-loopback robot interface and the Unitree remote");
    return options;
}

void PrintUsage(const std::string&) {
    std::cout<<"g1_parkour --sim [--domain 1 --interface lo]\n"
        <<"g1_parkour --hardware --domain 0 --interface ROBOT_IF\n"
        <<"Common: --config JSON --check-policy; model paths, reset_seconds and hardware limits come from JSON\n"
        <<"Simulation F710: --joystick DEVICE; Start enable, X reset, A policy, Y zero, B stop. Hardware: Unitree Start enable, X reset, A policy, Select stop; direct sticks.\n"
        <<"Startup is dry-run. p -> wait -> r previews targets; 9/Start hands over, clears preview, then p -> r again.\n"
        <<"Verify camera pose, torso IMU axes and joint signs before Start handover.\n";
}
}  // namespace g1_parkour_app
