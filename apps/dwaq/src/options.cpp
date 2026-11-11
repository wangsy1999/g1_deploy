#include "g1_dwaq_options.h"
#include <iostream>
#include <stdexcept>

namespace g1_dwaq_app {
Options ParseOptions(int argc, char** argv) {
    Options options;
    options.config = G1_POLICY_DEFAULT_CONFIG;
    options.program = argv[0];
    g1_control::OptionState state;
    bool check_joystick = false, check_policy = false;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (g1_control::ParseRuntimeOption(argc, argv, i, options, state)) continue;
        auto value = [&] { return g1_control::OptionValue(argc, argv, i, argument); };
        if (argument == "--hardware-read-only") {
            g1_control::SelectMode(true, options, state);
            options.mode = Mode::ReadOnly;
        } else if (argument == "--joystick") options.joystick = value();
        else if (argument == "--check-joystick") check_joystick = true;
        else if (argument == "--check-policy") check_policy = true;
        else if (argument == "--help") {
            options.mode = Mode::Help;
            return options;
        } else throw std::invalid_argument("Unknown argument: " + argument);
    }
    if (options.hardware && check_joystick)
        throw std::invalid_argument("Use --check-joystick without --hardware; input diagnostics never connect to DDS.");
    if (options.hardware && !options.joystick.empty())
        throw std::invalid_argument("F710 is simulation-only; hardware defaults to the Unitree remote.");
    if ((check_joystick && check_policy) ||
        (options.mode == Mode::ReadOnly && (check_joystick || check_policy)))
        throw std::invalid_argument("Choose one diagnostic mode");
    if (check_joystick) {
        if (state.connection_values)
            throw std::invalid_argument("Use --check-joystick [--joystick DEVICE] without DDS/config arguments");
        options.mode = Mode::CheckJoystick;
        return options;
    }
    if (check_policy) options.mode = Mode::CheckPolicy;
    g1_control::ValidateRuntimeOptions(options);
    return options;
}

void PrintUsage(const std::string& program) {
    std::cout << "Usage: " << program << " --sim [--domain 1 --interface lo]\n"
              << program << " --hardware --domain 0 --interface ROBOT_IF\n"
              << "Common: --config JSON (model path, reset_seconds and hardware limits are read from JSON)\n"
              << "Read-only: --hardware-read-only --domain 0 --interface ROBOT_IF\n";
    std::cout << "Offline inference: --check-policy [--config JSON]\n"
              << "F710: --joystick /dev/input/js0; input-only check: --check-joystick [--joystick DEVICE]\n"
              << "Hardware defaults to Unitree remote: Start enables, X reset, A policy, Select stop.\n"
              << "Start enable, X reset, A policy; left stick XY/right stick X to move (simulation only, no RB required); Y zero, B exit.\n";
}
}  // namespace g1_dwaq_app
