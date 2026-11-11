#include "g1_dwaq_app.h"
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace g1_dwaq_app {
Options ParseOptions(int argc, char** argv) {
    Options options;
    options.config = G1_POLICY_DEFAULT_CONFIG;
    options.program = argv[0];
    g1_control::OptionState state;
    bool check_joystick = false, check_policy = false;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if (g1_control::ParseRuntimeOption(argc, argv, i, options, state)) continue;
        auto value = [&] { return g1_control::OptionValue(argc, argv, i, argument); };
        if (argument == "--hardware-read-only") {
            g1_control::SelectMode(true, options, state);
            options.mode = Mode::ReadOnly;
        } else if (argument == "--joystick") options.joystick = value();
        else if (argument == "--model") {
            options.model = value();
            state.named_values = true;
        } else if (argument == "--unitree-remote") options.unitree_remote = true;
        else if (argument == "--check-joystick") check_joystick = true;
        else if (argument == "--check-policy") check_policy = true;
        else if (argument == "--help") {
            options.mode = Mode::Help;
            return options;
        } else if (argument.rfind("--", 0) == 0) throw std::invalid_argument("Unknown argument: " + argument);
        else positional.push_back(argument);
    }
    if (options.unitree_remote && (!options.hardware || options.mode == Mode::ReadOnly ||
                                  check_joystick || !options.joystick.empty()))
        throw std::invalid_argument("--unitree-remote requires --hardware and cannot be combined with F710 or input/read-only checks.");
    if (options.hardware && check_joystick)
        throw std::invalid_argument("Use --check-joystick without --hardware; input diagnostics never connect to DDS.");
    if (options.hardware && !options.joystick.empty())
        throw std::invalid_argument("F710 is simulation-only; hardware defaults to the Unitree remote.");
    if (check_joystick && check_policy)
        throw std::invalid_argument("Choose one diagnostic mode");
    if (check_joystick) {
        if (!positional.empty() || state.named_values)
            throw std::invalid_argument("Use --check-joystick [--joystick DEVICE] without control arguments");
        options.mode = Mode::CheckJoystick;
        return options;
    }
    if (!positional.empty() && state.named_values)
        throw std::invalid_argument("Do not combine legacy positional arguments with named control values");
    if (check_policy) {
        if (options.mode == Mode::ReadOnly) throw std::invalid_argument("Choose one diagnostic mode");
        if (positional.size() > 2) throw std::invalid_argument("Usage: --check-policy --config JSON [--model ONNX]");
        options.mode = Mode::CheckPolicy;
        if (!positional.empty()) {
            const bool model_first = std::filesystem::path(positional[0]).extension() == ".onnx";
            if (model_first) {
                options.model = positional[0];
                if (positional.size() > 1) options.config = positional[1];
            } else {
                options.config = positional[0];
                if (positional.size() > 1) options.model = positional[1];
            }
        }
        return options;
    }
    if (positional.size() > 5) {
        options.mode = Mode::Help;
        options.help_exit_code = 1;
        return options;
    }
    // Older scripts may still pass reset_seconds, interface, domain, config and model.
    if (!positional.empty()) {
        if (options.hardware && positional.size() < 3)
            throw std::invalid_argument("Legacy hardware usage requires reset_seconds, interface and domain 0");
        options.reset_seconds = g1_control::ParseNumber(positional[0]);
        if (positional.size() > 1) options.interface = positional[1];
        if (positional.size() > 2) options.domain = g1_control::ParseInteger(positional[2]);
        if (positional.size() > 3) options.config = positional[3];
        if (positional.size() > 4) options.model = positional[4];
    }
    g1_control::ValidateRuntimeOptions(options);
    if (options.hardware && options.mode == Mode::Control) options.unitree_remote = true;
    return options;
}

void PrintUsage(const std::string& program) {
    std::cout << "Usage: " << program << " --sim [--domain 1 --interface lo]\n"
              << program << " --hardware --domain 0 --interface ROBOT_IF\n"
              << "Common: --config JSON --model ONNX --reset-seconds 3\n"
              << "Read-only: --hardware-read-only --domain 0 --interface ROBOT_IF\n"
              << "Legacy positional syntax remains accepted: [reset_seconds] [interface] [domain] [config] [model]\n";
    std::cout << "Model override: --model ONNX\n"
              << "Offline inference: --check-policy [--config JSON] [--model ONNX]\n"
              << "F710: --joystick /dev/input/js0; input-only check: --check-joystick [--joystick DEVICE]\n"
              << "Hardware defaults to Unitree remote: Start enables, X reset, A policy, Select stop; --unitree-remote remains accepted.\n"
              << "Start enable, X reset, A policy; left stick XY/right stick X to move (simulation only, no RB required); Y zero, B exit.\n";
}
}  // namespace g1_dwaq_app
