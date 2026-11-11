#pragma once

#include <string>
#include "g1_control_options.h"

namespace g1_parkour_app {
enum class Mode { Control, CheckPolicy, Help };

struct Options : g1_control::RuntimeOptions {
    Mode mode = Mode::Control;
    std::string program;
    int help_exit_code = 0;
    int port = 0;
    std::string models;
    std::string limits_config;
    std::string joystick;
};

Options ParseOptions(int argc, char** argv);
void PrintUsage(const std::string& program);
int Run(const Options& options);
}  // namespace g1_parkour_app
