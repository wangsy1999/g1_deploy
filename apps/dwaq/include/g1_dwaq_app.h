#pragma once

#include <string>
#include "g1_control_options.h"

namespace g1_dwaq_app {
enum class Mode { Control, ReadOnly, CheckPolicy, CheckJoystick, Help };

struct Options : g1_control::RuntimeOptions {
    Mode mode = Mode::Control;
    std::string program;
    int help_exit_code = 0;
    bool unitree_remote = false; // Enabled automatically for hardware control by ParseOptions.
    std::string model;
    std::string joystick;
};

Options ParseOptions(int argc, char** argv);
void PrintUsage(const std::string& program);
int Run(const Options& options);
}  // namespace g1_dwaq_app
