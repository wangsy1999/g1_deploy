#include "g1_hybrid_options.h"
#include <iostream>

namespace g1_hybrid_app {
Options ParseOptions(int argc, char** argv) { return g1_parkour_app::ParseOptions(argc, argv); }
void PrintUsage(const std::string& program) {
    std::cout << program << " --sim [--domain 1 --interface lo --port 17200 --joystick DEVICE]\n"
              << program << " --hardware --domain 0 --interface ROBOT_IF [--port 17201]\n"
              << "Common: --config JSON --check-policy\n"
              << "DWaQ for stand/backward/turn; sustained forward command selects Parkour.\n"
              << "Switch thresholds and both policy configurations come from JSON.\n"
              << "Startup is dry-run; Start enables, X resets, A starts policy, Select stops (F710: B stops).\n";
}
}  // namespace g1_hybrid_app
