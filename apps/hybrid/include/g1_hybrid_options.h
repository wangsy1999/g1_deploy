#pragma once
#include "g1_parkour_options.h"

namespace g1_hybrid_app {
using Options = g1_parkour_app::Options;
using Mode = g1_parkour_app::Mode;
// Reuse camera-controller runtime options; models/switching live in hybrid JSON.
Options ParseOptions(int argc, char** argv);
void PrintUsage(const std::string& program);
}  // namespace g1_hybrid_app
