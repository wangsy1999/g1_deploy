#pragma once

#include "g1_parkour_options.h"

namespace g1_parkour_app {
// 实现在 src/controller.cpp：执行所选检查模式，或初始化并运行控制器。
int RunController(const Options& options);

class ControlPolicy;
// Shared DDS/input/reset/stop lifecycle for a policy implementation.
int RunControlLoop(const Options& options, ControlPolicy& policy, double reset_seconds);
}  // namespace g1_parkour_app
