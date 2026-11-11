#include "g1_hybrid_controller.h"
#include "g1_hybrid_policy.h"
#include "g1_parkour_controller.h"
#include "g1_control_signals.h"
#include <iostream>

namespace g1_hybrid_app {
int RunController(const Options& options) {
    g1_control::InstallSignalHandlers();
    g1_hybrid::Policy policy(options.config);
    std::cout << "Hybrid models warmed up; synthetic histories cleared.\n";
    if (options.mode == Mode::CheckPolicy) {
        const auto defaults = policy.DefaultTargets();
        g1_hybrid::Policy::Observation observation;
        observation.lowstate.imu_state().quaternion() = {1, 0, 0, 0};
        observation.gravity = {0, 0, -1};
        observation.depth.fill(2.5f);
        const auto& camera = policy.CameraPolicy();
        for (int joint = 0; joint < 29; ++joint) {
            observation.lowstate.motor_state()[joint].q() = defaults.position[joint];
            observation.q[joint] = camera.sign[joint] * defaults.position[camera.ids[joint]];
        }
        for (const auto velocity : {std::array<float, 3>{0, 0, 0}, {.3f, 0, 0}, {-.2f, 0, 0}}) {
            for (int tick = 0; tick < 40; ++tick) policy.Step(observation, velocity);
            std::cout << "Hybrid inference OK: vx=" << velocity[0] << " uses "
                      << (policy.SelectedMode() == g1_hybrid::Mode::Dwaq ? "DWaQ" : "Parkour") << '\n';
        }
        return 0;
    }
    const auto duration = g1_config::ReadResetSeconds(YAML::LoadFile(options.config));
    return g1_parkour_app::RunControlLoop(options, policy, duration);
}
}  // namespace g1_hybrid_app
