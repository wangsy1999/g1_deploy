#pragma once

#include "g1_control_signals.h"
#include <stdexcept>
#include <string>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace g1_control {
// A client parameter lets tests exercise handover without contacting a robot.
template<class Client>
void ReleaseMotionService(Client& client, const volatile std::sig_atomic_t& stopped) {
    client.SetTimeout(5.0f);
    client.Init();
    for (int attempt = 0; attempt < 5 && !stopped; ++attempt) {
        std::string form, name;
        if (client.CheckMode(form, name) != 0)
            throw std::runtime_error("MotionSwitcher CheckMode failed");
        if (name.empty() && !stopped) return;
        if (stopped) break;
        if (client.ReleaseMode() != 0)
            throw std::runtime_error("MotionSwitcher ReleaseMode failed");
    }
    throw std::runtime_error("Motion service release not confirmed");
}

inline void ReleaseMotionService() {
    unitree::robot::b2::MotionSwitcherClient client;
    ReleaseMotionService(client, stop_requested);
}
}  // namespace g1_control
