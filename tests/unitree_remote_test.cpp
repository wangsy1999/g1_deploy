#include "g1_unitree_remote.h"
#include <iostream>
#include <stdexcept>

void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int main() {
    g1_unitree_remote::Input input;
    std::array<uint8_t, 40> packet{};
    auto update = [&](unsigned keys) {
        packet[2] = keys & 255;
        packet[3] = (keys >> 8) & 255;
        input.Update(packet);
        return input.state.Action();
    };
    require(update(0) == 0, "neutral packet produced action");
    require(update(1u << 10) == 'p', "Unitree X must reset");
    require(update(1u << 10) == 0, "held X repeated action");
    require(update(0) == 0, "X release produced action");
    require(update(1u << 8) == 'r', "Unitree A must enter policy");
    require(update(1u << 8) == 0, "held A repeated action");
    require(update(0) == 0, "A release produced action");
    require(update(1u << 2) == '9', "Start must request handover");
    require(update(0) == 0, "Start release produced action");
    require(update(1u << 3) == ' ', "Select must stop and exit");
    g1_unitree_remote::Input initial;
    packet[2] = 0; packet[3] = 4;
    initial.Update(packet);
    require(initial.state.Action() == 0, "initial held X must not trigger reset");
    g1_unitree_remote::Input stopped;
    packet[2] = 8; packet[3] = 0;
    stopped.Update(packet);
    require(stopped.state.Action() == ' ', "initial held Select must stop");
    std::cout << "REMOTE_MAPPING_OK: X=reset, A=policy, Start=handover, Select=exit; edge and initial-packet checks passed\n";
}
