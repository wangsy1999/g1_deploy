#include "g1_locomotion_policy.h"
#include <iostream>
#include <limits>

void Near(float actual, float expected) {
    if (!std::isfinite(actual) || std::abs(actual-expected) > 2e-5f)
        throw std::runtime_error("Mismatch: " + std::to_string(actual) + " vs " + std::to_string(expected));
}

int main(int argc, char** argv) {
    try {
        if (argc != 3) throw std::runtime_error("Expected model and deploy config paths");
        g1_policy::Policy policy(argv[1], argv[2]);
        auto positive=policy.GamepadVelocity({1,1,1});
        Near(positive[0],1);Near(positive[1],.5f);Near(positive[2],1.57f);
        auto negative=policy.GamepadVelocity({-1,-1,-1});
        Near(negative[0],-.6f);Near(negative[1],-.5f);Near(negative[2],-1.57f);
        auto half=policy.GamepadVelocity({-.5f,.5f,.5f});
        Near(half[0],-.3f);Near(half[1],.25f);Near(half[2],.785f);
        const std::array<int, 29> ids{0,6,12,1,7,13,2,8,14,3,9,15,22,4,10,16,23,5,11,17,24,18,25,19,26,20,27,21,28};
        g1_policy::State state;
        state.q = policy.DefaultDds();
        for (int i = 0; i < 29; ++i) { state.q[i] += 0.01f*(i+1); state.dq[i] = 0.2f*(i+1); }
        state.omega = {1, 2, 3};
        state.quaternion = {std::sqrt(0.5f), 0, std::sqrt(0.5f), 0};
        auto first = policy.Observe(state, {0.1f, 0.2f, -0.1f});
        std::array<float, 96> frame{};
        frame[0] = 1; frame[1] = 2; frame[2] = 3;
        frame[3] = 1; frame[4] = 0; frame[5] = 0;
        frame[6] = 0.1f; frame[7] = 0.2f; frame[8] = -0.1f;
        for (int i = 0; i < 29; ++i) { frame[9+i] = 0.01f*(ids[i]+1); frame[38+i] = 0.01f*(ids[i]+1); }
        int cursor = 0, offset = 0;
        for (int size : {3,3,3,29,29,29}) {
            for (int h = 0; h < 5; ++h)
                for (int i = 0; i < size; ++i) Near(first[cursor++], frame[offset+i]);
            offset += size;
        }
        // Only newest command/position samples change; older history stays intact.
        state.q[0] += 0.5f;
        const auto second = policy.Observe(state, {0.3f, 0.2f, -0.1f});
        Near(second[30], 0.1f); Near(second[42], 0.3f);
        Near(second[45], 0.01f); Near(second[45+4*29], 0.51f);
        // A normalized quaternion and its negative must project gravity identically.
        for (float& v : state.quaternion) v *= -2;
        const auto third = policy.Observe(state, {0.3f, 0.2f, -0.1f});
        Near(third[27], 1); Near(third[28], 0); Near(third[29], 0);
        std::array<float, 3> command{100, -100, 100}; policy.ClampVelocity(command);
        Near(command[0], 1); Near(command[1], -0.5f); Near(command[2], 1.57f);
        // Real model output must round-trip through DDS mapping into last_action.
        policy.Reset(); state = {}; state.q = policy.DefaultDds();
        const auto target = policy.Step(state, {0,0,0});
        const auto observation = policy.Observe(state, {0,0,0});
        for (int i = 0; i < 29; ++i)
            Near(observation[335+4*29+i], (target[ids[i]]-state.q[ids[i]])/0.25f);
        policy.Reset();
        const auto repeated = policy.Step(state, {0,0,0});
        for (int i = 0; i < 29; ++i) Near(target[i], repeated[i]);
        bool rejected = false;
        state.quaternion.fill(0);
        try { policy.Step(state, {0,0,0}); } catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Invalid quaternion accepted");
        state.quaternion = {1,0,0,0}; state.dq[0] = std::numeric_limits<float>::quiet_NaN(); rejected = false;
        try { policy.Step(state, {0,0,0}); } catch (const std::runtime_error&) { rejected = true; }
        if (!rejected) throw std::runtime_error("Non-finite velocity accepted");
        // Training clips scaled observations and actions, with no joint target clip.
        policy.Reset(); state = {}; state.q = policy.DefaultDds();
        state.omega = {1000, -1000, 0}; state.dq[0] = 4000;
        const auto clipped = policy.Observe(state, {0,0,0});
        Near(clipped[0], 100); Near(clipped[1], -100); Near(clipped[190], 100);
        g1_policy::Joints raw{};
        raw.fill(200);
        const auto upper = policy.ProcessActions(raw);
        for (int i = 0; i < 29; ++i) Near(upper[i], policy.DefaultDds()[i] + 25);
        raw.fill(-200);
        const auto lower = policy.ProcessActions(raw);
        for (int i = 0; i < 29; ++i) Near(lower[i], policy.DefaultDds()[i] - 25);
        const auto feedback = policy.Observe(state, {0,0,0});
        for (int i = 0; i < 29; ++i) Near(feedback[335+4*29+i], -100);
        raw.fill(3);
        const auto small = policy.ProcessActions(raw);
        const auto neutral = policy.DefaultDds();
        for (int i = 0; i < 29; ++i) Near(small[i], neutral[i]+0.75f);
        Near(policy.kp[12], 100); Near(policy.kp[13], 75); Near(policy.kp[14], 75);
        for (int i = 15; i < 29; ++i) Near(policy.kd[i], 3);
        std::cout << "Policy observation, history, action mapping, reset and invalid-state checks passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
