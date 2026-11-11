#include "g1_keyboard.h"
#include "g1_control_signals.h"
#include "g1_control_runtime.h"
#include "g1_motion_service.h"
#include "g1_control_targets.h"
#include <unitree/idl/hg/LowCmd_.hpp>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <sys/wait.h>
#include <vector>

namespace {
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class Function> void MustFail(Function function) {
    bool failed = false;
    try { function(); } catch (const std::runtime_error&) { failed = true; }
    Require(failed, "Expected failure was accepted");
}
struct FakeMotionService {
    std::vector<std::string> calls;
    int active_checks = 2, check_error = 0, release_error = 0;
    volatile std::sig_atomic_t* cancel_on_check = nullptr;
    void SetTimeout(float seconds) { Require(seconds == 5, "Wrong service timeout"); calls.push_back("timeout"); }
    void Init() { calls.push_back("init"); }
    int CheckMode(std::string&, std::string& name) {
        calls.push_back("check");
        name = active_checks-- > 0 ? "active" : "";
        if (cancel_on_check) *cancel_on_check = 1;
        return check_error;
    }
    int ReleaseMode() { calls.push_back("release"); return release_error; }
};
void CheckMotionService() {
    volatile std::sig_atomic_t stopped = 0;
    FakeMotionService service;
    g1_control::ReleaseMotionService(service, stopped);
    Require(service.calls == std::vector<std::string>{"timeout", "init", "check", "release", "check", "release", "check"},
            "Service release was not confirmed in order");
    FakeMotionService idle; idle.active_checks = 0;
    g1_control::ReleaseMotionService(idle, stopped);
    Require(idle.calls.size() == 3, "Idle service was released unnecessarily");
    FakeMotionService busy; busy.active_checks = 100;
    MustFail([&] { g1_control::ReleaseMotionService(busy, stopped); });
    Require(busy.calls.size() == 12, "Release retries were not bounded");
    FakeMotionService bad_check; bad_check.check_error = 1;
    MustFail([&] { g1_control::ReleaseMotionService(bad_check, stopped); });
    Require(bad_check.calls.size() == 3, "Release attempted after failed status check");
    FakeMotionService bad_release; bad_release.release_error = 1;
    MustFail([&] { g1_control::ReleaseMotionService(bad_release, stopped); });
    FakeMotionService cancelled; cancelled.active_checks = 0; cancelled.cancel_on_check = &stopped;
    MustFail([&] { g1_control::ReleaseMotionService(cancelled, stopped); });
    Require(cancelled.calls.size() == 3, "Cancelled confirmation accepted or release attempted");
    FakeMotionService already_stopped;
    MustFail([&] { g1_control::ReleaseMotionService(already_stopped, stopped); });
    Require(already_stopped.calls.size() == 2, "Release attempted after stop request");
}
void CheckSignals() {
    for (int signal : {SIGINT, SIGTERM, SIGHUP}) {
        const pid_t child = fork();
        Require(child >= 0, "fork failed");
        if (child == 0) {
            g1_control::InstallSignalHandlers();
            if (g1_control::StopRequested()) _exit(2);
            std::raise(signal);
            _exit(g1_control::StopRequested() ? 0 : 3);
        }
        int status = 0;
        Require(waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
                "Signal did not request a graceful stop");
    }
}
void CheckKeyboard() {
    int pipefd[2]; Require(pipe(pipefd) == 0, "pipe failed");
    {
        g1_control::Keyboard keyboard(pipefd[0]);
        Require(keyboard.Read() == 0, "Idle keyboard blocked or produced input");
        Require(write(pipefd[1], "pr ", 3) == 3, "Keyboard pipe write failed");
        Require(keyboard.Read() == 'p' && keyboard.Read() == 'r' && keyboard.Read() == ' ', "Keys were lost");
        close(pipefd[1]);
        Require(keyboard.Read(false) == 0 && keyboard.Read(true) == ' ', "Incorrect EOF policy");
    }
    close(pipefd[0]);
    const int master = posix_openpt(O_RDWR | O_NOCTTY);
    Require(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0, "Cannot create test terminal");
    const int slave = open(ptsname(master), O_RDWR | O_NOCTTY);
    Require(slave >= 0, "Cannot open test terminal");
    termios before{}, during{}, after{};
    Require(tcgetattr(slave, &before) == 0, "Cannot inspect terminal");
    try {
        g1_control::Keyboard keyboard(slave);
        Require(tcgetattr(slave, &during) == 0, "Cannot inspect active terminal");
        Require(!(during.c_lflag & (ICANON | ECHO)) &&
                (during.c_lflag & ISIG) == (before.c_lflag & ISIG), "Keyboard changed terminal signals or retained buffering");
        throw std::runtime_error("Exercise terminal restoration during exception unwinding");
    } catch (const std::runtime_error& error) {
        // Preserve genuine check failures instead of swallowing them.
        if (std::string(error.what()) != "Exercise terminal restoration during exception unwinding") throw;
    }
    Require(tcgetattr(slave, &after) == 0 && before.c_iflag == after.c_iflag &&
            before.c_oflag == after.c_oflag && before.c_cflag == after.c_cflag &&
            before.c_lflag == after.c_lflag && std::memcmp(before.c_cc, after.c_cc, NCCS) == 0,
            "Terminal settings were not restored");
    close(slave); close(master);
}
void CheckInputPriority() {
    Require(g1_control::kStartupTimeout == std::chrono::seconds(10), "Startup wait differs");
    const std::array<char, 7> actions{0, ' ', '9', 'p', 'r', 'x', 'w'};
    for (char keyboard : actions) {
        for (char remote : actions) {
            const char selected = g1_control::SelectAction(keyboard, remote);
            if (keyboard == ' ' || remote == ' ') Require(selected == ' ', "Stop was overridden");
            else if (remote) Require(selected == remote, "Keyboard overrode remote");
            else Require(selected == keyboard, "Keyboard fallback lost");
        }
    }
}
void CheckPositionTargets() {
    std::array<float, 29> start{}, pose{}, kp{}, kd{};
    for (int joint = 0; joint < 29; ++joint) {
        start[joint] = -.1f * joint; pose[joint] = .1f * joint;
        kp[joint] = 40 + joint; kd[joint] = 1 + .1f * joint;
    }
    Require(g1_control::InterpolateResetPose(start, pose, 0) == start &&
            g1_control::InterpolateResetPose(start, pose, 1) == pose, "Pose interpolation endpoints changed");
    Require(g1_control::NearPose(start, start) && !g1_control::NearPose(start, pose), "Pose entry check failed");
    unitree_hg::msg::dds_::LowCmd_ command{};
    command.mode_machine() = 5; command.crc() = 123;
    command.motor_cmd()[29].mode() = 7;
    g1_control::SetPositionTargets(command, pose, kp, kd);
    for (int joint = 0; joint < 29; ++joint) {
        const auto& motor = command.motor_cmd()[joint];
        Require(motor.mode() == 1 && motor.q() == pose[joint] && motor.kp() == kp[joint] &&
                motor.kd() == kd[joint] && motor.dq() == 0 && motor.tau() == 0, "Position or PD order changed");
    }
    Require(command.mode_machine() == 5 && command.crc() == 123 && command.motor_cmd()[29].mode() == 7,
            "Position fill changed metadata or other motors");
    struct Input {
        bool ready, centered, hold, moving;
        bool Ready() const { return ready; }
        bool Centered() const { return centered; }
        bool HoldRequired() const { return hold; }
        bool MovingEnabled() const { return moving; }
    };
    Require(g1_control::InputReady(static_cast<const Input*>(nullptr)), "Keyboard-only entry rejected");
    for (int bits = 0; bits < 16; ++bits) {
        Input input{bool(bits & 1), bool(bits & 2), bool(bits & 4), bool(bits & 8)};
        Require(g1_control::InputReady(&input) == (input.ready && input.centered && (!input.hold || !input.moving)),
                "Input entry interlock changed");
    }
}
void CheckTargets() {
    Require(g1_control::ResetProgress(-1, 3) == 0 && g1_control::ResetProgress(4, 3) == 1, "Reset progress not clamped");
    Require(g1_control::InterpolateReset(1000000.f, .1f, 1) == .1f, "Reset did not finish at exact target");
    Require(g1_control::InterpolateReset(-2.f, 2.f, 0) == -2.f &&
            g1_control::InterpolateReset(-2.f, 2.f, .5) == 0 &&
            std::abs(g1_control::InterpolateReset(-2.f, 2.f, .25) + 1.375f) < 1e-6f,
            "Reset trajectory changed");
    for (bool hardware : {false, true}) {
        unitree_hg::msg::dds_::LowCmd_ command;
        command.mode_machine() = 5; command.mode_pr() = 1; command.crc() = 123;
        for (auto& motor : command.motor_cmd()) {
            motor.mode() = 7; motor.q() = 2; motor.dq() = 3;
            motor.kp() = 4; motor.kd() = 5; motor.tau() = 6;
        }
        const auto untouched = command.motor_cmd()[29];
        g1_control::SetDampingTargets(command, hardware, 1.25f);
        for (int i = 0; i < 29; ++i) {
            const auto& m = command.motor_cmd()[i];
            Require(m.mode() == (hardware ? 1 : 0) && m.q() == 0 && m.dq() == 0 &&
                    m.kp() == 0 && m.tau() == 0 && m.kd() == (hardware ? 1.25f : 0), "Invalid damping command");
        }
        Require(command.mode_machine() == 5 && command.mode_pr() == 1 && command.crc() == 123 &&
                command.motor_cmd()[29] == untouched, "Damping changed metadata or uncontrolled motors");
    }
}
}  // namespace
int main() {
    try {
        CheckMotionService(); CheckSignals(); CheckKeyboard(); CheckInputPriority(); CheckPositionTargets(); CheckTargets();
        std::cout << "Shared keyboard, signals, service release, reset and damping checks passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
