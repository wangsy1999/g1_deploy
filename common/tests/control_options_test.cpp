#include "g1_dwaq_options.h"
#include "g1_parkour_options.h"
#include <initializer_list>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
void Require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class Parser> auto Parse(Parser parser, std::initializer_list<std::string> values) {
    std::vector<std::string> arguments{"controller"};
    arguments.insert(arguments.end(), values.begin(), values.end());
    std::vector<char*> argv;
    for (auto& value : arguments) argv.push_back(value.data());
    return parser(static_cast<int>(argv.size()), argv.data());
}
auto Dwaq(std::initializer_list<std::string> args) { return Parse(g1_dwaq_app::ParseOptions, args); }
auto Parkour(std::initializer_list<std::string> args) { return Parse(g1_parkour_app::ParseOptions, args); }
template<class Function> void Reject(Function function) {
    bool failed = false;
    try { function(); } catch (const std::exception&) { failed = true; }
    Require(failed, "Invalid arguments accepted");
}
void SameRuntime(const g1_control::RuntimeOptions& a, const g1_control::RuntimeOptions& b) {
    Require(a.hardware == b.hardware && a.domain == b.domain &&
            a.interface == b.interface && a.config == b.config, "Runtime options differ");
}
std::string HardwareInterface() {
    struct if_nameindex* interfaces = if_nameindex();
    std::string result;
    if (interfaces) {
        for (auto* entry = interfaces; entry->if_index; ++entry)
            if (std::string(entry->if_name) != "lo") { result = entry->if_name; break; }
        if_freenameindex(interfaces);
    }
    return result;
}
}  // namespace
int main() {
    try {
        const auto dwaq = Dwaq({"--sim", "--domain", "2", "--interface", "lo", "--config", "test.json"});
        const auto parkour = Parkour({"--sim", "--domain", "2", "--interface", "lo", "--config", "test.json"});
        SameRuntime(dwaq, parkour);
        Require(Parkour({}).port == 17200, "Default simulation port changed");
        Require(Dwaq({"--check-policy", "--config", "custom.json"}).config == "custom.json", "Named configuration lost");
        for (const auto* removed : {"--unitree-remote", "--model", "--model-dir", "--limits", "--reset-seconds"}) {
            Reject([&] { Dwaq({removed, "value"}); });
            Reject([&] { Parkour({removed, "value"}); });
        }
        Reject([] { Dwaq({"3", "lo", "1"}); });
        Reject([] { Dwaq({"--check-policy", "custom.json"}); });
        Reject([] { Parkour({"3", "lo", "1"}); });
        Reject([] { Dwaq({"--check-joystick", "--hardware-read-only"}); });
        Require(Dwaq({"--check-joystick"}).mode == g1_dwaq_app::Mode::CheckJoystick, "Joystick diagnostic broken");
        Reject([] { Dwaq({"--sim", "--hardware"}); });
        Reject([] { Parkour({"--sim", "--hardware"}); });
        Reject([] { Dwaq({"--domain", "1x"}); });
        Reject([] { Parkour({"--domain", "1x"}); });
        Reject([] { Dwaq({"--reset-seconds", "nan"}); });
        Reject([] { Parkour({"--reset-seconds", "nan"}); });
        Reject([] { Dwaq({"--interface"}); });
        Reject([] { Parkour({"--interface"}); });
        Reject([] { Dwaq({"3", "lo", "1", "--config", "mixed.json"}); });
        Reject([] { Dwaq({"--check-policy", "--check-joystick"}); });
        const auto interface = HardwareInterface();
        if (!interface.empty()) {
            const auto hardware = Dwaq({"--hardware", "--domain", "0", "--interface", interface, "--config", "test.json"});
            SameRuntime(hardware, Parkour({"--hardware", "--domain", "0", "--interface", interface, "--config", "test.json"}));
        } else {
            std::cout << "No non-loopback interface; hardware-interface cases skipped\n";
        }
        std::cout << "Named options, removed arguments and shared validation passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
