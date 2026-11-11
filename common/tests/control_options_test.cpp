#include "g1_dwaq_app.h"
#include "g1_parkour_app.h"
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
    Require(a.hardware == b.hardware && a.reset_seconds == b.reset_seconds && a.domain == b.domain &&
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
        const auto dwaq = Dwaq({"--sim", "--domain", "2", "--interface", "lo", "--config", "test.json", "--reset-seconds", "4"});
        const auto parkour = Parkour({"--sim", "--domain", "2", "--interface", "lo", "--config", "test.json", "--reset-seconds", "4"});
        SameRuntime(dwaq, parkour);
        SameRuntime(dwaq, Dwaq({"4", "lo", "2", "test.json"}));
        Require(!dwaq.unitree_remote, "Simulation enabled hardware remote");
        Require(Parkour({}).port == 17200, "Default simulation port changed");
        Require(Dwaq({"--check-policy", "--config", "custom.json", "--model", "custom.onnx"}).model == "custom.onnx", "Named policy check lost model override");
        Require(Dwaq({"--check-policy", "custom.onnx", "custom.json"}).config == "custom.json", "Legacy model-first check broken");
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
            SameRuntime(hardware, Dwaq({"--hardware", "3", interface, "0", "test.json"}));
            Require(hardware.unitree_remote, "Hardware did not default to remote");
            Require(!Dwaq({"--hardware-read-only", "--domain", "0", "--interface", interface}).unitree_remote, "Read-only enabled control input");
        } else {
            std::cout << "No non-loopback interface; hardware-interface cases skipped\n";
        }
        std::cout << "Named options, legacy compatibility and shared validation passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
