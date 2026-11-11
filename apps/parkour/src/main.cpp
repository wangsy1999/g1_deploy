#include "g1_parkour_app.h"
#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    try {
        const auto options = g1_parkour_app::ParseOptions(argc, argv);
        if (options.mode == g1_parkour_app::Mode::Help) {
            g1_parkour_app::PrintUsage(options.program);
            return options.help_exit_code;
        }
        return g1_parkour_app::Run(options);
    } catch (const std::exception& error) {
        std::cerr << "g1_parkour: " << error.what() << '\n';
        return 1;
    }
}
