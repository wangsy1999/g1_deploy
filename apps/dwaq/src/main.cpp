#include "g1_dwaq_options.h"
#include "g1_dwaq_controller.h"
#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    try {
        const auto options = g1_dwaq_app::ParseOptions(argc, argv);
        if (options.mode == g1_dwaq_app::Mode::Help) {
            g1_dwaq_app::PrintUsage(options.program);
            return options.help_exit_code;
        }
        return g1_dwaq_app::RunController(options);
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
