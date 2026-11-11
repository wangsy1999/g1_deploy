#include "g1_hybrid_options.h"
#include "g1_hybrid_controller.h"
#include <exception>
#include <iostream>

int main(int argc, char** argv) {
    try {
        const auto options = g1_hybrid_app::ParseOptions(argc, argv);
        if (options.mode == g1_hybrid_app::Mode::Help) {
            g1_hybrid_app::PrintUsage(options.program);
            return options.help_exit_code;
        }
        return g1_hybrid_app::RunController(options);
    } catch (const std::exception& error) {
        std::cerr << "g1_hybrid: " << error.what() << '\n';
        return 1;
    }
}
