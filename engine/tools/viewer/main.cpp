// ---------------------------------------------------------------------------
// main.cpp -- spade_viewer entry point. v1-free (see bridge.cpp's file
// comment): only bridge.hpp and std are needed to parse a scene name and
// drive the Viewer.
//
// Usage: spade_viewer <drop|bounce|shower|gate>
// ---------------------------------------------------------------------------

#include <cstdio>
#include <exception>
#include <optional>
#include <string_view>
#include <utility>

#include "bridge.hpp"

namespace {

void print_usage(std::string_view program_name) {
    std::fprintf(stderr, "usage: %.*s <scene>\navailable scenes:\n", static_cast<int>(program_name.size()),
                 program_name.data());
    for (const std::string& name : spade::viewer::scene_names()) {
        std::fprintf(stderr, "  %s\n", name.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string_view program_name = argc > 0 ? std::string_view(argv[0]) : std::string_view("spade_viewer");

    if (argc != 2) {
        std::fprintf(stderr, "error: expected exactly one argument (the scene name)\n");
        print_usage(program_name);
        return 2;
    }

    const std::string_view scene_name(argv[1]);
    std::optional<spade::viewer::Scene> scene = spade::viewer::make_scene(scene_name);
    if (!scene) {
        std::fprintf(stderr, "error: unknown scene '%.*s'\n", static_cast<int>(scene_name.size()),
                     scene_name.data());
        print_usage(program_name);
        return 2;
    }

    try {
        spade::viewer::Viewer viewer(std::move(*scene));
        viewer.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "spade_viewer: fatal: %s\n", e.what());
        return 1;
    }

    return 0;
}
