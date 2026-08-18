// ---------------------------------------------------------------------------
// main.cpp -- spade_viewer entry point. v1-free (see bridge.cpp's file
// comment): only bridge.hpp and std are needed to parse a scene name and
// drive the Viewer.
//
// Usage: spade_viewer <scene> [cpu|vulkan]
//
// THE OPTIONAL SECOND ARGUMENT (S6 Task 6) selects the compute backend. It
// defaults to `cpu`, so every existing invocation -- including every one
// scripts/demo.ps1 made before this task -- behaves exactly as it did. `vulkan`
// routes the SAME scene, the same spawns and the same fixed-step decomposition
// through the ported kernels; nothing else about the run changes, which is what
// makes the comparison a demonstration of the port rather than of a second code
// path.
//
// ALL EIGHT schedule passes are ported as of S6 Task 8 -- MediumUpdate,
// ForceElements (rotors and drag), CollisionStatic, CollisionDynamic,
// Integrate and SensorSynthesis have kernels; Gravity and Publish are inert
// by design on both backends -- so no scene is refused for an unported pass
// any more (through Task 7 a scene needing turbulence, rotors or IMU sensors
// was rejected at startup with a message naming the pass; that gate is gone,
// see simulation.cpp). CPU<->vulkan agreement is banded, not bit-identical --
// see engine/testing/parity.hpp for the per-scenario tables.
// ---------------------------------------------------------------------------

#include <cstdio>
#include <exception>
#include <optional>
#include <string_view>
#include <utility>

#include "bridge.hpp"

namespace {

void print_usage(std::string_view program_name) {
    std::fprintf(stderr, "usage: %.*s <scene> [cpu|vulkan]\navailable scenes:\n",
                 static_cast<int>(program_name.size()), program_name.data());
    for (const std::string& name : spade::viewer::scene_names()) {
        std::fprintf(stderr, "  %s\n", name.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string_view program_name = argc > 0 ? std::string_view(argv[0]) : std::string_view("spade_viewer");

    if (argc < 2 || argc > 3) {
        std::fprintf(stderr, "error: expected a scene name and an optional backend\n");
        print_usage(program_name);
        return 2;
    }

    const std::string_view scene_name(argv[1]);

    // The backend argument is parsed BEFORE the scene is built, so an
    // unrecognized one fails immediately rather than after a window has opened.
    spade::compute::BackendDesc backend{};
    if (argc == 3) {
        const std::string_view backend_name(argv[2]);
        if (backend_name == "cpu") {
            backend.kind = spade::compute::BackendKind::cpu;
        } else if (backend_name == "vulkan") {
            backend.kind = spade::compute::BackendKind::vulkan;
        } else {
            std::fprintf(stderr, "error: unknown backend '%.*s' (expected cpu or vulkan)\n",
                         static_cast<int>(backend_name.size()), backend_name.data());
            print_usage(program_name);
            return 2;
        }
    }

    std::optional<spade::viewer::Scene> scene = spade::viewer::make_scene(scene_name);
    if (!scene) {
        std::fprintf(stderr, "error: unknown scene '%.*s'\n", static_cast<int>(scene_name.size()),
                     scene_name.data());
        print_usage(program_name);
        return 2;
    }

    try {
        spade::viewer::Viewer viewer(std::move(*scene), backend);
        viewer.run();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "spade_viewer: fatal: %s\n", e.what());
        return 1;
    }

    return 0;
}
