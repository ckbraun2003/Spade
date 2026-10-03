// ---------------------------------------------------------------------------
// main.cpp -- spade_viewer entry point. v1-free (see bridge.cpp's file
// comment): only bridge.hpp and std are needed to parse a scene name and
// drive the Viewer.
//
// Usage: spade_viewer <scene> [cpu|vulkan] [--trajectory <file> [--ticks N]]
//
// --trajectory steps the scene headless, with no window and no v1, and writes
// its INT-4 trajectory baseline (trajectory.hpp). It needs the cpu backend.
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

#include <charconv>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include "bridge.hpp"
#include "trajectory.hpp"

namespace {

void print_usage(std::string_view program_name) {
    std::fprintf(stderr, "usage: %.*s <scene> [cpu|vulkan] [--trajectory <file> [--ticks N]]\navailable scenes:\n",
                 static_cast<int>(program_name.size()), program_name.data());
    for (const std::string& name : spade::viewer::scene_names()) {
        std::fprintf(stderr, "  %s\n", name.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    const std::string_view program_name = argc > 0 ? std::string_view(argv[0]) : std::string_view("spade_viewer");

    if (argc < 2) {
        std::fprintf(stderr, "error: expected a scene name and an optional backend\n");
        print_usage(program_name);
        return 2;
    }

    const std::string_view scene_name(argv[1]);

    // A second argument that is not a flag is the backend, as it always was.
    const bool has_backend = argc >= 3 && !std::string_view(argv[2]).starts_with("--");

    // Flags after the positionals: only the headless trajectory and its length.
    std::optional<std::string> trajectory_path;
    uint64_t ticks = spade::viewer::kTrajectoryDefaultTicks;
    bool ticks_given = false;
    for (int i = has_backend ? 3 : 2; i < argc; ++i) {
        const std::string_view flag(argv[i]);
        if (flag == "--trajectory" && i + 1 < argc) {
            trajectory_path = argv[++i];
        } else if (flag == "--ticks" && i + 1 < argc) {
            const std::string_view value(argv[++i]);
            const std::from_chars_result parsed = std::from_chars(value.data(), value.data() + value.size(), ticks);
            if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() || ticks == 0) {
                std::fprintf(stderr, "error: --ticks needs a positive whole number, not '%.*s'\n",
                             static_cast<int>(value.size()), value.data());
                return 2;
            }
            ticks_given = true;
        } else {
            std::fprintf(stderr, "error: unexpected argument '%.*s'\n", static_cast<int>(flag.size()), flag.data());
            print_usage(program_name);
            return 2;
        }
    }
    if (ticks_given && !trajectory_path) {
        std::fprintf(stderr, "error: --ticks applies only to --trajectory; add --trajectory <file>\n");
        return 2;
    }

    // The backend argument is parsed BEFORE the scene is built, so an
    // unrecognized one fails immediately rather than after a window has opened.
    spade::compute::BackendDesc backend{};
    if (has_backend) {
        const std::string_view backend_name(argv[2]);
        if (backend_name == "cpu") {
            backend.kind = spade::compute::BackendKind::cpu;
        } else if (backend_name == "vulkan") {
            backend.kind = spade::compute::BackendKind::vulkan;
            // A trajectory baseline is bit-identical only on the CPU (L4).
            if (trajectory_path) {
                std::fprintf(stderr,
                             "error: --trajectory needs the cpu backend: vulkan agrees with cpu within "
                             "bands, not bit for bit. Run it with cpu.\n");
                return 2;
            }
            // ⚠ SAID AT THE MOMENT A HUMAN CHOOSES IT, because the symptom
            // (a scene that crawls, or shows no frame at all) is otherwise
            // indistinguishable from a broken build -- which is exactly how
            // it was first reported. See compute/backend.hpp's BackendDesc
            // for the measurements. It is NOT refused: the backend stays
            // selectable so it can be measured and improved.
            std::fprintf(stderr,
                         "spade_viewer: NOTE -- the vulkan compute backend is SLOWER than cpu in "
                         "every configuration measured (74-2139x; see compute/backend.hpp). A "
                         "single-world scene runs its whole dynamic-collision sweep on one GPU "
                         "lane, and 1000 bodies may take minutes per frame. This is expected, not "
                         "a broken build.\n");
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

    if (trajectory_path) {
        return spade::viewer::write_trajectory(*scene, ticks, *trajectory_path);
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
