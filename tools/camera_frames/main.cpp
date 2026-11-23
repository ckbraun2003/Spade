// spade_camera_frames: the engine camera's CPU reference frames of one pose of a world file -- the pinhole and the KB4 fisheye -- as PNGs (O150, video 4). The drawing is
// camera_frames.hpp's; this is the flags, the world file and the files.

#include <cstdio>
#include <string>
#include <vector>

#include "camera_frames.hpp"
#include "render/ray_scene.hpp"
#include "render/scene.hpp"
#include "world/world_file.hpp"

namespace {
constexpr const char* kUsage =
    "usage: spade_camera_frames --world FILE [--camera X,Y,Z] [--look-at X,Y,Z] [--up X,Y,Z] [--size WxH]\n"
    "                           [--pinhole-fov DEG] [--fisheye-fov DEG] [--k K1,K2,K3,K4] [--out DIR] [--name STEM] [--instances]\n"
    "  --world FILE        the world file to draw (primitives; a world whose props need mesh files is refused by the scene)\n"
    "  --camera, --look-at, --up   the pose, in the world's frame (+y up); defaults 0,1.5,-4  0,1,0  0,1,0\n"
    "  --size WxH          the picture, both lenses (default 640x480)\n"
    "  --pinhole-fov DEG   the pinhole's HORIZONTAL field of view (default 90)\n"
    "  --fisheye-fov DEG   the fisheye's DIAGONAL field of view, under 180 (default 170); --k its distortion k1..k4 (default 0: equidistant)\n"
    "  --out DIR --name STEM   writes DIR/STEM_pinhole.png and STEM_fisheye.png (default ./camera); --instances adds the instance ids in false colour\n"
    "the CPU reference camera (the engine's render_camera_frame): no GL, no GPU.\n";
}  // namespace

int main(int argc, char** argv) {
    std::vector<std::string> args;
    for (int i = 1; i < argc; ++i) args.emplace_back(argv[i]);
    if (args.empty() || args[0] == "--help" || args[0] == "-h") {
        std::fputs(kUsage, args.empty() ? stderr : stdout);
        return args.empty() ? 2 : 0;
    }
    const auto options = camera_frames::parse_args(args);
    if (!options) {
        std::fprintf(stderr, "spade_camera_frames: %s\n", options.error().context.c_str());
        return 2;
    }
    const auto world = spade::load_world_file(options->world);
    if (!world) {
        std::fprintf(stderr, "spade_camera_frames: %s\n", world.error().context.c_str());
        return 1;
    }
    const auto scene = spade::render::scene_from_world(*world, {});
    if (!scene) {
        std::fprintf(stderr, "spade_camera_frames: the scene: %s\n", scene.error().context.c_str());
        return 1;
    }
    const auto rays = spade::render::RayScene::build(*scene);
    if (!rays) {
        std::fprintf(stderr, "spade_camera_frames: the ray scene: %s\n", rays.error().context.c_str());
        return 1;
    }
    const auto views = camera_frames::render_views(*scene, *rays, *options, {});
    if (!views) {
        std::fprintf(stderr, "spade_camera_frames: %s\n", views.error().context.c_str());
        return 1;
    }
    const auto written = camera_frames::write_views(*options, *views);
    if (!written) {
        std::fprintf(stderr, "spade_camera_frames: %s\n", written.error().context.c_str());
        return 1;
    }
    for (const std::string& path : *written) std::printf("%s\n", path.c_str());
    return 0;
}
