// tools/flow_movie/line_export.hpp -- streamlines of a cached hover frame as JSON, for the website's 3-D viewer
// (Cameron's ask, relayed by the lead 2026-11-10): about 300 to 800 lines seeded around each rotor disc, each
// point in metres in the aircraft's BODY frame with the air's speed there, under a header that names the rotors,
// the axes, the units and the run. Pure: a published FlowVolume in, text out; no GL, no ffmpeg, no solver, so a
// test links nothing of the tool. The only floats are the engine's own (sin32/cos32 seeds, RK4 in arc length).
//
// The speed at a point is |u| of the air relative to the held aircraft, m/s, as the volume stores it. A body
// frame is a rigid turn and shift of the volume's world, so lengths and speeds are the same in both; only the
// points move.
#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"

namespace flow_movie {

// One rotor disc, in the BODY frame (vehicles::RotorDesc: local_pos, local_orient * +Y, radius, spin_dir).
struct ExportRotor {
    glm::vec3 hub{0.0f};   // m, from the aircraft's centre of mass
    glm::vec3 axis{0.0f, 1.0f, 0.0f};  // unit: the thrust direction (the air leaves the disc along -axis)
    float radius = 0.0f;   // m
    float spin_dir = 0.0f;
};

// The body frame's pose in the volume's world: a point p of the body is pos + orient * p.
struct ExportBody {
    glm::vec3 pos{0.0f};
    glm::quat orient{1.0f, 0.0f, 0.0f, 0.0f};
};

// What the header says about the run. All of it is the caller's to give: the exporter never reads a clock or a
// repository.
struct ExportInfo {
    std::string name;    // "flow_hover"
    std::string sim;     // the build that simulated the field (the cache's mark)
    std::string config;  // the case: its identity (model, program, grid, wind)
    std::string note;    // one line: what the field is and is not
    uint32_t frame = 0;  // the cached frame the field is
    uint64_t step = 0;
    double t_s = 0.0;    // simulated time of the frame
};

// Each file is under 10 MB (the brief). Decimal, so 10 MB the way a website counts it.
inline constexpr uint64_t kExportMaxBytes = 10'000'000;

struct ExportOptions {
    uint32_t lines = 600;           // seeds in all, 1 to 20000, shared among the rotors
    float seed_height_radii = 0.3f;  // the seed plane's height above the disc along its axis, in radii
    float spacing_m = 0.008f;       // the first try at the least arc length between kept points
    int point_decimals = 4;         // metres: 0.1 mm
    int speed_decimals = 2;         // m/s
    uint64_t max_bytes = kExportMaxBytes;
};

struct ExportResult {
    std::string json;
    uint32_t lines = 0;  // written
    uint32_t dropped = 0;  // traced but under two points once decimated
    uint64_t points = 0;
    float spacing_m = 0.0f;  // the least arc length between kept points that fit under max_bytes
};

// The lines' seeds, in the volume's world: per rotor, rings about its axis at 0.3, 0.55, 0.8, 1.05 and 1.3 radii
// in the plane `seed_height_radii` above the disc, the rings' counts in proportion to their radius, the rotors
// sharing `lines` evenly (the first rotors take the remainder). A pure function of its inputs.
[[nodiscard]] spade::Result<std::vector<glm::vec3>> export_seeds(const ExportBody& body,
                                                                 std::span<const ExportRotor> rotors,
                                                                 const ExportOptions& options);

// Traces both ways from every seed (one box diagonal at most each way, a quarter cell a step), decimates by arc
// length to the least spacing that keeps the text under max_bytes (the first try, then 1.25 times more, at most
// twelve times), and writes {"header": {...}, "lines": [{"points": [[x, y, z], ...], "speeds": [...]}, ...]} with
// fixed decimals. invalid_argument, by name: a volume that fails validation, no rotors, a rotor that is not
// finite or has no radius or an axis that is not unit, a body that is not finite, lines or decimals out of range.
// capacity_exceeded: even the last spacing does not fit (the size reached is named).
[[nodiscard]] spade::Result<ExportResult> export_streamlines(const spade::FlowVolume& volume, const ExportBody& body,
                                                             std::span<const ExportRotor> rotors,
                                                             const ExportInfo& info,
                                                             const ExportOptions& options = {});

// The .md beside a data file: what it is, where it came from, how it was made, and what it does not claim.
struct ExportCaption {
    const ExportInfo* info = nullptr;
    const ExportResult* result = nullptr;
    std::string commit;
    std::string date_utc;
    std::string command;
    std::string ticket;
    bool crosswind = false;  // a wind from the side (supported but not validated), else the case's own headwind
    bool device = false;     // the fluid stepped by the GPU solver
};
[[nodiscard]] std::string export_caption_text(const ExportCaption& in);

// A MANIFEST line for a data file, in the shape of the videos': name | what | - | label | description.
[[nodiscard]] std::string data_manifest_line(const std::string& file, uint64_t bytes, const std::string& label,
                                             const std::string& description);

}  // namespace flow_movie
