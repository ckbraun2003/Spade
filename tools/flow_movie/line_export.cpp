#include "line_export.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <vector>

#include "render/flow_lines.hpp"

namespace flow_movie {
namespace {

constexpr float kRingRadii[] = {0.30f, 0.55f, 0.80f, 1.05f, 1.30f};  // of the rotor's radius
constexpr uint32_t kMaxLines = 20000;
constexpr int kMaxTries = 12;
constexpr float kSpacingGrowth = 1.25f;

[[nodiscard]] bool finite3(glm::vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

// "%.*f" without a negative zero: -0.00004 at four decimals is "0.0000", not "-0.0000".
void put_number(std::string& out, double x, int decimals) {
    char buffer[48];
    int n = std::snprintf(buffer, sizeof buffer, "%.*f", decimals, x);
    if (n > 0 && buffer[0] == '-') {
        bool zero = true;
        for (int i = 1; i < n; ++i) zero = zero && (buffer[i] == '0' || buffer[i] == '.');
        if (zero) n = std::snprintf(buffer, sizeof buffer, "%.*f", decimals, 0.0);
    }
    out.append(buffer, static_cast<std::size_t>(n > 0 ? n : 0));
}

void put_vec(std::string& out, glm::vec3 v, int decimals) {
    out += '[';
    put_number(out, v.x, decimals);
    out += ',';
    put_number(out, v.y, decimals);
    out += ',';
    put_number(out, v.z, decimals);
    out += ']';
}

// A JSON string: quotes, backslashes and control characters escaped; the rest verbatim.
void put_string(std::string& out, const std::string& text) {
    out += '"';
    for (const char c : text) {
        const auto u = static_cast<unsigned char>(c);
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (u < 0x20u) {
            char escaped[8];
            std::snprintf(escaped, sizeof escaped, "\\u%04x", static_cast<unsigned>(u));
            out += escaped;
        } else {
            out += c;
        }
    }
    out += '"';
}

struct Kept {
    std::vector<glm::vec3> points;  // body frame
    std::vector<float> speeds;
};

// The line's points a least arc length apart (the first and the last always kept), in the body frame.
[[nodiscard]] Kept decimate(const spade::render::Streamline& line, const ExportBody& body, float spacing) {
    Kept kept;
    const glm::quat inverse = glm::conjugate(body.orient);
    glm::vec3 last_world{0.0f};
    for (std::size_t i = 0; i < line.points.size(); ++i) {
        const bool edge = i == 0 || i + 1 == line.points.size();
        if (!edge && glm::length(line.points[i] - last_world) < spacing) continue;
        kept.points.push_back(inverse * (line.points[i] - body.pos));
        kept.speeds.push_back(line.speeds[i]);
        last_world = line.points[i];
    }
    return kept;
}

[[nodiscard]] spade::Result<void> validate(const spade::FlowVolume& volume, const ExportBody& body,
                                           std::span<const ExportRotor> rotors, const ExportOptions& options) {
    using spade::Code;
    using spade::Error;
    if (auto ok = spade::validate_flow_volume(volume); !ok) return ok;
    if (rotors.empty()) return std::unexpected(Error{Code::invalid_argument, "the export needs at least one rotor"});
    if (!finite3(body.pos) || !std::isfinite(body.orient.w) || !std::isfinite(body.orient.x) ||
        !std::isfinite(body.orient.y) || !std::isfinite(body.orient.z)) {
        return std::unexpected(Error{Code::invalid_argument, "the body's pose is not finite"});
    }
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        const ExportRotor& r = rotors[i];
        const std::string who = "rotor " + std::to_string(i);
        if (!finite3(r.hub) || !finite3(r.axis) || !std::isfinite(r.radius) || !std::isfinite(r.spin_dir)) {
            return std::unexpected(Error{Code::invalid_argument, who + " is not finite"});
        }
        if (!(r.radius > 0.0f)) return std::unexpected(Error{Code::invalid_argument, who + " has no radius"});
        if (std::fabs(glm::length(r.axis) - 1.0f) > 1.0e-3f) {
            return std::unexpected(Error{Code::invalid_argument, who + "'s axis is not a unit vector"});
        }
    }
    if (options.lines < 1u || options.lines > kMaxLines) {
        return std::unexpected(Error{Code::invalid_argument, "lines must be 1 to " + std::to_string(kMaxLines)});
    }
    if (!(options.spacing_m > 0.0f) || !std::isfinite(options.spacing_m)) {
        return std::unexpected(Error{Code::invalid_argument, "the spacing must be finite and above 0"});
    }
    if (options.point_decimals < 0 || options.point_decimals > 8 || options.speed_decimals < 0 ||
        options.speed_decimals > 8) {
        return std::unexpected(Error{Code::invalid_argument, "decimals must be 0 to 8"});
    }
    if (!(options.seed_height_radii >= 0.0f) || !std::isfinite(options.seed_height_radii)) {
        return std::unexpected(Error{Code::invalid_argument, "the seed height must be finite and not negative"});
    }
    return {};
}

}  // namespace

spade::Result<std::vector<glm::vec3>> export_seeds(const ExportBody& body, std::span<const ExportRotor> rotors,
                                                   const ExportOptions& options) {
    if (rotors.empty()) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "the export needs at least one rotor"});
    }
    constexpr std::size_t kRings = std::size(kRingRadii);
    float weight_sum = 0.0f;
    for (const float f : kRingRadii) weight_sum += f;
    std::vector<glm::vec3> seeds;
    for (std::size_t i = 0; i < rotors.size(); ++i) {
        const ExportRotor& r = rotors[i];
        const uint32_t share = options.lines / static_cast<uint32_t>(rotors.size()) +
                               (i < options.lines % rotors.size() ? 1u : 0u);
        const glm::vec3 axis = glm::normalize(body.orient * r.axis);
        const glm::vec3 centre = body.pos + body.orient * r.hub + axis * (options.seed_height_radii * r.radius);
        for (std::size_t ring = 0; ring < kRings; ++ring) {
            const auto n = static_cast<uint32_t>(
                std::lround(static_cast<double>(share) * static_cast<double>(kRingRadii[ring] / weight_sum)));
            if (n == 0u) continue;
            for (const glm::vec3& s : spade::render::seed_ring(centre, axis, kRingRadii[ring] * r.radius, n)) {
                seeds.push_back(s);
            }
        }
    }
    return seeds;
}

spade::Result<ExportResult> export_streamlines(const spade::FlowVolume& volume, const ExportBody& body,
                                               std::span<const ExportRotor> rotors, const ExportInfo& info,
                                               const ExportOptions& options) {
    if (auto ok = validate(volume, body, rotors, options); !ok) return std::unexpected(ok.error());
    auto seeds = export_seeds(body, rotors, options);
    if (!seeds) return std::unexpected(seeds.error());

    spade::render::StreamlineOptions trace;
    trace.max_length_diagonals = 1.0f;  // one box diagonal each way: a line on a closed orbit costs one crossing
    auto traced = spade::render::trace_streamlines(volume, *seeds, trace);
    if (!traced) return std::unexpected(traced.error());

    const int pd = options.point_decimals, sd = options.speed_decimals;
    float spacing = options.spacing_m;
    uint64_t reached = 0;
    for (int attempt = 0; attempt < kMaxTries; ++attempt, spacing *= kSpacingGrowth) {
        ExportResult result;
        result.spacing_m = spacing;
        std::string lines;
        for (const spade::render::Streamline& line : *traced) {
            const Kept kept = decimate(line, body, spacing);
            if (kept.points.size() < 2u) {
                ++result.dropped;
                continue;
            }
            if (result.lines > 0u) lines += ',';
            lines += "{\"points\":[";
            for (std::size_t i = 0; i < kept.points.size(); ++i) {
                if (i > 0u) lines += ',';
                put_vec(lines, kept.points[i], pd);
            }
            lines += "],\"speeds\":[";
            for (std::size_t i = 0; i < kept.speeds.size(); ++i) {
                if (i > 0u) lines += ',';
                put_number(lines, kept.speeds[i], sd);
            }
            lines += "]}";
            ++result.lines;
            result.points += kept.points.size();
        }

        std::string head = "{\"format\":\"spade-streamlines-1\",\"name\":";
        put_string(head, info.name);
        head += ",\"frame\":\"body\",\"units\":{\"length\":\"m\",\"speed\":\"m/s\"}";
        head += ",\"axes\":";
        put_string(head, "the airframe model's body frame, right-handed, origin at the centre of mass: +x the nose, "
                         "+y up (the thrust direction of an upright rotor), +z the aircraft's right");
        head += ",\"speed\":";
        put_string(head, "|u| of the air relative to the held aircraft at the point, m/s");
        head += ",\"sim\":";
        put_string(head, info.sim);
        head += ",\"config\":";
        put_string(head, info.config);
        head += ",\"note\":";
        put_string(head, info.note);
        head += ",\"field\":{\"frame\":" + std::to_string(info.frame) + ",\"step\":" + std::to_string(info.step) +
                ",\"t_s\":";
        put_number(head, info.t_s, 6);
        head += ",\"cells\":[" + std::to_string(volume.header.nx) + "," + std::to_string(volume.header.ny) + "," +
                std::to_string(volume.header.nz) + "],\"cell_m\":";
        put_number(head, static_cast<double>(volume.header.cell), 6);
        head += "},\"rotors\":[";
        for (std::size_t i = 0; i < rotors.size(); ++i) {
            if (i > 0u) head += ',';
            head += "{\"index\":" + std::to_string(i) + ",\"hub\":";
            put_vec(head, rotors[i].hub, 5);
            head += ",\"axis\":";
            put_vec(head, rotors[i].axis, 5);
            head += ",\"radius\":";
            put_number(head, static_cast<double>(rotors[i].radius), 5);
            head += ",\"spin\":";
            put_number(head, static_cast<double>(rotors[i].spin_dir), 0);
            head += '}';
        }
        head += "],\"seeding\":";
        char height[32];
        std::snprintf(height, sizeof height, "%.2f", static_cast<double>(options.seed_height_radii));
        put_string(head, "rings about each rotor's axis at 0.30, 0.55, 0.80, 1.05 and 1.30 radii, in the plane " +
                             std::string(height) +
                             " radii above the disc along its axis, traced both ways; "
                             "a line ends at the box, at a stagnation point, at no data, or after one box diagonal");
        head += ",\"decimation\":{\"least_arc_length_m\":";
        put_number(head, static_cast<double>(spacing), 4);
        head += ",\"point_decimals\":" + std::to_string(pd) + ",\"speed_decimals\":" + std::to_string(sd) + "}";
        head += ",\"counts\":{\"lines\":" + std::to_string(result.lines) +
                ",\"points\":" + std::to_string(result.points) +
                ",\"dropped_under_two_points\":" + std::to_string(result.dropped) + "}}";

        result.json = "{\"header\":" + head + ",\"lines\":[" + lines + "]}\n";
        reached = result.json.size();
        if (reached <= options.max_bytes) return result;
    }
    return std::unexpected(spade::Error{spade::Code::capacity_exceeded,
                                        "the export is " + std::to_string(reached) + " bytes after " +
                                            std::to_string(kMaxTries) + " decimations, over the " +
                                            std::to_string(options.max_bytes) + " allowed: fewer lines"});
}

std::string export_caption_text(const ExportCaption& in) {
    const ExportInfo& i = *in.info;
    const ExportResult& r = *in.result;
    char t_s[32];
    std::snprintf(t_s, sizeof t_s, "%.4f", i.t_s);
    char spacing[32];
    std::snprintf(spacing, sizeof spacing, "%.1f", static_cast<double>(r.spacing_m) * 1000.0);
    std::string t = "# " + i.name + ": streamlines of the reference quad's wake, in the aircraft's body frame\n\n";
    t += "- Commit: " + in.commit + "\n";
    t += "- Date: " + in.date_utc + "\n";
    t += "- How made: tools/flow_movie, `" + in.command + "`, jobrun ticket " + in.ticket + "\n";
    t += "- Shows: " + std::to_string(r.lines) + " streamlines (" + std::to_string(r.points) +
         " points) of the air relative to the held aircraft, seeded in rings around each rotor disc, at one instant: "
         "frame " + std::to_string(i.frame) + ", step " + std::to_string(i.step) + ", t = " + t_s +
         " s of simulated time from rest. Points are in metres in the airframe model's body frame (+x the nose, "
         "+y up, +z right; origin at the centre of mass), with the air's speed at each point in m/s. A streamline "
         "is a snapshot of an unsteady field, not the path of any particle.\n";
    t += "- Decimation: points at least " + std::string(spacing) + " mm of arc apart, to fit " +
         std::to_string(kExportMaxBytes / 1'000'000u) + " MB.\n";
    t += "- Run: " + i.sim + "; case " + i.config + "\n";
    char cell[32], wind[32];
    std::snprintf(cell, sizeof cell, "%.3g", static_cast<double>(in.cell_mm));
    std::snprintf(wind, sizeof wind, "%.3g", static_cast<double>(in.wind_mps));
    t += "- Stand-in: the reference quad's rigid parts as solids in the flow at " + std::string(cell) +
         " mm, its rotors as actuator discs of fixed thrust and swirl, the drone held level at its still-air hover "
         "throttle";
    if (!(in.wind_mps > 0.0f)) {
        t += " in still air. ";
    } else if (!in.crosswind) {
        t += " in a steady " + std::string(wind) + " m/s headwind. ";
    } else {
        t += " in a steady " + std::string(wind) +
             " m/s wind from the side (crosswind setting supported but not validated). In plain words: the rotor "
             "model's push does not respond to the side wind, and the grid carries the wake downwind. The held drone "
             "does not compensate: this is a visualisation of the air, not a prediction of the thrust, power or "
             "attitude a real quad would need in a crosswind. ";
    }
    t += "The wake below the quad has not been measured against a downwash record. Speeds are relative speeds, "
         "not validated speeds.";
    if (in.device) {
        t += " The fluid is stepped by the GPU solver (GpuFluid), checked bit-equal to the CPU's through about "
             "0.18 s of simulated time only: a single run on the device of record, not the CPU's run.";
    }
    t += "\n";
    return t;
}

std::string data_manifest_line(const std::string& file, uint64_t bytes, const std::string& label,
                               const std::string& description) {
    return "- " + file + " | data, " + std::to_string(bytes) + " bytes | - | " + label + " | " + description;
}

}  // namespace flow_movie
