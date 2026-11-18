// tools/flow_movie/look.hpp -- the look options of the web masters (the lead's L86 approval; Cameron's ask 1, "smooth,
// stable airflow"). Four choices on a published flow volume, each DECLARED in the clip's caption (look_note), none hidden:
//
//   --l2-level L    the vortex surfaces at lambda2-hat = -L (1/s) for the WHOLE clip, in place of 1/6 of each frame's deepest
//                   core, which is the disc rim's strongest cell and drifts three-fold over S2's run;
//   --l2-smooth     lambda2-hat smoothed over one cell (a Gaussian of sigma 1 cell: 7 taps a side, separable). A no-data cell
//                   (a solid) stays no-data and is left out of its neighbours' mean, so a constant field stays constant;
//   --min-cells N   a connected (6-connected) piece of lambda2-hat below -L smaller than N cells is set to 0: below what the
//                   scheme resolves, it is a speck and not a vortex;
//   --mean-frames M the velocity (u, v, w) averaged over the M frames ending at the drawn one, for streamlines whose shapes
//                   would otherwise re-route with every frame.
//
// Pure: a FlowVolume in, a FlowVolume out; no GL, no solver, no clock, so a test links nothing of the tool.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <charconv>
#include <format>
#include <optional>
#include <span>
#include <string_view>
#include <system_error>
#include <string>
#include <vector>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "core/fp16.hpp"
#include "render/scene.hpp"
#include "wake_fade.hpp"

namespace flow_movie {

// The cloud thinned near the drone (--q-look thin_r, thin_floor): the vorticity within `radius` metres of `centre` is scaled by a
// factor from `floor` at the centre back to 1 at `radius` (a smoothstep), so the airframe reads through the haze. Off at radius 0.
struct CloudThin {
    glm::vec3 centre{0.0f};
    float radius = 0.0f;
    float floor = 0.3f;
    [[nodiscard]] bool on() const noexcept { return radius > 0.0f; }
};

struct Look {
    float l2_level = 0.0f;      // 0: the case's own relative level
    bool l2_smooth = false;
    uint32_t min_cells = 0;     // 0: no cull
    uint32_t mean_frames = 1;   // 1: the frame alone
    WakeFade fade;              // the soft edge of a drawn wake: the drone's framing sets it (not an option)
    CloudThin thin;             // the vorticity thinned near the drone (--q-look): the cloud look's

    [[nodiscard]] bool any() const noexcept {
        return l2_level > 0.0f || l2_smooth || min_cells > 0u || mean_frames > 1u || fade.on() || thin.on();
    }
};

namespace look_detail {

inline constexpr std::size_t kL2Lane = spade::kFlowLambda2Hat % spade::kFlowHalvesPerCell;

[[nodiscard]] inline bool no_data(uint16_t half) noexcept { return (half & 0x7C00u) == 0x7C00u; }

}  // namespace look_detail

// channel_min / channel_max of lambda2-hat over its valid cells (0, 0 with none), so a view reading them is not stale; and a
// new generation, since the callers have just written the s texture.
inline void refresh_lambda2_range(spade::FlowVolume& v) {
    float lo = 0.0f, hi = 0.0f;
    bool any = false;
    const std::size_t cells = std::size_t{v.header.nx} * v.header.ny * v.header.nz;
    for (std::size_t c = 0; c < cells; ++c) {
        const uint16_t h = v.s[c * spade::kFlowHalvesPerCell + look_detail::kL2Lane];
        if (look_detail::no_data(h)) continue;
        const float x = spade::fp16::to_f32(h);
        if (!any) {
            lo = hi = x;
            any = true;
        } else {
            lo = std::min(lo, x);
            hi = std::max(hi, x);
        }
    }
    v.header.channel_min[spade::kFlowLambda2Hat] = lo;
    v.header.channel_max[spade::kFlowLambda2Hat] = hi;
    v.generation = spade::next_flow_generation();  // the halves changed: a cache of the old ones must not be drawn
}

// lambda2-hat smoothed over one cell. Three separable passes (x, y, z) of exp(-t^2 / 2) at t = -3 .. 3, each cell's mean over
// the VALID neighbours it can reach (a no-data cell and the box's edge are left out and the weights renormalised), so a
// constant stays constant and a solid is neither read as zero vorticity nor written.
inline void smooth_lambda2(spade::FlowVolume& v) {
    const uint32_t nx = v.header.nx, ny = v.header.ny, nz = v.header.nz;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    std::array<float, 7> w{};
    float norm = 0.0f;
    for (int t = -3; t <= 3; ++t) {
        w[static_cast<std::size_t>(t + 3)] = std::exp(-0.5f * static_cast<float>(t * t));
        norm += w[static_cast<std::size_t>(t + 3)];
    }
    for (float& x : w) x /= norm;
    std::vector<float> a(cells), b(cells);
    std::vector<uint8_t> valid(cells);
    for (std::size_t c = 0; c < cells; ++c) {
        const uint16_t h = v.s[c * spade::kFlowHalvesPerCell + look_detail::kL2Lane];
        valid[c] = look_detail::no_data(h) ? 0u : 1u;
        a[c] = valid[c] != 0u ? spade::fp16::to_f32(h) : 0.0f;
    }
    const auto pass = [&](const std::vector<float>& in, std::vector<float>& out, std::size_t stride, uint32_t extent,
                          uint32_t outer_a, std::size_t stride_a, uint32_t outer_b, std::size_t stride_b) {
        for (uint32_t pb = 0; pb < outer_b; ++pb) {
            for (uint32_t pa = 0; pa < outer_a; ++pa) {
                const std::size_t base = pa * stride_a + pb * stride_b;
                for (uint32_t i = 0; i < extent; ++i) {
                    const std::size_t c = base + i * stride;
                    if (valid[c] == 0u) {
                        out[c] = 0.0f;
                        continue;
                    }
                    float sum = 0.0f, weight = 0.0f;
                    for (int t = -3; t <= 3; ++t) {
                        const int j = static_cast<int>(i) + t;
                        if (j < 0 || j >= static_cast<int>(extent)) continue;
                        const std::size_t n = base + static_cast<std::size_t>(j) * stride;
                        if (valid[n] == 0u) continue;
                        sum += w[static_cast<std::size_t>(t + 3)] * in[n];
                        weight += w[static_cast<std::size_t>(t + 3)];
                    }
                    out[c] = sum / weight;
                }
            }
        }
    };
    const std::size_t sx = 1, sy = nx, sz = std::size_t{nx} * ny;
    pass(a, b, sx, nx, ny, sy, nz, sz);
    pass(b, a, sy, ny, nx, sx, nz, sz);
    pass(a, b, sz, nz, nx, sx, ny, sy);
    for (std::size_t c = 0; c < cells; ++c) {
        if (valid[c] != 0u) v.s[c * spade::kFlowHalvesPerCell + look_detail::kL2Lane] = spade::fp16::from_f32(b[c]).bits;
    }
    refresh_lambda2_range(v);
}

// Sets to 0 every 6-connected piece of valid cells with lambda2-hat below -level that holds fewer than `min_cells` cells.
// Returns the number of pieces removed. 0 cells turns it off. A piece of min_cells or more, and every other cell, is
// untouched.
inline uint32_t cull_small_vortices(spade::FlowVolume& v, float level, uint32_t min_cells) {
    if (min_cells == 0u) return 0u;
    const uint32_t nx = v.header.nx, ny = v.header.ny, nz = v.header.nz;
    const std::size_t cells = std::size_t{nx} * ny * nz;
    const float below = -level;
    const auto inside = [&](std::size_t c) {
        const uint16_t h = v.s[c * spade::kFlowHalvesPerCell + look_detail::kL2Lane];
        return !look_detail::no_data(h) && spade::fp16::to_f32(h) < below;
    };
    std::vector<uint8_t> seen(cells, 0u);
    std::vector<std::size_t> piece, stack;
    uint32_t removed = 0;
    for (std::size_t seed = 0; seed < cells; ++seed) {
        if (seen[seed] != 0u || !inside(seed)) continue;
        piece.clear();
        stack.assign(1, seed);
        seen[seed] = 1u;
        while (!stack.empty()) {
            const std::size_t c = stack.back();
            stack.pop_back();
            piece.push_back(c);
            const uint32_t i = static_cast<uint32_t>(c % nx), j = static_cast<uint32_t>(c / nx % ny),
                           k = static_cast<uint32_t>(c / (std::size_t{nx} * ny));
            const auto visit = [&](bool ok, std::size_t n) {
                if (ok && seen[n] == 0u && inside(n)) {
                    seen[n] = 1u;
                    stack.push_back(n);
                }
            };
            visit(i > 0u, c - 1u);
            visit(i + 1u < nx, c + 1u);
            visit(j > 0u, c - nx);
            visit(j + 1u < ny, c + nx);
            visit(k > 0u, c - std::size_t{nx} * ny);
            visit(k + 1u < nz, c + std::size_t{nx} * ny);
        }
        if (piece.size() < min_cells) {
            for (const std::size_t c : piece) v.s[c * spade::kFlowHalvesPerCell + look_detail::kL2Lane] = 0u;
            ++removed;
        }
    }
    refresh_lambda2_range(v);
    return removed;
}

// After the velocity halves were written: the header's finite range of u, v and w, its count of no-data halves (both textures)
// and a new generation, so a cache of the old halves is not drawn.
inline void refresh_velocity_range(spade::FlowVolume& out) {
    const std::size_t cells = std::size_t{out.header.nx} * out.header.ny * out.header.nz;
    for (std::size_t lane = 0; lane < 3u; ++lane) {
        float lo = 0.0f, hi = 0.0f;
        bool any = false;
        for (std::size_t c = 0; c < cells; ++c) {
            const uint16_t half = out.v[c * spade::kFlowHalvesPerCell + lane];
            if (look_detail::no_data(half)) continue;
            const float x = spade::fp16::to_f32(half);
            lo = any ? std::min(lo, x) : x;
            hi = any ? std::max(hi, x) : x;
            any = true;
        }
        out.header.channel_min[lane] = lo;
        out.header.channel_max[lane] = hi;
    }
    uint32_t missing = 0;
    for (const uint16_t half : out.v) missing += look_detail::no_data(half) ? 1u : 0u;
    for (const uint16_t half : out.s) missing += look_detail::no_data(half) ? 1u : 0u;
    out.header.no_data = missing;
    out.generation = spade::next_flow_generation();
}

// `out` = the last of `frames` with its velocity (u, v, w) the mean of all of them. A cell that is no-data in any frame is
// no-data in the mean. Refused by name: no frames, and frames of different shapes. Every other channel is the last
// frame's (the drawn one), and `out` may be that frame.
[[nodiscard]] inline spade::Result<void> mean_velocity(spade::FlowVolume& out,
                                                         std::span<const spade::FlowVolume* const> frames) {
    if (frames.empty()) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "mean flow: there are no frames to average"});
    }
    const spade::FlowVolumeHeader& h0 = frames.front()->header;
    for (const spade::FlowVolume* f : frames) {
        if (f->header.nx != h0.nx || f->header.ny != h0.ny || f->header.nz != h0.nz) {
            return std::unexpected(spade::Error{spade::Code::invalid_argument,
                                                "mean flow: the frames' grids differ in shape"});
        }
    }
    const std::size_t cells = std::size_t{h0.nx} * h0.ny * h0.nz;
    std::vector<uint16_t> velocity(cells * 3u);
    for (std::size_t c = 0; c < cells; ++c) {
        for (std::size_t lane = 0; lane < 3u; ++lane) {
            float sum = 0.0f;
            bool bad = false;
            for (const spade::FlowVolume* f : frames) {
                const uint16_t half = f->v[c * spade::kFlowHalvesPerCell + lane];
                if (look_detail::no_data(half)) {
                    bad = true;
                    break;
                }
                sum += spade::fp16::to_f32(half);
            }
            velocity[c * 3u + lane] =
                bad ? spade::fp16::kNoDataBits : spade::fp16::from_f32(sum / static_cast<float>(frames.size())).bits;
        }
    }
    if (&out != frames.back()) out = *frames.back();
    for (std::size_t c = 0; c < cells; ++c) {
        for (std::size_t lane = 0; lane < 3u; ++lane) out.v[c * spade::kFlowHalvesPerCell + lane] = velocity[c * 3u + lane];
    }
    refresh_velocity_range(out);
    return {};
}

// The cached frames a mean of `mean_frames` reads for the drawn frame k: the mean_frames ending at k, fewer at the start.
struct MeanWindow {
    uint32_t first = 0;
    uint32_t count = 1;
};

[[nodiscard]] inline MeanWindow mean_window(uint32_t k, uint32_t mean_frames) {
    const uint32_t want = std::max(mean_frames, 1u);
    const uint32_t count = std::min(want, k + 1u);
    return MeanWindow{k + 1u - count, count};
}

// --l2-level text: one positive finite number below 10000 (1/s), whole: digits and at most one '.', nothing else.
[[nodiscard]] inline std::optional<float> parse_l2_level(std::string_view text) {
    if (text.empty()) return std::nullopt;
    bool digit = false, point = false;
    for (const char c : text) {
        if (c >= '0' && c <= '9') {
            digit = true;
        } else if (c == '.' && !point) {
            point = true;
        } else {
            return std::nullopt;
        }
    }
    if (!digit) return std::nullopt;
    float value = 0.0f;
    const auto done = std::from_chars(text.data(), text.data() + text.size(), value);
    if (done.ec != std::errc{} || done.ptr != text.data() + text.size()) return std::nullopt;
    if (!(value > 0.0f && value < 10000.0f)) return std::nullopt;
    return value;
}

// --min-cells and --mean-frames text: a whole number of digits, below 1,000,000.
[[nodiscard]] inline std::optional<uint32_t> parse_count(std::string_view text) {
    if (text.empty() || text.size() > 6u) return std::nullopt;
    uint32_t value = 0;
    const auto done = std::from_chars(text.data(), text.data() + text.size(), value);
    if (done.ec != std::errc{} || done.ptr != text.data() + text.size()) return std::nullopt;
    return value;
}

// The vortex surface's level for a frame, or nothing to draw. With --l2-level, the clip's one absolute level -L, drawn
// while the frame's deepest core is below it. Without, the case's own: `fraction` of the frame's deepest core, only when
// that core is below `floor` (above it, as in a box of uniform wind, the deepest value is the grid's noise).
[[nodiscard]] inline std::optional<float> vortex_iso(const spade::FlowVolume& v, const Look& look, float fraction,
                                                     float floor) {
    const float deepest = v.header.channel_min[spade::kFlowLambda2Hat];
    if (look.l2_level > 0.0f) {
        if (deepest < -look.l2_level) return -look.l2_level;
        return std::nullopt;
    }
    if (deepest < floor) return deepest * fraction;
    return std::nullopt;
}

// Why a look option cannot apply to a clip ("" when it can, or when there is none). The level, the smoothing and the cull
// belong to the clip that draws vortex surfaces (S3, in the side wind; S2 was one until Cameron's pick of the soft volume); the mean flow to the D1 clips that read the
// velocity (not S4's C_p); a cull needs the level that says what a piece is. `d1`: the clip runs the D1 film (S5 and S6 run
// the reference quad's CPU film).
[[nodiscard]] inline std::string look_refusal(std::string_view clip_id, bool d1, const Look& look) {
    if (!look.any()) return {};
    const std::string id(clip_id);
    if (!d1) return "the look options are for the D1 clips (S1 to S4 and S8); " + id + " runs the reference quad's CPU film";
    const bool vortex_clip = clip_id == "S3";
    if ((look.l2_level > 0.0f || look.l2_smooth || look.min_cells > 0u) && !vortex_clip) {
        return "--l2-level, --l2-smooth and --min-cells are for the clip that draws vortex surfaces (S3); " + id +
               " does not";
    }
    if (look.mean_frames > 1u && clip_id == "S4") {
        return "--mean-frames is for the clips that read the velocity; S4 draws the pressure coefficient";
    }
    if (look.min_cells > 0u && look.l2_level <= 0.0f) {
        return "--min-cells needs --l2-level: a piece is the cells below one level";
    }
    return {};
}

// S1's soft streamlines: a polyline's ALPHAS fall to nothing toward both of its ends, by a smoothstep of the arc length to the
// nearer end over `fade_m` metres, so the line dissolves instead of stopping (a line shorter than twice `fade_m` never reaches
// full). The colours are left alone: a line dimmed toward black darkens what is behind it, the drone among it; one of lower alpha
// lets it show (the lead, 2026-11-17). Zero `fade_m`, or a line of fewer than two points, changes nothing. Needs the line drawn as
// a capsule (the scene's line style), as the soft S1 view does.
inline void fade_polyline_ends(spade::render::FlowPolyline& line, float fade_m) {
    const std::size_t n = std::min(line.points.size(), line.colours.size());
    if (!(fade_m > 0.0f) || n < 2u) return;
    std::vector<float> arc(n, 0.0f);
    for (std::size_t i = 1; i < n; ++i) {
        const glm::vec3 d = line.points[i] - line.points[i - 1];
        arc[i] = arc[i - 1] + std::sqrt(glm::dot(d, d));
    }
    const float total = arc[n - 1];
    line.alphas.assign(line.points.size(), 1.0f);
    for (std::size_t i = 0; i < n; ++i) {
        const float t = std::clamp(std::min(arc[i], total - arc[i]) / fade_m, 0.0f, 1.0f);
        line.alphas[i] = t * t * (3.0f - 2.0f * t);
    }
}

// The same for every polyline of `scene` from index `from` on (the lines a view added: the rims drawn before stay as they are).
inline void fade_polylines_from(spade::render::RenderScene& scene, std::size_t from, float fade_m) {
    for (std::size_t i = from; i < scene.flow_polylines.size(); ++i) fade_polyline_ends(scene.flow_polylines[i], fade_m);
}

// The wake's soft edge: u, v and w of every cell scaled by the fade of the side faces and of the height (see WakeFade); the dye,
// a no-data half and the second texture are not touched. The header's velocity range is made again and the volume names new
// content.
inline void fade_wake_edges(spade::FlowVolume& v, const WakeFade& fade) {
    if (!fade.on()) return;
    const spade::FlowVolumeHeader& h = v.header;
    const auto smooth = [](float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };
    const auto side_factor = [&](uint32_t i, uint32_t n) {
        if (!(fade.side_m > 0.0f)) return 1.0f;
        const float from_low = (static_cast<float>(i) + 0.5f) * h.cell;
        const float from_high = (static_cast<float>(n - i) - 0.5f) * h.cell;
        return smooth(std::min(from_low, from_high) / fade.side_m);
    };
    std::vector<float> fx(h.nx), fy(h.ny), fz(h.nz);
    for (uint32_t i = 0; i < h.nx; ++i) fx[i] = side_factor(i, h.nx);
    for (uint32_t k = 0; k < h.nz; ++k) fz[k] = side_factor(k, h.nz);
    for (uint32_t j = 0; j < h.ny; ++j) {
        const float y = h.origin.y + (static_cast<float>(j) + 0.5f) * h.cell;
        float f = 1.0f;
        if (fade.floor_to_y < fade.floor_from_y) f *= 1.0f - smooth((fade.floor_from_y - y) / (fade.floor_from_y - fade.floor_to_y));
        if (fade.ceiling_to_y > fade.ceiling_from_y) f *= 1.0f - smooth((y - fade.ceiling_from_y) / (fade.ceiling_to_y - fade.ceiling_from_y));
        fy[j] = f;
    }
    for (uint32_t k = 0; k < h.nz; ++k) {
        for (uint32_t j = 0; j < h.ny; ++j) {
            const float fyz = fy[j] * fz[k];
            for (uint32_t i = 0; i < h.nx; ++i) {
                const float f = fyz * fx[i];
                if (f >= 1.0f) continue;
                const std::size_t base = spade::flow_cell_index(h, i, j, k) * spade::kFlowHalvesPerCell;
                for (std::size_t lane = 0; lane < 3u; ++lane) {
                    const uint16_t half = v.v[base + lane];
                    if (look_detail::no_data(half)) continue;
                    v.v[base + lane] = spade::fp16::from_f32(spade::fp16::to_f32(half) * f).bits;
                }
            }
        }
    }
    refresh_velocity_range(v);
}

// The vorticity channel (|omega|, the s texture's lane 1) scaled near `thin.centre`; the velocity, the dye, the other channels and the
// header (the colours keep the unthinned frame's range) are not touched, and a no-data half is left as it is.
inline void thin_vorticity(spade::FlowVolume& v, const CloudThin& thin) {
    if (!thin.on()) return;
    const spade::FlowVolumeHeader& h = v.header;
    constexpr std::size_t lane = spade::kFlowVorticity % spade::kFlowHalvesPerCell;
    for (uint32_t k = 0; k < h.nz; ++k) {
        for (uint32_t j = 0; j < h.ny; ++j) {
            for (uint32_t i = 0; i < h.nx; ++i) {
                const glm::vec3 at = h.origin + (glm::vec3(static_cast<float>(i), static_cast<float>(j), static_cast<float>(k)) + 0.5f) * h.cell;
                const float x = std::min(glm::length(at - thin.centre) / thin.radius, 1.0f);
                if (x >= 1.0f) continue;
                const float f = thin.floor + (1.0f - thin.floor) * (x * x * (3.0f - 2.0f * x));
                const std::size_t index = spade::flow_cell_index(h, i, j, k) * spade::kFlowHalvesPerCell + lane;
                const uint16_t half = v.s[index];
                if (look_detail::no_data(half)) continue;
                v.s[index] = spade::fp16::from_f32(spade::fp16::to_f32(half) * f).bits;
            }
        }
    }
}

// What a look does to a frame's fields once its velocity is the mean (or the frame's own): lambda2-hat smoothed, its small
// pieces culled, the wake faded at its edges, in that order. One function, so the drawing and its test cannot part ways.
inline void finish_look(const Look& look, spade::FlowVolume& volume) {
    if (look.l2_smooth) smooth_lambda2(volume);
    if (look.min_cells > 0u) cull_small_vortices(volume, look.l2_level, look.min_cells);
    if (look.fade.on()) fade_wake_edges(volume, look.fade);
    if (look.thin.on()) thin_vorticity(volume, look.thin);
}

// The caption's sentence: what the look did, in plain words ("" when it did nothing).
[[nodiscard]] inline std::string look_note(const Look& look, double frame_ms) {
    std::string note;
    const auto add = [&note](const std::string& s) { note += (note.empty() ? "" : "; ") + s; };
    if (look.l2_level > 0.0f) {
        add(std::format("vortex surfaces at lambda2-hat = -{:g} 1/s, one level for the whole clip", look.l2_level));
    }
    if (look.l2_smooth) add("lambda2 smoothed over one cell");
    if (look.min_cells > 0u) add(std::format("vortex pieces under {} cells left out", look.min_cells));
    if (look.mean_frames > 1u) {
        add(std::format("velocity averaged over the {} frames ending at each (about {:g} ms of flow)", look.mean_frames,
                        frame_ms * look.mean_frames));
    }
    if (look.thin.on()) {
        add(std::format("vorticity thinned to {:g} of its value at the drone, back to full {:g} m out", look.thin.floor, look.thin.radius));
    }
    if (look.fade.on()) {
        std::string where;
        if (look.fade.side_m > 0.0f) {
            where = std::format("over the last {:g} m before each side face of the box", look.fade.side_m);
        }
        if (look.fade.floor_to_y < look.fade.floor_from_y) {
            where += std::format("{}below {:g} m and gone at {:g} m under the drone's centre", where.empty() ? "" : " and ",
                                 -look.fade.floor_from_y, -look.fade.floor_to_y);
        }
        if (look.fade.ceiling_to_y > look.fade.ceiling_from_y) {
            where += std::format("{}above {:g} m and gone at {:g} m over the drone's centre (the slow inflow above the rotors)",
                                 where.empty() ? "" : " and ", look.fade.ceiling_from_y, look.fade.ceiling_to_y);
        }
        add("the wake fades out " + where + ", so the box's edges are not drawn");
    }
    return note;
}

}  // namespace flow_movie
