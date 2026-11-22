// The particle look's seeds (Cameron, 2026-11-17: the airflow should surround the blades and feel like particles): where air
// is drawn in above and round each rotor, and a far-field box round the drone. A deterministic hash, no RNG: the same inputs
// give the same points. Used by the particle preview (tracers carried through one frozen frame) and by the real particle
// system after it.
#pragma once

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <optional>
#include <map>
#include <numbers>
#include <span>
#include <utility>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <glm/glm.hpp>

#include "core/error.hpp"
#include "core/flow_volume.hpp"
#include "render/flow_lines.hpp"
#include "render/flow_sample.hpp"

namespace flow_movie {

struct PreviewRotor {
    glm::vec3 hub;
    glm::vec3 axis;  // the way thrust points: "above" is along it
    float radius;
};

struct PreviewBox {
    glm::vec3 lo, hi;
};

namespace particles_detail {

// splitmix64 to [0, 1): a stateless hash of an index and a stream, so a seed never depends on how many came before it.
[[nodiscard]] inline float unit(uint64_t index, uint64_t stream) noexcept {
    uint64_t z = index * 0x9E3779B97F4A7C15ull + stream * 0xBF58476D1CE4E5B9ull + 0x94D049BB133111EBull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    z ^= z >> 31;
    return static_cast<float>(z >> 40) * (1.0f / 16777216.0f);
}

}  // namespace particles_detail

// `per_rotor` seeds in the shell above each rotor (0.02 to 0.35 m along its axis, thicker near the disc, within two radii of the
// axis), then `far` seeds filling `box`. Rotor r's seeds come first, in order.
[[nodiscard]] inline std::vector<glm::vec3> preview_seeds(const std::vector<PreviewRotor>& rotors, const PreviewBox& box,
                                                          std::size_t per_rotor, std::size_t far) {
    using particles_detail::unit;
    std::vector<glm::vec3> out;
    out.reserve(rotors.size() * per_rotor + far);
    for (std::size_t r = 0; r < rotors.size(); ++r) {
        const PreviewRotor& rotor = rotors[r];
        const glm::vec3 axis = glm::normalize(rotor.axis);
        const glm::vec3 helper = std::abs(axis.y) < 0.9f ? glm::vec3(0.0f, 1.0f, 0.0f) : glm::vec3(1.0f, 0.0f, 0.0f);
        const glm::vec3 e1 = glm::normalize(glm::cross(axis, helper));
        const glm::vec3 e2 = glm::cross(axis, e1);
        for (std::size_t i = 0; i < per_rotor; ++i) {
            const uint64_t k = r * 1000003ull + i;
            const float theta = 2.0f * std::numbers::pi_v<float> * unit(k, 1);
            const float radial = rotor.radius * (0.05f + 1.95f * std::sqrt(unit(k, 2)));  // an even fill of the disc of radius 2R
            const float height = 0.02f + 0.33f * unit(k, 3) * unit(k, 3);                  // thicker near the disc
            out.push_back(rotor.hub + axis * height + (e1 * std::cos(theta) + e2 * std::sin(theta)) * radial);
        }
    }
    for (std::size_t i = 0; i < far; ++i) {
        const uint64_t k = 7000000ull + i;
        out.emplace_back(box.lo.x + (box.hi.x - box.lo.x) * unit(k, 4), box.lo.y + (box.hi.y - box.lo.y) * unit(k, 5),
                         box.lo.z + (box.hi.z - box.lo.z) * unit(k, 6));
    }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------------
// The particle field: points the stored frames carry (Cameron, 2026-11-17: the air drawn in above and round the rotors, through
// them and down, as particles). One advance_particles() call is one stored frame: every live point moves by one midpoint (RK2)
// step of dt through that frame's velocity, those that must go are retired by reason, and the frame's slice of a fixed seed
// pool is emitted. A point keeps where it was a frame ago, so a draw can streak it. No RNG and no clock: the same frames, pool
// and options give the same field, bit for bit.
//
// A point RETIRES, counted by reason, when a step ends outside the region (the box the look keeps), when either stage meets
// no-data, or past max_age_s. Capacity is fixed: an emission past it is dropped and counted, never a silent grow.
// ---------------------------------------------------------------------------------------------------------------------------
struct ParticleOptions {
    uint32_t capacity = 100000u;        // live points, 1 to 2^24
    float max_age_s = 0.5f;             // > 0
    uint32_t per_frame = 0u;            // seeds emitted a frame, from the pool in order, wrapping
    uint32_t substeps = 1u;             // midpoint steps a frame, 1 to 64: each dt / substeps through the same frame (1: one step, as it was)
    glm::vec3 region_lo{-1.0e9f};       // a point whose step ends outside [lo, hi] is retired
    glm::vec3 region_hi{1.0e9f};
};

struct ParticleField {
    std::vector<glm::vec3> position;
    std::vector<glm::vec3> previous;  // where the point was a frame ago; the seed itself for a new point (no streak)
    std::vector<float> age;           // seconds since emission
    uint32_t frames = 0;              // frames advanced so far
    uint64_t emitted = 0;             // seeds taken from the pool so far, the dropped ones too: the schedule is the pool's
    uint64_t dropped = 0;
    uint64_t retired_left_region = 0;
    uint64_t retired_no_data = 0;
    uint64_t retired_aged = 0;
};

// invalid_argument, by name, before any change: a volume that fails validate_flow_volume(), dt not finite and > 0, a capacity
// outside 1 to 2^24, max_age_s not finite and > 0, a region that is not finite and lo <= hi, per_frame > 0 with an empty pool,
// a pool point that is not finite, or a field whose arrays disagree in length or exceed the capacity.
[[nodiscard]] inline spade::Result<void> advance_particles(ParticleField& field, const spade::FlowVolume& volume, const spade::FlowVolume* next_frame,
                                                           std::span<const glm::vec3> pool, float dt, const ParticleOptions& options) {
    const auto bad = [](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "particles: " + why});
    };
    if (auto valid = spade::validate_flow_volume(volume); !valid) return std::unexpected(valid.error());
    if (next_frame != nullptr) {
        if (auto valid = spade::validate_flow_volume(*next_frame); !valid) return std::unexpected(valid.error());
        const spade::FlowVolumeHeader& a = volume.header;
        const spade::FlowVolumeHeader& b = next_frame->header;
        if (a.nx != b.nx || a.ny != b.ny || a.nz != b.nz || a.cell != b.cell || a.origin != b.origin) return bad("the next frame's grid differs from the drawn frame's");
    }
    if (!std::isfinite(dt) || dt <= 0.0f) return bad("dt must be finite and > 0");
    if (options.capacity < 1u || options.capacity > (1u << 24)) return bad("capacity must be 1 to 2^24");
    if (!std::isfinite(options.max_age_s) || options.max_age_s <= 0.0f) return bad("max_age_s must be finite and > 0");
    for (int a = 0; a < 3; ++a) {
        if (!std::isfinite(options.region_lo[a]) || !std::isfinite(options.region_hi[a]) ||
            options.region_lo[a] > options.region_hi[a]) {
            return bad("the region must be finite with lo <= hi");
        }
    }
    if (options.substeps < 1u || options.substeps > 64u) return bad("substeps must be 1 to 64");
    if (options.per_frame > 0u && pool.empty()) return bad("per_frame > 0 needs a non-empty seed pool");
    for (const glm::vec3& p : pool) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return bad("a seed is not finite");
    }
    if (field.previous.size() != field.position.size() || field.age.size() != field.position.size() ||
        field.position.size() > options.capacity) {
        return bad("the field's arrays disagree in length or exceed the capacity");
    }

    const spade::render::FlowSampler sampler = spade::render::make_flow_sampler(volume);
    // B1: with the next frame, the velocity of a stage is the drawn frame's blended toward the next's by the stage's time through the interval (0 at its start, 1 at the next frame's time); without
    // it (a streamed run, which holds one frame), the drawn frame's alone, exactly as before.
    std::optional<spade::render::FlowSampler> sampler_next;
    if (next_frame != nullptr) sampler_next = spade::render::make_flow_sampler(*next_frame);
    const auto velocity_at = [&](glm::vec3 at, float weight, bool& no_data) -> glm::vec3 {
        const spade::render::FlowVelocity a = spade::render::sample_velocity(sampler, at);
        if (a.no_data) {
            no_data = true;
            return glm::vec3(0.0f);
        }
        if (!sampler_next) return a.u;
        const spade::render::FlowVelocity b = spade::render::sample_velocity(*sampler_next, at);
        if (b.no_data) {
            no_data = true;
            return glm::vec3(0.0f);
        }
        return a.u + (b.u - a.u) * weight;
    };
    const auto inside = [&](glm::vec3 p) {
        return p.x >= options.region_lo.x && p.x <= options.region_hi.x && p.y >= options.region_lo.y &&
               p.y <= options.region_hi.y && p.z >= options.region_lo.z && p.z <= options.region_hi.z;
    };
    const float h = dt / static_cast<float>(options.substeps);
    std::size_t kept = 0;
    for (std::size_t i = 0; i < field.position.size(); ++i) {
        const glm::vec3 p = field.position[i];
        // `substeps` midpoint steps of h = dt / substeps through this frame's velocity (one step of dt when substeps is 1, exactly as before); no-data at either
        // stage of any of them retires the point.
        glm::vec3 next = p;
        bool lost = false;
        for (uint32_t s = 0; s < options.substeps && !lost; ++s) {
            const float n_sub = static_cast<float>(options.substeps);
            bool stage_lost = false;
            const glm::vec3 k1 = velocity_at(next, static_cast<float>(s) / n_sub, stage_lost);
            if (stage_lost) {
                lost = true;
                break;
            }
            const glm::vec3 k2 = velocity_at(next + k1 * (0.5f * h), (static_cast<float>(s) + 0.5f) / n_sub, stage_lost);
            if (stage_lost) {
                lost = true;
                break;
            }
            next = next + k2 * h;
        }
        if (lost) {
            ++field.retired_no_data;
            continue;
        }
        const float age = field.age[i] + dt;
        if (!inside(next)) {
            ++field.retired_left_region;
            continue;
        }
        if (age > options.max_age_s) {
            ++field.retired_aged;
            continue;
        }
        field.position[kept] = next;
        field.previous[kept] = p;
        field.age[kept] = age;
        ++kept;
    }
    field.position.resize(kept);
    field.previous.resize(kept);
    field.age.resize(kept);

    for (uint32_t i = 0; i < options.per_frame; ++i) {
        const glm::vec3 seed = pool[static_cast<std::size_t>(field.emitted % pool.size())];
        ++field.emitted;
        if (field.position.size() >= options.capacity) {
            ++field.dropped;
            continue;
        }
        field.position.push_back(seed);
        field.previous.push_back(seed);
        field.age.push_back(0.0f);
    }
    ++field.frames;
    return {};
}

// The drawn frame alone (a streamed run holds one frame): the velocity is held for the whole interval, as before B1.
[[nodiscard]] inline spade::Result<void> advance_particles(ParticleField& field, const spade::FlowVolume& volume, std::span<const glm::vec3> pool, float dt, const ParticleOptions& options) {
    return advance_particles(field, volume, nullptr, pool, dt, options);
}

// The frames a particle run reads: the clip's draws and every frame before its first, in order, each once. The points are
// carried through all of them (the inflow must already be drawn in when the clip starts), so the stream's plan writes them
// and keeps each until its advance is done.
//
// The driver is the run's one particle field: advanced once per stored frame, in order, with the frame's simulated time.
// `snapshot_frames` are the frames a still will be drawn from; the still is drawn after the solve, when the live field is long
// past them, so a copy is kept as each of those frames is advanced. at(k) is the field that shows frame k: its copy, or the
// live field while it is at k; nothing otherwise.
class ParticleDriver {
public:
    ParticleDriver(ParticleOptions options, std::vector<uint32_t> snapshot_frames)
        : options_(options), snapshot_frames_(std::move(snapshot_frames)) {}

    void set_pool(std::vector<glm::vec3> pool) { pool_ = std::move(pool); }
    [[nodiscard]] bool has_pool() const noexcept { return !pool_.empty(); }

    // k must be the next frame (0 first), and time_s past the last frame's: the step is their difference (frame 0 moves
    // nothing, there is nothing to move). A region left open is the first volume's box less one cell.
    // `next_frame`, when the run has it (a run drawn from a kept cache does), is blended in by time (advance_particles); STUB (RED): ignored for now.
    [[nodiscard]] spade::Result<void> advance(uint32_t k, const spade::FlowVolume& volume, double time_s, const spade::FlowVolume* next_frame = nullptr) {
        (void)next_frame;
        const auto bad = [](const std::string& why) {
            return std::unexpected(spade::Error{spade::Code::invalid_argument, "particles: " + why});
        };
        if (k != next_) return bad("frame " + std::to_string(k) + " is not the next frame, " + std::to_string(next_));
        if (pool_.empty()) return bad("no seed pool was set");
        if (k > 0u && !(time_s > last_time_)) return bad("time must go forward between frames");
        ParticleOptions o = options_;
        if (!region_done_) {
            if (o.region_lo.x <= -1.0e8f && o.region_hi.x >= 1.0e8f) {
                const spade::FlowVolumeHeader& h = volume.header;
                const glm::vec3 lo = h.origin + glm::vec3(h.cell);
                const glm::vec3 hi = h.origin + glm::vec3(static_cast<float>(h.nx), static_cast<float>(h.ny),
                                                          static_cast<float>(h.nz)) * h.cell - glm::vec3(h.cell);
                options_.region_lo = lo;
                options_.region_hi = hi;
                o = options_;
            }
            region_done_ = true;
        }
        const float dt = k == 0u ? 1.0e-3f : static_cast<float>(time_s - last_time_);
        if (auto stepped = advance_particles(live_, volume, pool_, dt, o); !stepped) return stepped;
        if (std::find(snapshot_frames_.begin(), snapshot_frames_.end(), k) != snapshot_frames_.end()) snapshots_[k] = live_;
        last_time_ = time_s;
        ++next_;
        return {};
    }

    // A field made earlier (a run's snapshot, read back) as the one that shows frame k.
    void add_snapshot(uint32_t k, ParticleField field) { snapshots_[k] = std::move(field); }

    // The copy kept at frame k, or nothing: what a run writes out beside its kept frame.
    [[nodiscard]] const ParticleField* snapshot_at(uint32_t k) const {
        const auto it = snapshots_.find(k);
        return it == snapshots_.end() ? nullptr : &it->second;
    }

    [[nodiscard]] const ParticleField* at(uint32_t k) const {
        if (const auto it = snapshots_.find(k); it != snapshots_.end()) return &it->second;
        if (next_ > 0u && k + 1u == next_) return &live_;
        return nullptr;
    }

private:
    ParticleOptions options_;
    std::vector<uint32_t> snapshot_frames_;
    std::vector<glm::vec3> pool_;
    ParticleField live_;
    std::map<uint32_t, ParticleField> snapshots_;
    uint32_t next_ = 0;
    double last_time_ = 0.0;
    bool region_done_ = false;
};

// A still of the particle look from a run's KEPT frames, with no solve (L106b): the driver carried through frames 0 to `last` in order, each read once.
// `read(k, volume, time_s)` fills frame k's volume and its simulated time (the step is the difference between frames, as a streamed run's is) or returns the
// error; `pool_of()` gives the seed pool, asked once, right after frame 0 is read (the pool comes from a frame's log, the held drone's rotors). The driver
// keeps its snapshots as in a streamed run, so at(last) shows the field the still draws. A frame that cannot be read or advanced stops the pass, named.
template <typename Read, typename PoolOf>
[[nodiscard]] spade::Result<void> advance_through_frames(ParticleDriver& driver, uint32_t last, Read&& read, PoolOf&& pool_of) {
    spade::FlowVolume volume;
    double time_s = 0.0;
    for (uint32_t k = 0; k <= last; ++k) {
        if (auto got = read(k, volume, time_s); !got) {
            return std::unexpected(spade::Error{got.error().code, "particles: frame " + std::to_string(k) + " of 0 to " + std::to_string(last) + ": " + got.error().context});
        }
        if (k == 0u && !driver.has_pool()) driver.set_pool(pool_of());
        if (auto stepped = driver.advance(k, volume, time_s); !stepped) return stepped;
    }
    return {};
}

// The pool in a fixed other order: element i of the result is pool[(i s) mod N], s the integer nearest 0.618 N made coprime to N
// (the golden-ratio stride, which spreads any short run over the whole pool). A pool built shell by shell would otherwise emit
// the rotor shells for half its cycle and the far field for the rest. Same points, no RNG.
[[nodiscard]] inline std::vector<glm::vec3> spread_pool(const std::vector<glm::vec3>& pool) {
    const std::size_t n = pool.size();
    if (n <= 2u) return pool;
    std::size_t stride = static_cast<std::size_t>(std::llround(0.6180339887 * static_cast<double>(n)));
    while (std::gcd(stride, n) != 1u) ++stride;
    std::vector<glm::vec3> out;
    out.reserve(n);
    for (std::size_t i = 0; i < n; ++i) out.push_back(pool[(i * stride) % n]);
    return out;
}

// The field as drawn points: one per particle, bright blue-white where it was seeded, dimmer and bluer as it ages to max_age_s.
[[nodiscard]] inline spade::render::FlowPoints particle_points(const ParticleField& field, float max_age_s, uint32_t size_px) {
    spade::render::FlowPoints points;
    points.size_px = size_px;
    points.positions = field.position;
    points.colours.reserve(field.position.size());
    for (const float age : field.age) {
        const float t = std::clamp(age / max_age_s, 0.0f, 1.0f);
        const glm::vec3 c = glm::mix(glm::vec3(0.85f, 0.97f, 1.0f), glm::vec3(0.10f, 0.25f, 0.75f), t) * (1.0f - 0.5f * t);
        points.colours.push_back(glm::clamp(c, glm::vec3(0.0f), glm::vec3(1.0f)));
    }
    return points;
}

// THE DRAWN STREAKS, for the GL capsule path with additive light (FlowLineStyle::additive, FlowPolyline::alphas). One two-point
// polyline per particle the field holds and the volume has a velocity for: the tail is where the point was `tail_s` ago along
// its own velocity (the streak is as long as the air is fast; still air is a point), the head is the point; alpha is nothing at the
// tail and the head's alpha at the head; the colour is by speed, deep blue and dim when slow to warm white and bright at fast_mps
// and over. A point is born invisible and fades in over fade_in_s and fades out over the last 0.4 of max_age_s, so recycling never
// pops. A particle in no-data has no velocity and is left out; the order is the field's.
struct StreakLook {
    float tail_s = 0.0025f;   // how far back along the velocity the tail reaches, seconds
    float width_px = 2.0f;
    float fade_in_s = 0.04f;
    float fast_mps = 10.0f;   // the speed at which the colour reaches its warm end
    float peak_alpha = 0.9f;  // the head's alpha when full
    float min_len_m = 0.0f;   // a slow streak is never shorter than this (0: u times tail_s, as it was)
    float glow_px = 5.0f;     // the capsules' glow (the scene's line style)
    float glow_strength = 0.30f;
    // Streaks near the drone are dimmed so the airframe reads through the light: alpha x (1 - dim) at dim_centre, smoothly back to
    // untouched at dim_r metres. dim 0 is off.
    glm::vec3 dim_centre{0.0f};
    float dim_r = 0.15f;
    float dim = 0.0f;
    // Not a drawing number: how a frame is ADVANCED (ParticleOptions::substeps), set from the same --particle-look string. 1: one midpoint step a frame.
    uint32_t substeps = 1u;
};

[[nodiscard]] inline std::vector<spade::render::FlowPolyline> particle_streaks(const ParticleField& field,
                                                                              const spade::FlowVolume& volume,
                                                                              float max_age_s, const StreakLook& look = {}) {
    const auto smoothstep = [](float e0, float e1, float x) {
        const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
        return t * t * (3.0f - 2.0f * t);
    };
    const spade::render::FlowSampler sampler = spade::render::make_flow_sampler(volume);
    std::vector<spade::render::FlowPolyline> out;
    out.reserve(field.position.size());
    for (std::size_t i = 0; i < field.position.size(); ++i) {
        const spade::render::FlowVelocity u = spade::render::sample_velocity(sampler, field.position[i]);
        if (u.no_data) continue;
        const float speed = glm::length(u.u);
        const float s = std::clamp(speed / look.fast_mps, 0.0f, 1.0f);
        const glm::vec3 colour = glm::clamp(
            glm::mix(glm::vec3(0.15f, 0.35f, 1.0f), glm::vec3(1.0f, 0.93f, 0.80f), s) * (0.35f + 0.65f * s), glm::vec3(0.0f),
            glm::vec3(1.0f));
        const float age = field.age[i];
        float alpha = look.peak_alpha * smoothstep(0.0f, look.fade_in_s, age) *
                      (1.0f - smoothstep(0.6f * max_age_s, max_age_s, age));
        if (look.dim > 0.0f) {
            alpha *= 1.0f - look.dim * (1.0f - smoothstep(0.0f, look.dim_r, glm::length(field.position[i] - look.dim_centre)));
        }
        spade::render::FlowPolyline line;
        glm::vec3 reach = u.u * look.tail_s;  // from the head back to the tail
        if (look.min_len_m > 0.0f && speed > 1.0e-6f) reach = (u.u / speed) * std::max(speed * look.tail_s, look.min_len_m);
        line.points = {field.position[i] - reach, field.position[i]};
        line.colours = {colour, colour};
        line.alphas = {0.0f, alpha};
        line.width_px = look.width_px;
        out.push_back(std::move(line));
    }
    return out;
}

// THE SNAPSHOT FILE: the field at a frame, so a still is redrawn from it and the kept frame in seconds, not from an 11-minute
// solve. Little-endian raw: the magic "SPPART1", u32 count, u32 frames, five u64 counters (emitted, dropped, left the region, no
// data, aged), then the positions, the previous positions (3 floats each) and the ages. Exactly that size: a short or long file is
// refused, never half-read.
inline constexpr std::array<char, 8> kParticleMagic{'S', 'P', 'P', 'A', 'R', 'T', '1', '\0'};

[[nodiscard]] inline spade::Result<void> write_particle_field(const std::filesystem::path& path, const ParticleField& field) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return std::unexpected(spade::Error{spade::Code::io_error, "particle snapshot: cannot write " + path.string()});
    }
    const auto put = [&out](const void* data, std::size_t bytes) { out.write(static_cast<const char*>(data), static_cast<std::streamsize>(bytes)); };
    const auto count = static_cast<uint32_t>(field.position.size());
    put(kParticleMagic.data(), kParticleMagic.size());
    put(&count, sizeof count);
    put(&field.frames, sizeof field.frames);
    for (const uint64_t c : {field.emitted, field.dropped, field.retired_left_region, field.retired_no_data, field.retired_aged}) {
        put(&c, sizeof c);
    }
    put(field.position.data(), field.position.size() * sizeof(glm::vec3));
    put(field.previous.data(), field.previous.size() * sizeof(glm::vec3));
    put(field.age.data(), field.age.size() * sizeof(float));
    out.flush();
    if (!out) return std::unexpected(spade::Error{spade::Code::io_error, "particle snapshot: a write to " + path.string() + " failed"});
    return {};
}

[[nodiscard]] inline spade::Result<ParticleField> read_particle_field(const std::filesystem::path& path) {
    const auto bad = [&path](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::io_error, "particle snapshot " + path.string() + ": " + why});
    };
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) return bad("cannot be opened");
    const auto size = static_cast<std::uint64_t>(in.tellg());
    in.seekg(0);
    constexpr std::uint64_t kHeader = 8u + 4u + 4u + 5u * 8u;
    if (size < kHeader) return bad("is truncated (a header is " + std::to_string(kHeader) + " bytes)");
    std::array<char, 8> magic{};
    in.read(magic.data(), magic.size());
    if (magic != kParticleMagic) return bad("is not a particle snapshot (the magic is wrong)");
    uint32_t count = 0;
    ParticleField field;
    in.read(reinterpret_cast<char*>(&count), sizeof count);
    in.read(reinterpret_cast<char*>(&field.frames), sizeof field.frames);
    uint64_t* counters[5] = {&field.emitted, &field.dropped, &field.retired_left_region, &field.retired_no_data, &field.retired_aged};
    for (uint64_t* c : counters) in.read(reinterpret_cast<char*>(c), sizeof *c);
    if (count > (1u << 24)) return bad("holds more than 2^24 points");
    const std::uint64_t expect = kHeader + std::uint64_t{count} * (2u * sizeof(glm::vec3) + sizeof(float));
    if (size != expect) return bad("is truncated or has trailing bytes: " + std::to_string(size) + " bytes, " + std::to_string(expect) + " expected for " + std::to_string(count) + " points");
    field.position.resize(count);
    field.previous.resize(count);
    field.age.resize(count);
    in.read(reinterpret_cast<char*>(field.position.data()), static_cast<std::streamsize>(count * sizeof(glm::vec3)));
    in.read(reinterpret_cast<char*>(field.previous.data()), static_cast<std::streamsize>(count * sizeof(glm::vec3)));
    in.read(reinterpret_cast<char*>(field.age.data()), static_cast<std::streamsize>(count * sizeof(float)));
    if (!in) return bad("could not be read to its end");
    return field;
}

// THE LOOK ON THE COMMAND LINE: "key=value,key=value" over StreakLook's defaults (--particle-look). A redraw from the snapshot takes
// seconds, so one job draws several variants. Keys: tail (s, > 0), min_len (m, 0 up), width (px, 0.5 to 64), glow (px, 0 to 64),
// strength (0 to 1), alpha (0 to 1), fade_in (s, > 0), fast (m/s, > 0), dim (0 to 1), dim_r (m, > 0), substeps (a whole number 1 to 64: how a
// frame is advanced, so it takes effect where the points are carried, not in a redraw from a snapshot). Empty: the defaults. An
// unknown key, no '=', an empty key or value, a value that is not a finite number, or one out of its range: refused, naming it.
[[nodiscard]] inline spade::Result<StreakLook> parse_particle_look(std::string_view text) {
    const auto bad = [](const std::string& why) {
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "particle look: " + why});
    };
    const auto trim = [](std::string_view v) {
        while (!v.empty() && (v.front() == ' ' || v.front() == '\t')) v.remove_prefix(1);
        while (!v.empty() && (v.back() == ' ' || v.back() == '\t')) v.remove_suffix(1);
        return v;
    };
    StreakLook look;
    if (trim(text).empty()) return look;
    std::size_t at = 0;
    while (at <= text.size()) {
        const std::size_t comma = text.find(',', at);
        const std::string_view item = trim(text.substr(at, comma == std::string_view::npos ? std::string_view::npos : comma - at));
        at = comma == std::string_view::npos ? text.size() + 1u : comma + 1u;
        if (item.empty()) return bad("an empty item (a stray comma)");
        const std::size_t eq = item.find('=');
        if (eq == std::string_view::npos) return bad("'" + std::string(item) + "' has no '=': write key=value");
        const std::string key(trim(item.substr(0, eq)));
        const std::string_view value = trim(item.substr(eq + 1u));
        if (key.empty()) return bad("an empty key in '" + std::string(item) + "'");
        if (value.empty()) return bad("'" + key + "' has no value");
        float number = 0.0f;
        const auto [end, ec] = std::from_chars(value.data(), value.data() + value.size(), number);
        if (ec != std::errc{} || end != value.data() + value.size() || !std::isfinite(number)) {
            return bad("'" + key + "' is not a number: '" + std::string(value) + "'");
        }
        struct Row {
            const char* key;
            float* slot;
            float lo, hi;
            bool lo_open;  // the low end itself is refused
        };
        const Row rows[] = {{"tail", &look.tail_s, 0.0f, 1.0f, true},          {"min_len", &look.min_len_m, 0.0f, 1.0f, false},
                            {"width", &look.width_px, 0.5f, 64.0f, false},     {"glow", &look.glow_px, 0.0f, 64.0f, false},
                            {"strength", &look.glow_strength, 0.0f, 1.0f, false}, {"alpha", &look.peak_alpha, 0.0f, 1.0f, false},
                            {"fade_in", &look.fade_in_s, 0.0f, 10.0f, true},   {"fast", &look.fast_mps, 0.0f, 1000.0f, true},
                            {"dim", &look.dim, 0.0f, 1.0f, false},             {"dim_r", &look.dim_r, 0.0f, 10.0f, true}};
        if (key == "substeps") {
            if (number < 1.0f || number > 64.0f || number != std::floor(number)) {
                return bad("'substeps' = " + std::string(value) + " is not a whole number from 1 to 64");
            }
            look.substeps = static_cast<uint32_t>(number);
            continue;
        }
        const Row* row = nullptr;
        for (const Row& r : rows) {
            if (key == r.key) row = &r;
        }
        if (row == nullptr) return bad("unknown key '" + key + "' (tail min_len width glow strength alpha fade_in fast dim dim_r substeps)");
        if (number < row->lo || number > row->hi || (row->lo_open && number <= row->lo)) {
            return bad("'" + key + "' = " + std::string(value) + " is out of its range (" + (row->lo_open ? "above " : "") +
                       std::to_string(row->lo) + " to " + std::to_string(row->hi) + ")");
        }
        *row->slot = number;
    }
    return look;
}

}  // namespace flow_movie
