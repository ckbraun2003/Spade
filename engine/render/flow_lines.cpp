#include "render/flow_lines.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "core/fp32_math.hpp"
#include "render/flow_sample.hpp"

namespace spade::render {
namespace {

struct Box {
    glm::vec3 lo{0.0f};
    glm::vec3 hi{0.0f};
    [[nodiscard]] bool contains(glm::vec3 p) const {
        return p.x >= lo.x && p.y >= lo.y && p.z >= lo.z && p.x <= hi.x && p.y <= hi.y && p.z <= hi.z;
    }
};

[[nodiscard]] float length_of(glm::vec3 v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

// The flow's unit direction at p, or why there is none.
struct Direction {
    glm::vec3 d{0.0f};
    std::optional<LineEnd> end;
};

[[nodiscard]] Direction direction_at(const FlowSampler& sampler, glm::vec3 p, float floor) {
    const FlowVelocity v = sample_velocity(sampler, p);
    if (v.no_data) return Direction{glm::vec3(0.0f), LineEnd::no_data};
    const float speed = length_of(v.u);
    if (!(speed > floor)) return Direction{glm::vec3(0.0f), LineEnd::stagnation};
    return Direction{v.u / speed, std::nullopt};
}

// One direction from the seed, in steps of h (negative: backward). Appends
// the points after the seed and their speeds; returns why it ended.
[[nodiscard]] LineEnd trace_one(const FlowSampler& sampler, const Box& box, glm::vec3 seed, float h,
                                const StreamlineOptions& options, std::vector<glm::vec3>& points,
                                std::vector<float>& speeds) {
    glm::vec3 x = seed;
    const float floor = options.stagnation_speed;
    for (uint32_t i = 0; i < options.max_steps; ++i) {
        const Direction k1 = direction_at(sampler, x, floor);
        if (k1.end) return *k1.end;
        const Direction k2 = direction_at(sampler, x + (0.5f * h) * k1.d, floor);
        if (k2.end) return *k2.end;
        const Direction k3 = direction_at(sampler, x + (0.5f * h) * k2.d, floor);
        if (k3.end) return *k3.end;
        const Direction k4 = direction_at(sampler, x + h * k3.d, floor);
        if (k4.end) return *k4.end;
        const glm::vec3 next = x + (h / 6.0f) * (k1.d + 2.0f * k2.d + 2.0f * k3.d + k4.d);
        if (!box.contains(next)) return LineEnd::left_box;
        const FlowVelocity v = sample_velocity(sampler, next);
        if (v.no_data) return LineEnd::no_data;
        points.push_back(next);
        speeds.push_back(length_of(v.u));
        x = next;
    }
    return LineEnd::step_budget;
}

[[nodiscard]] bool finite(glm::vec3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

}  // namespace

Result<std::vector<Streamline>> trace_streamlines(const FlowVolume& volume, std::span<const glm::vec3> seeds,
                                                  const StreamlineOptions& options) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "trace_streamlines: " + why});
    };
    if (Result<void> valid = validate_flow_volume(volume); !valid) {
        return refuse(valid.error().context);
    }
    if (!(options.step_cells > 0.0f && options.step_cells <= 4.0f)) {
        return refuse("the step is not in (0, 4] cells");
    }
    if (options.max_steps == 0u || options.max_steps > (1u << 20)) {
        return refuse("the step budget is not 1 to 2^20");
    }
    if (!(options.stagnation_speed >= 0.0f && std::isfinite(options.stagnation_speed))) {
        return refuse("the stagnation floor is not finite and >= 0");
    }
    if (!options.forward && !options.backward) {
        return refuse("neither direction is asked for");
    }
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        if (!finite(seeds[i])) return refuse("seed " + std::to_string(i) + " is not finite");
    }

    const FlowVolumeHeader& h = volume.header;
    const FlowSampler sampler = make_flow_sampler(volume);
    const Box box{h.origin, h.origin + glm::vec3(static_cast<float>(h.nx), static_cast<float>(h.ny),
                                                 static_cast<float>(h.nz)) *
                                           h.cell};
    const float step = options.step_cells * h.cell;
    std::vector<Streamline> lines;
    lines.reserve(seeds.size());
    for (const glm::vec3& seed : seeds) {
        Streamline line;
        std::vector<glm::vec3> back_points, fore_points;
        std::vector<float> back_speeds, fore_speeds;
        float seed_speed = 0.0f;  // outside the box, or on no-data: 0, and both ends say why
        if (!box.contains(seed)) {
            line.backward_end = line.forward_end = LineEnd::left_box;
        } else {
            const FlowVelocity v = sample_velocity(sampler, seed);
            seed_speed = v.no_data ? 0.0f : length_of(v.u);
            line.backward_end = options.backward
                                    ? trace_one(sampler, box, seed, -step, options, back_points, back_speeds)
                                    : LineEnd::step_budget;
            line.forward_end = options.forward ? trace_one(sampler, box, seed, step, options, fore_points, fore_speeds)
                                               : LineEnd::step_budget;
        }
        line.points.assign(back_points.rbegin(), back_points.rend());
        line.speeds.assign(back_speeds.rbegin(), back_speeds.rend());
        line.seed_point = line.points.size();
        line.points.push_back(seed);
        line.speeds.push_back(seed_speed);
        line.points.insert(line.points.end(), fore_points.begin(), fore_points.end());
        line.speeds.insert(line.speeds.end(), fore_speeds.begin(), fore_speeds.end());
        lines.push_back(std::move(line));
    }
    return lines;
}

namespace {

// One RK4 step in time from x over dt: where the tracer lands, or why it
// retires (no-data at any stage, or landing outside the box).
struct Carried {
    glm::vec3 x{0.0f};
    std::optional<LineEnd> end;
};

[[nodiscard]] Carried carry(const FlowSampler& sampler, const Box& box, glm::vec3 x, float dt) {
    const FlowVelocity k1 = sample_velocity(sampler, x);
    if (k1.no_data) return Carried{x, LineEnd::no_data};
    const FlowVelocity k2 = sample_velocity(sampler, x + (0.5f * dt) * k1.u);
    if (k2.no_data) return Carried{x, LineEnd::no_data};
    const FlowVelocity k3 = sample_velocity(sampler, x + (0.5f * dt) * k2.u);
    if (k3.no_data) return Carried{x, LineEnd::no_data};
    const FlowVelocity k4 = sample_velocity(sampler, x + dt * k3.u);
    if (k4.no_data) return Carried{x, LineEnd::no_data};
    const glm::vec3 next = x + (dt / 6.0f) * (k1.u + 2.0f * k2.u + 2.0f * k3.u + k4.u);
    if (!box.contains(next)) return Carried{next, LineEnd::left_box};
    return Carried{next, std::nullopt};
}

struct Emission {
    double time = 0.0;
    uint32_t source = 0;
};

}  // namespace

Result<void> advance_tracers(TracerState& state, const FlowVolume& volume, std::span<const TracerSource> sources,
                             float dt, const TracerOptions& options) {
    const auto refuse = [](const std::string& why) {
        return std::unexpected(Error{Code::invalid_argument, "advance_tracers: " + why});
    };
    if (Result<void> valid = validate_flow_volume(volume); !valid) {
        return refuse(valid.error().context);
    }
    if (!(std::isfinite(dt) && dt > 0.0f)) {
        return refuse("dt is not finite and > 0");
    }
    if (options.capacity == 0u || options.capacity > (1u << 24)) {
        return refuse("the capacity is not 1 to 2^24");
    }
    if (!(std::isfinite(options.max_age_s) && options.max_age_s > 0.0f)) {
        return refuse("max_age_s is not finite and > 0");
    }
    for (std::size_t i = 0; i < sources.size(); ++i) {
        const TracerSource& src = sources[i];
        const std::string name = "source " + std::to_string(i);
        if (!finite(src.position)) return refuse(name + "'s position is not finite");
        if (!(std::isfinite(src.interval_s) && src.interval_s > 0.0f)) {
            return refuse(name + "'s interval is not finite and > 0");
        }
        if (static_cast<double>(dt) / static_cast<double>(src.interval_s) > static_cast<double>(1u << 20)) {
            return refuse(name + " would emit more than 2^20 tracers in one step");
        }
    }
    const std::size_t live = state.positions.size();
    if (state.ages.size() != live || state.sources.size() != live) {
        return refuse("the state's arrays disagree in length");
    }
    if (live > options.capacity) {
        return refuse("the state holds " + std::to_string(live) + " tracers, past its capacity of " +
                      std::to_string(options.capacity));
    }

    const FlowVolumeHeader& h = volume.header;
    const FlowSampler sampler = make_flow_sampler(volume);
    const Box box{h.origin, h.origin + glm::vec3(static_cast<float>(h.nx), static_cast<float>(h.ny),
                                                 static_cast<float>(h.nz)) *
                                           h.cell};

    // Live tracers first, in order; retired ones leave, counted by reason.
    std::size_t keep = 0;
    for (std::size_t i = 0; i < live; ++i) {
        const Carried c = carry(sampler, box, state.positions[i], dt);
        const float age = state.ages[i] + dt;
        if (c.end == LineEnd::no_data) {
            ++state.retired_no_data;
        } else if (c.end == LineEnd::left_box) {
            ++state.retired_left_box;
        } else if (age > options.max_age_s) {
            ++state.retired_aged;
        } else {
            state.positions[keep] = c.x;
            state.ages[keep] = age;
            state.sources[keep] = state.sources[i];
            ++keep;
        }
    }
    state.positions.resize(keep);
    state.ages.resize(keep);
    state.sources.resize(keep);

    // Then this step's emissions, in time order and by source within a
    // time, so cutting a run into other steps lists them the same way.
    const double start = state.clock_s;
    const double end = start + static_cast<double>(dt);
    std::vector<Emission> emissions;
    for (uint32_t s = 0; s < sources.size(); ++s) {
        const double interval = static_cast<double>(sources[s].interval_s);
        for (double k = std::ceil(start / interval); k * interval < end; k += 1.0) {
            emissions.push_back(Emission{k * interval, s});
        }
    }
    std::stable_sort(emissions.begin(), emissions.end(),
                     [](const Emission& a, const Emission& b) { return a.time < b.time; });
    for (const Emission& e : emissions) {
        if (state.positions.size() >= options.capacity) {
            ++state.dropped;
            continue;
        }
        const glm::vec3 from = sources[e.source].position;
        const auto remaining = static_cast<float>(end - e.time);
        if (!box.contains(from)) {
            ++state.retired_left_box;
            continue;
        }
        const Carried c = carry(sampler, box, from, remaining);
        if (c.end == LineEnd::no_data) {
            ++state.retired_no_data;
        } else if (c.end == LineEnd::left_box) {
            ++state.retired_left_box;
        } else if (remaining > options.max_age_s) {
            ++state.retired_aged;
        } else {
            state.positions.push_back(c.x);
            state.ages.push_back(remaining);
            state.sources.push_back(e.source);
        }
    }
    state.clock_s = end;
    return {};
}

std::vector<glm::vec3> seed_rake(glm::vec3 a, glm::vec3 b, uint32_t n) {
    std::vector<glm::vec3> out;
    if (n == 1u) {
        out.push_back(a + (b - a) * 0.5f);
        return out;
    }
    for (uint32_t i = 0; i < n; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(n - 1u);
        out.push_back(a + (b - a) * t);
    }
    return out;
}

std::vector<glm::vec3> seed_ring(glm::vec3 centre, glm::vec3 axis, float radius, uint32_t n) {
    // Two unit vectors spanning the ring's plane.
    const glm::vec3 helper = std::fabs(axis.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::vec3 c(axis.y * helper.z - axis.z * helper.y, axis.z * helper.x - axis.x * helper.z,
                      axis.x * helper.y - axis.y * helper.x);
    const glm::vec3 e1 = c / length_of(c);
    const glm::vec3 e2(axis.y * e1.z - axis.z * e1.y, axis.z * e1.x - axis.x * e1.z, axis.x * e1.y - axis.y * e1.x);
    std::vector<glm::vec3> out;
    constexpr float kTwoPi = 6.28318531f;
    for (uint32_t i = 0; i < n; ++i) {
        const float theta = kTwoPi * static_cast<float>(i) / static_cast<float>(n);
        out.push_back(centre + radius * (math::cos32(theta) * e1 + math::sin32(theta) * e2));
    }
    return out;
}

// RED: stubs.
Result<void> validate_flow_polyline(const FlowPolyline&) { return {}; }

Result<void> validate_flow_points(const FlowPoints&) { return {}; }

FlowPolyline colour_streamline(const Streamline&, const PreparedTransfer&) { return {}; }

FlowPoints tracer_points(const TracerState&, glm::vec3, uint32_t) { return {}; }

}  // namespace spade::render
