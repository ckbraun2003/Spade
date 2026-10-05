// The built-in scenes' physics records: what assets/scenes/ cannot hold yet.
//
// ⚠ A BRIDGE, DELETED WHEN WORLD FILE V3 CARRIES THESE RECORDS (module-API
// design §11; docs/design/interface/07-status.md files the deletion against
// v3). World file v2 holds a world's geometry and environment but not the
// per-world physics records -- the run's seed, the turbulence, the contact
// and broad-phase parameters -- which instantiate() takes as a
// WorldInstanceDesc. The viewer sets them in code, and the sandbox may not
// link the viewer (SL2b), so they are copied here, once, for each scene file
// in assets/scenes/. tests/test_sandbox_builtin_records.cpp pins every entry
// to the record spade_viewer's own make_scene() builds, field by field, so
// the copy cannot drift. (The lead's ruling, 2026-10-05.)
//
// Display-free (SL15b).

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

#include "physics/contacts.hpp"
#include "physics/grid.hpp"
#include "sim/world_set.hpp"
#include "world/medium.hpp"

namespace spade::sandbox {

// The step the scene files assume: the viewer's (pinned by the test).
inline constexpr uint64_t kBuiltinStepNs = 4'000'000;
inline constexpr uint32_t kBuiltinSubsteps = 4;

// The contact and grid records are alignas(16) std430 rows, so this holder
// gains padding; it is a description, never uploaded or hashed, as
// sim/world_set.hpp says of WorldInstanceDesc, which disables C4324 the same way.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
struct BuiltinRecords {
    uint64_t seed = 0;
    DrydenParams turbulence{};
    physics::ContactParams contacts{};
    physics::GridParams grid{};
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

namespace detail {
[[nodiscard]] inline BuiltinRecords records(uint64_t seed, float e, float mu, float r, float cell) {
    BuiltinRecords out;
    out.seed = seed;
    out.contacts.restitution_e = e;
    out.contacts.friction_mu = mu;
    out.contacts.proxy_radius = r;
    out.grid.cell_size = cell;
    return out;
}
}  // namespace detail

// The records for a scene file's stem ("drop", "bounce_lane_2", ...), or none
// for a scene this table does not know.
[[nodiscard]] inline std::optional<BuiltinRecords> builtin_records(std::string_view stem) {
    using detail::records;
    if (stem == "drop") return records(1, 0.35f, 0.5f, 0.3f, 0.8f);
    if (stem == "shower") return records(7, 0.15f, 0.6f, 0.12f, 0.3f);
    if (stem == "gate") return records(3, 0.4f, 0.3f, 0.25f, 0.6f);
    // bounce: one world per lane, the restitution rising lane by lane.
    constexpr std::array<std::string_view, 4> kBounce = {"bounce_lane_0", "bounce_lane_1", "bounce_lane_2",
                                                         "bounce_lane_3"};
    constexpr std::array<float, 4> kBounceE = {0.0f, 0.25f, 0.5f, 0.75f};
    for (uint64_t lane = 0; lane < kBounce.size(); ++lane) {
        if (stem == kBounce[lane]) return records(lane + 1, kBounceE[lane], 0.3f, 0.35f, 1.0f);
    }
    // The quadrotor scenes set the turbulence explicitly.
    if (stem == "hover" || stem == "wind" || stem == "flight") {
        BuiltinRecords r = stem == "flight" ? records(23, 0.3f, 0.5f, 0.2f, 0.6f) : records(11, 0.2f, 0.5f, 0.2f, 0.5f);
        r.turbulence = dryden_params(TurbulenceLevel::none);
        if (stem == "wind") {
            r.seed = 23;
            r.turbulence = dryden_params(TurbulenceLevel::moderate);
        }
        return r;
    }
    constexpr std::array<std::string_view, 4> kSwarm = {"swarm_lane_0", "swarm_lane_1", "swarm_lane_2",
                                                        "swarm_lane_3"};
    for (uint64_t lane = 0; lane < kSwarm.size(); ++lane) {
        if (stem == kSwarm[lane]) {
            BuiltinRecords r = records(lane + 101, 0.2f, 0.5f, 0.2f, 0.5f);
            r.turbulence = dryden_params(TurbulenceLevel::none);
            return r;
        }
    }
    return std::nullopt;
}

// An instance carrying `r`; instantiate() fills in the composed world.
[[nodiscard]] inline WorldInstanceDesc instance_of(const BuiltinRecords& r) {
    WorldInstanceDesc out;
    out.seed = r.seed;
    out.turbulence = r.turbulence;
    out.contacts = r.contacts;
    out.grid = r.grid;
    return out;
}

}  // namespace spade::sandbox
