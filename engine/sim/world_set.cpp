#include "sim/world_set.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <limits>
#include <span>
#include <string>
#include <type_traits>
#include <utility>

#include "core/rng.hpp"
#include "core/validate.hpp"
#include "sensors/rings.hpp"
#include "state/snapshot.hpp"

namespace spade {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

// "v is in [lo, hi]", spelled so a NaN FAILS rather than comparing false on
// both sides and slipping through -- the same discipline physics/grid.hpp's
// grid_cell_of() uses for its range test.
[[nodiscard]] bool in_range(float v, float lo, float hi) noexcept { return v >= lo && v <= hi; }

[[nodiscard]] std::string world_prefix(std::size_t index) {
    return "world " + std::to_string(index) + ": ";
}

[[nodiscard]] Result<void> validate_contacts(const physics::ContactParams& c, std::size_t index) {
    const std::string at = world_prefix(index);
    if (!in_range(c.restitution_e, 0.0f, 1.0f)) {
        return std::unexpected(invalid(at + "contacts.restitution_e must be in [0, 1]"));
    }
    if (!(c.friction_mu >= 0.0f) || !finite(c.friction_mu)) {
        return std::unexpected(invalid(at + "contacts.friction_mu must be finite and >= 0"));
    }
    if (!in_range(c.baumgarte_beta, 0.0f, 1.0f)) {
        return std::unexpected(invalid(at + "contacts.baumgarte_beta must be in [0, 1]"));
    }
    if (!(c.slop >= 0.0f) || !finite(c.slop)) {
        return std::unexpected(invalid(at + "contacts.slop must be finite and >= 0"));
    }
    if (!(c.proxy_radius >= 0.0f) || !finite(c.proxy_radius)) {
        return std::unexpected(invalid(at + "contacts.proxy_radius must be finite and >= 0"));
    }
    if (c._r0 != 0.0f || c._r1 != 0.0f || c._r2 != 0.0f) {
        return std::unexpected(invalid(at + "contacts reserved lanes must be 0"));
    }
    return {};
}

[[nodiscard]] Result<void> validate_grid(const physics::GridParams& g, std::size_t index) {
    const std::string at = world_prefix(index);
    if (!(g.cell_size > 0.0f) || !finite(g.cell_size)) {
        return std::unexpected(invalid(at + "grid.cell_size must be finite and > 0"));
    }
    if (g._r0 != 0.0f || g._r1 != 0.0f || g._r2 != 0.0f) {
        return std::unexpected(invalid(at + "grid reserved lanes must be 0"));
    }
    return {};
}

// The footgun physics/grid.hpp documents and does not check: the resolve step
// only gathers the 27 cells around a body, so a contact DIAMETER larger than
// one cell silently loses pairs that straddle a gap. This is the layer that can
// afford to check it, so it does.
[[nodiscard]] Result<void> validate_grid_against_contacts(const physics::GridParams& g,
                                                          const physics::ContactParams& c,
                                                          std::size_t index) {
    if (g.cell_size < 2.0f * c.proxy_radius) {
        return std::unexpected(invalid(world_prefix(index) +
                                       "grid.cell_size must be >= 2 * contacts.proxy_radius "
                                       "(smaller silently misses contacts across cell boundaries)"));
    }
    return {};
}

[[nodiscard]] Result<void> validate_turbulence(const DrydenParams& d, std::size_t index) {
    const std::string at = world_prefix(index);
    if (!(d.scale_u > 0.0f) || !(d.scale_v > 0.0f) || !(d.scale_w > 0.0f) || !finite(d.scale_u) ||
        !finite(d.scale_v) || !finite(d.scale_w)) {
        return std::unexpected(invalid(at + "turbulence scale lengths must be finite and > 0"));
    }
    if (!(d.sigma_u >= 0.0f) || !(d.sigma_v >= 0.0f) || !(d.sigma_w >= 0.0f) || !finite(d.sigma_u) ||
        !finite(d.sigma_v) || !finite(d.sigma_w)) {
        return std::unexpected(invalid(at + "turbulence sigmas must be finite and >= 0"));
    }
    if (!(d.reference_airspeed > 0.0f) || !finite(d.reference_airspeed)) {
        return std::unexpected(invalid(at + "turbulence reference_airspeed must be finite and > 0"));
    }
    if (d._reserved0 != 0.0f) {
        return std::unexpected(invalid(at + "turbulence reserved lane must be 0"));
    }
    return {};
}

// Byte-wise equality. Both records static_assert that every byte belongs to a
// named field (no implicit padding), which is exactly what makes memcmp the
// right comparison rather than a field-by-field one that could fall behind a
// future reserved lane.
template <class T>
[[nodiscard]] bool same_bytes(const T& a, const T& b) noexcept {
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

// ---------------------------------------------------------------------------
// config_hash's fold primitives.
//
// THE MIXING FUNCTION IS state/snapshot.hpp's fnv1a64(), reused rather than
// re-derived -- the same primitive the snapshot schema hash and the replay
// digest (engine/testing/replay.hpp) fold with, seeded so a fold chains. There
// is deliberately no third FNV implementation in this tree; core/rng.hpp's
// fnv1a64(string_view) is the same construction over the same constants but
// takes no seed, so it cannot chain and is not what this needs.
// ---------------------------------------------------------------------------

// Folds an object's byte representation. Only ever instantiated for scalars and
// for the parameter records whose "no implicit padding" asserts appear below.
template <class T>
[[nodiscard]] uint64_t fold_value(uint64_t seed, const T& value) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "config_hash folds byte representations");
    std::byte bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    return fnv1a64(std::span<const std::byte>(bytes, sizeof(T)), seed);
}

[[nodiscard]] uint64_t fold_bytes(uint64_t seed, std::span<const std::byte> bytes) noexcept {
    return fnv1a64(bytes, seed);
}

// A length-prefixed run of trivially-copyable elements: the count first, then
// the raw bytes. The prefix is what makes the boundary between one run and
// whatever follows it unambiguous (see the fold-order contract in the header).
template <class T>
[[nodiscard]] uint64_t fold_run(uint64_t seed, const std::vector<T>& items) noexcept {
    static_assert(std::is_trivially_copyable_v<T>, "config_hash folds byte representations");
    seed = fold_value(seed, static_cast<uint64_t>(items.size()));
    if (items.empty()) return seed;
    return fold_bytes(seed, std::as_bytes(std::span<const T>(items.data(), items.size())));
}

// ---------------------------------------------------------------------------
// THE PRECONDITION OF EVERY RAW-BYTE FOLD ABOVE, re-asserted at the point of
// use rather than trusted from a distance: each of these five records has NO
// IMPLICIT PADDING -- every byte of it belongs to a named field, so its byte
// image is a function of its field values alone.
//
// Each type carries this same assert in its own header (physics/contacts.hpp,
// physics/grid.hpp, world/medium.hpp) or is asserted here for the first time
// (world/sdf.hpp pins SdfNode's and SdfTransform's SIZE but has never pinned
// the named-byte sum). Restating them here means that a future field added to
// any of the five breaks THIS file's build, at the fold that depends on the
// property, instead of silently folding indeterminate bytes into a hash whose
// whole job is to be reproducible.
//
// SdfNode::_pad and SdfTransform::_pad are EXPLICIT lanes with zero
// initializers, not compiler padding -- every construction site in the tree
// value-initializes the aggregate (world/builder.cpp's `SdfNode node{}` /
// `SdfTransform t{}`), so those bytes are deterministically zero and folding
// them is a fold of a known constant, not of garbage.
// ---------------------------------------------------------------------------
static_assert(sizeof(SdfNode::kind) + sizeof(SdfNode::op) + sizeof(SdfNode::transform) +
                      sizeof(SdfNode::_pad) + sizeof(SdfNode::params) ==
                  sizeof(SdfNode),
              "SdfNode has implicit padding: config_hash cannot fold its bytes");
static_assert(sizeof(SdfTransform::world_to_local) + sizeof(SdfTransform::scale) +
                      sizeof(SdfTransform::_pad) ==
                  sizeof(SdfTransform),
              "SdfTransform has implicit padding: config_hash cannot fold its bytes");
static_assert(std::is_trivially_copyable_v<SdfNode> && std::is_trivially_copyable_v<SdfTransform>,
              "config_hash folds SDF program bytes");
static_assert(8 * sizeof(float) == sizeof(DrydenParams),
              "DrydenParams has implicit padding: config_hash cannot fold its bytes");
static_assert(sizeof(physics::ContactParams::restitution_e) + sizeof(physics::ContactParams::friction_mu) +
                      sizeof(physics::ContactParams::baumgarte_beta) + sizeof(physics::ContactParams::slop) +
                      sizeof(physics::ContactParams::proxy_radius) + sizeof(physics::ContactParams::_r0) +
                      sizeof(physics::ContactParams::_r1) + sizeof(physics::ContactParams::_r2) ==
                  sizeof(physics::ContactParams),
              "ContactParams has implicit padding: config_hash cannot fold its bytes");
static_assert(sizeof(physics::GridParams::cell_size) + sizeof(physics::GridParams::_r0) +
                      sizeof(physics::GridParams::_r1) + sizeof(physics::GridParams::_r2) ==
                  sizeof(physics::GridParams),
              "GridParams has implicit padding: config_hash cannot fold its bytes");

}  // namespace

WorldSetDesc replicate(const WorldInstanceDesc& prototype, uint32_t count, uint64_t scene_seed) {
    WorldSetDesc set;
    set.worlds.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        WorldInstanceDesc instance = prototype;
        instance.seed = rng::splitmix64(scene_seed ^ rng::fnv1a64(kWorldSeedDomainTag) ^ uint64_t{i});
        set.worlds.push_back(std::move(instance));
    }
    return set;
}

Result<WorldSetDesc> world_set_from(const WorldRef& ref, uint32_t count, uint64_t scene_seed,
                                    const WorldInstanceDesc& instance_prototype) {
    if (count == 0) {
        return std::unexpected(invalid("world_set_from: count must be > 0"));
    }
    const Result<WorldDesc> world = resolve_world(ref);
    if (!world) {
        return std::unexpected(world.error());
    }
    WorldInstanceDesc prototype = instance_prototype;
    prototype.world = *world;
    return replicate(prototype, count, scene_seed);
}

Result<WorldSetLayout> validate_world_set(const WorldSetDesc& desc) {
    if (desc.worlds.empty()) {
        return std::unexpected(invalid("world set is empty"));
    }
    if (desc.worlds.size() > std::numeric_limits<uint32_t>::max()) {
        return std::unexpected(Error{Code::capacity_exceeded, "world count does not fit a uint32"});
    }

    WorldSetLayout layout;
    layout.world_count = static_cast<uint32_t>(desc.worlds.size());

    for (std::size_t i = 0; i < desc.worlds.size(); ++i) {
        const WorldInstanceDesc& instance = desc.worlds[i];
        const std::string at = world_prefix(i);

        // Re-validate the SDF program. A WorldDesc that came out of
        // WorldBuilder::build() is valid by construction, but a hand-built or
        // (from S5) file-loaded one need not be, and every downstream pass
        // treats validity as a precondition it does not check.
        const Result<uint32_t> depth = instance.world.sdf.validate();
        if (!depth) {
            return std::unexpected(Error{depth.error().code, at + depth.error().context});
        }

        const Capacities& caps = instance.world.capacities;
        if (caps.bodies == 0 || caps.force_elements == 0 || caps.sensors == 0 || caps.contacts == 0) {
            return std::unexpected(invalid(at + "every world capacity must be > 0"));
        }

        const Environment& env = instance.world.environment;
        if (!finite(env.gravity) || !finite(env.wind) || !finite(env.air_density) ||
            !finite(env.temperature_k)) {
            return std::unexpected(invalid(at + "environment has non-finite values"));
        }
        if (!(env.air_density >= 0.0f)) {
            return std::unexpected(invalid(at + "environment.air_density must be >= 0"));
        }

        if (Result<void> r = validate_contacts(instance.contacts, i); !r) {
            return std::unexpected(r.error());
        }
        if (Result<void> r = validate_grid(instance.grid, i); !r) {
            return std::unexpected(r.error());
        }
        if (Result<void> r = validate_grid_against_contacts(instance.grid, instance.contacts, i); !r) {
            return std::unexpected(r.error());
        }
        if (Result<void> r = validate_turbulence(instance.turbulence, i); !r) {
            return std::unexpected(r.error());
        }

        layout.body_capacity = std::max(layout.body_capacity, caps.bodies);
        layout.element_capacity = std::max(layout.element_capacity, caps.force_elements);
        layout.sensor_capacity = std::max(layout.sensor_capacity, caps.sensors);
    }

    // ArenaSet allocates world_count * capacity_per_world elements per array and
    // addresses them with a uint32 global slot index, so the product must fit.
    // Checked here rather than being left to register_array()'s own
    // capacity_exceeded so the message names the world set rather than an array.
    const uint64_t bodies_total = uint64_t{layout.world_count} * uint64_t{layout.body_capacity};
    // Covers BOTH force-element arrays: the drag table and the rotor table are
    // each registered at element_capacity per world (Task 18's shared
    // force-element budget -- see Simulation::create), so one bound serves
    // both. If a third element kind ever gets a different capacity, it needs
    // its own line here.
    const uint64_t elements_total = uint64_t{layout.world_count} * uint64_t{layout.element_capacity};
    const uint64_t sensors_total = uint64_t{layout.world_count} * uint64_t{layout.sensor_capacity};
    // The ring array is the biggest of them all: kRingDepth samples per sensor
    // (sensors/rings.hpp), so it hits the uint32 slot ceiling 64x sooner than
    // the sensor table itself does. Checked here rather than left to
    // register_array() so the message names the world set.
    const uint64_t ring_total = sensors_total * uint64_t{sensors::kRingDepth};
    const uint64_t slot_limit = std::numeric_limits<uint32_t>::max();
    if (bodies_total > slot_limit || elements_total > slot_limit || sensors_total > slot_limit ||
        ring_total > slot_limit) {
        return std::unexpected(
            Error{Code::capacity_exceeded, "world count times per-world capacity does not fit a uint32 slot index"});
    }

    layout.uniform_dynamic_params = true;
    for (std::size_t i = 1; i < desc.worlds.size(); ++i) {
        if (!same_bytes(desc.worlds[i].contacts, desc.worlds[0].contacts) ||
            !same_bytes(desc.worlds[i].grid, desc.worlds[0].grid)) {
            layout.uniform_dynamic_params = false;
            break;
        }
    }

    return layout;
}

// FIELD-SET GUARD (S5 final-review fix wave, I6). config_hash's fold above
// and world_set.hpp's "WHAT IT DELIBERATELY DOES NOT COVER" list are together
// meant to account for EVERY field WorldDesc carries -- but nothing enforced
// that until now, which is exactly how visual_refs (T5) went undocumented in
// both places. sizeof(WorldDesc) changes whenever a field is added, removed,
// or changes type, so pinning it here forces the next such change to touch
// this line -- and, per the message, to answer the actual question: does the
// new field belong in the fold, or in the header's exclusion list, and why.
//
// TWO MEASURED LITERALS, NOT ONE, DISCOVERED THE HARD WAY: a first attempt at
// this guard used a single literal (184) and passed msvc-ninja-release but
// FAILED msvc-ninja-debug with an actual size of 224. The 40-byte gap is
// MSVC's debug C++ runtime (/MDd, this program's msvc-ninja-debug preset):
// it gives every std::string/std::vector an extra _Container_proxy pointer
// for iterator debugging, 8 bytes per container, and WorldDesc holds exactly
// five (name, sdf.nodes, sdf.transforms, spawns, visual_refs) -- 5 * 8 = 40,
// 184 + 40 = 224, which is what msvc-ninja-debug measured. This is the SAME
// compiler and target, not a cross-platform difference, so it is guarded on
// the macro that actually causes it (_ITERATOR_DEBUG_LEVEL, MSVC STL's own
// name for the setting) rather than on _DEBUG or NDEBUG, which would guard
// on the usual correlate instead of the cause.
//
// THE CROSS-PLATFORM QUESTION THIS DOES NOT ANSWER, FLAGGED RATHER THAN
// ASSUMED: this program's CI never builds Debug at all
// (.github/workflows/spade.yml pins -DCMAKE_BUILD_TYPE=Release on both the
// Windows and Linux/gcc-13 jobs), so only the 184-byte, non-debug-iterator
// shape ever needs to agree across the two CI platforms -- and libstdc++'s
// std::string/std::vector are also 32 and 24 bytes on a 64-bit target with
// no debug-iterator inflation of their own unless a build explicitly defines
// _GLIBCXX_DEBUG, which this program's CI does not, so 184 is EXPECTED to
// hold on gcc-13 Release too. That expectation is NOT verified locally --
// this box has no gcc-13 toolchain -- and is flagged in this task's report
// as the one build this literal has not been cross-checked on; if the
// gcc-13 CI job's build ever disagrees, the fix is a third guarded branch
// here, not a loosening of the assert.
#if defined(_MSC_VER) && defined(_ITERATOR_DEBUG_LEVEL) && _ITERATOR_DEBUG_LEVEL != 0
static_assert(sizeof(WorldDesc) == 224,
              "WorldDesc's field set changed (msvc-ninja-debug shape, _ITERATOR_DEBUG_LEVEL != 0) -- "
              "classify the new/changed field into config_hash's fold above, or into "
              "world_set.hpp's documented \"WHAT IT DELIBERATELY DOES NOT COVER\" exclusion list "
              "with its own reason, then update this literal (and the non-debug-iterator one "
              "below) to the new sizeof(WorldDesc)");
#else
static_assert(sizeof(WorldDesc) == 184,
              "WorldDesc's field set changed -- classify the new/changed field into "
              "config_hash's fold above, or into world_set.hpp's documented "
              "\"WHAT IT DELIBERATELY DOES NOT COVER\" exclusion list with its own reason, "
              "then update this literal (and the debug-iterator one above) to the new "
              "sizeof(WorldDesc)");
#endif

// ---------------------------------------------------------------------------
// config_hash -- the fold order pinned in the header, executed.
//
// TAKES A DESC, NOT A VALIDATED LAYOUT, and derives the four capacity maxima
// itself. Two reasons, both deliberate: it can then be called before (or
// without) validate_world_set(), and there is no WorldSetLayout object it could
// fall out of step with. The maxima are strictly redundant -- they are a pure
// function of the per-world capacities folded below -- and are folded anyway,
// as a prefix, because they are the SHAPE the arenas are allocated to and a
// reader comparing two hashes by hand wants the shape stated before the
// contents. Note the fourth: WorldSetLayout carries no contacts maximum
// (contacts are not an arena array in v1), so this is the one capacity the
// layout does not derive and this function does.
//
// NOT noexcept BY ACCIDENT: it allocates nothing, throws nothing, and reads
// only the desc.
// ---------------------------------------------------------------------------
uint64_t config_hash(const WorldSetDesc& desc) noexcept {
    uint64_t seed = kFnv1a64Offset;

    seed = fold_value(seed, static_cast<uint64_t>(desc.worlds.size()));

    uint32_t max_bodies = 0;
    uint32_t max_elements = 0;
    uint32_t max_sensors = 0;
    uint32_t max_contacts = 0;
    for (const WorldInstanceDesc& instance : desc.worlds) {
        const Capacities& caps = instance.world.capacities;
        max_bodies = std::max(max_bodies, caps.bodies);
        max_elements = std::max(max_elements, caps.force_elements);
        max_sensors = std::max(max_sensors, caps.sensors);
        max_contacts = std::max(max_contacts, caps.contacts);
    }
    seed = fold_value(seed, max_bodies);
    seed = fold_value(seed, max_elements);
    seed = fold_value(seed, max_sensors);
    seed = fold_value(seed, max_contacts);

    // WORLDS IN INDEX ORDER. World index is world identity everywhere
    // downstream (see WorldSetDesc), so two sets holding the same worlds in a
    // different order are different sets and must hash differently -- which an
    // ordered fold gives for free and an order-insensitive one (a sum, an xor)
    // would destroy.
    for (const WorldInstanceDesc& instance : desc.worlds) {
        const WorldDesc& world = instance.world;

        seed = fold_value(seed, static_cast<uint64_t>(world.name.size()));
        if (!world.name.empty()) {
            seed = fold_bytes(seed, std::as_bytes(std::span<const char>(world.name.data(), world.name.size())));
        }

        seed = fold_run(seed, world.sdf.nodes);
        seed = fold_run(seed, world.sdf.transforms);

        // FIELD-WISE, in Environment's declaration order. Environment is an
        // AUTHORING struct (world/builder.hpp), not a std430 record: it carries
        // no layout battery, so nothing pins the absence of implicit padding in
        // it the way the five records asserted above pin theirs, and a
        // byte-wise fold would be resting on an unstated assumption. The fold
        // names the fields instead. Same for Capacities below.
        const Environment& env = world.environment;
        seed = fold_value(seed, env.gravity.x);
        seed = fold_value(seed, env.gravity.y);
        seed = fold_value(seed, env.gravity.z);
        seed = fold_value(seed, env.wind.x);
        seed = fold_value(seed, env.wind.y);
        seed = fold_value(seed, env.wind.z);
        seed = fold_value(seed, env.air_density);
        seed = fold_value(seed, env.temperature_k);
        seed = fold_value(seed, env.seed);

        const Capacities& caps = world.capacities;
        seed = fold_value(seed, caps.bodies);
        seed = fold_value(seed, caps.force_elements);
        seed = fold_value(seed, caps.sensors);
        seed = fold_value(seed, caps.contacts);

        // The INSTANCE seed -- the world's actual rng root (WorldParams::seed),
        // not env.seed's authoring default.
        seed = fold_value(seed, instance.seed);

        seed = fold_value(seed, instance.turbulence);
        seed = fold_value(seed, instance.contacts);
        seed = fold_value(seed, instance.grid);
    }

    return seed;
}

}  // namespace spade
