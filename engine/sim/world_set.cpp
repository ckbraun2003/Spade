#include "sim/world_set.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <utility>

#include "core/rng.hpp"
#include "sensors/rings.hpp"

namespace spade {
namespace {

[[nodiscard]] Error invalid(std::string context) {
    return Error{Code::invalid_argument, std::move(context)};
}

[[nodiscard]] bool finite(float v) noexcept { return std::isfinite(v); }

[[nodiscard]] bool finite(const glm::vec3& v) noexcept {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
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

}  // namespace spade
