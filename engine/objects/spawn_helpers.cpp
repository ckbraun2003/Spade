// spawn_helpers.cpp -- see spawn_helpers.hpp for the two rules: seeded never
// ambient, and all-or-nothing.

#include "objects/spawn_helpers.hpp"

#include <string>

#include "core/rng.hpp"

namespace spade::objects {
namespace {

// The domain tag. Two systems drawing from the same world seed must not draw
// the same numbers, and a literal tag here is what separates them -- see
// core/rng.hpp's derivation note. Changing this string changes every placement
// it produces, so it is as much a pinned constant as the splitmix constants
// themselves.
constexpr std::string_view kPositionDomain = "spawn_helpers";

// A SEPARATE STREAM FOR VELOCITY, not a second draw from the position stream.
// Sharing one stream would make every body after the first land somewhere else
// the moment a caller asked for velocities -- adding motion would silently
// re-lay-out the scene. Domain separation is exactly what core/rng.hpp exists
// to provide, so the two are independent by construction rather than by
// careful interleaving.
constexpr std::string_view kVelocityDomain = "spawn_helpers.velocity";

// Reserves nothing and spawns nothing; just answers whether it would fit.
[[nodiscard]] Result<void> check_capacity(const Simulation& sim, uint32_t world_index,
                                          uint32_t count) {
    const Result<uint32_t> capacity = sim.body_capacity(world_index);
    if (!capacity) return std::unexpected(capacity.error());
    const Result<uint32_t> live = sim.live_body_count(world_index);
    if (!live) return std::unexpected(live.error());

    // Counted against the LIVE population. Bodies already queued and not yet
    // flushed are not visible here, so a caller that queues spawns by hand and
    // then calls a helper can still be refused by spawn() itself -- which the
    // rollback below handles rather than leaving half-done.
    const uint32_t remaining = *capacity > *live ? *capacity - *live : 0u;
    if (count > remaining) {
        return std::unexpected(Error{
            Code::capacity_exceeded,
            "spawn of " + std::to_string(count) + " bodies exceeds world " +
                std::to_string(world_index) + "'s remaining capacity (" + std::to_string(remaining) +
                " of " + std::to_string(*capacity) + "); no bodies were spawned"});
    }
    return {};
}

// Uniform inside a ball of `radius`. ONE implementation, used for both a
// body's placement and its initial velocity -- they were the same rejection
// loop written twice, which is two places for a distribution bug to differ.
//
// REJECTION SAMPLING in the unit cube, rather than a cube root: uniform inside
// the solid ball, and the draw sequence stays a plain run of next_float()s, so
// a placement is reproducible by re-running the stream rather than by
// reproducing a transcendental. It terminates with probability 1 and, in
// practice, fast -- the ball is pi/6 ~= 52% of the cube, so under two draws per
// sample on average.
//
// A non-positive radius returns the zero vector WITHOUT drawing. That is what
// makes `velocity_radius_mps = 0` cost no stream draws, which in turn is why
// asking for velocities cannot shift the position sequence. (Callers that
// place positions validate radius > 0 before reaching here, so for them the
// early return is unreachable rather than a silent degenerate case.)
[[nodiscard]] glm::vec3 sample_in_ball(rng::Stream& stream, float radius) {
    if (!(radius > 0.0f)) return glm::vec3(0.0f);
    for (;;) {
        const float x = stream.next_float() * 2.0f - 1.0f;
        const float y = stream.next_float() * 2.0f - 1.0f;
        const float z = stream.next_float() * 2.0f - 1.0f;
        if (x * x + y * y + z * z <= 1.0f) return radius * glm::vec3(x, y, z);
    }
}

// Places `count` bodies at positions from `next_position`, undoing its own work
// if any single spawn fails.
template <class PositionFn>
[[nodiscard]] Result<std::vector<BodyRef>> spawn_each(Simulation& sim, uint32_t world_index,
                                                      uint32_t count, float mass,
                                                      rng::Stream& velocity_stream,
                                                      float velocity_radius,
                                                      PositionFn next_position) {
    std::vector<BodyRef> refs;
    refs.reserve(count);
    for (uint32_t i = 0; i < count; ++i) {
        BodySpawn body;
        body.pos = next_position();
        // From its OWN stream, so a scene's placement is identical whether or
        // not velocities were requested.
        body.vel = sample_in_ball(velocity_stream, velocity_radius);
        body.mass = mass;

        Result<BodyRef> ref = sim.spawn(world_index, body);
        if (!ref) {
            // ALL OR NOTHING. Undo this call's own bodies so the caller is left
            // exactly where it started rather than with a partial scene.
            for (const BodyRef& placed : refs) {
                (void)sim.despawn(placed);  // best effort; the original error is what matters
            }
            return std::unexpected(ref.error());
        }
        refs.push_back(*ref);
    }
    return refs;
}

}  // namespace

Result<std::vector<BodyRef>> spawn_in_sphere(Simulation& sim, uint32_t world_index,
                                             const SphereSpawn& params) {
    if (!(params.radius > 0.0f)) {
        return std::unexpected(Error{Code::invalid_argument, "sphere spawn radius must be > 0"});
    }
    if (Result<void> fits = check_capacity(sim, world_index, params.count); !fits) {
        return std::unexpected(fits.error());
    }

    rng::Stream stream = rng::make_stream(params.seed, kPositionDomain, world_index);
    rng::Stream velocity = rng::make_stream(params.seed, kVelocityDomain, world_index);
    return spawn_each(sim, world_index, params.count, params.mass, velocity,
                      params.velocity_radius_mps,
                      [&] { return params.center + sample_in_ball(stream, params.radius); });
}

Result<std::vector<BodyRef>> spawn_in_cube(Simulation& sim, uint32_t world_index,
                                           const CubeSpawn& params) {
    if (!(params.half_extent > 0.0f)) {
        return std::unexpected(Error{Code::invalid_argument, "cube spawn half_extent must be > 0"});
    }
    if (Result<void> fits = check_capacity(sim, world_index, params.count); !fits) {
        return std::unexpected(fits.error());
    }

    rng::Stream stream = rng::make_stream(params.seed, kPositionDomain, world_index);
    rng::Stream velocity = rng::make_stream(params.seed, kVelocityDomain, world_index);
    return spawn_each(sim, world_index, params.count, params.mass, velocity,
                      params.velocity_radius_mps, [&] {
        const float x = stream.next_float() * 2.0f - 1.0f;
        const float y = stream.next_float() * 2.0f - 1.0f;
        const float z = stream.next_float() * 2.0f - 1.0f;
        return params.center + params.half_extent * glm::vec3(x, y, z);
    });
}

}  // namespace spade::objects
