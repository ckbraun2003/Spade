#include "sensors/gnss.hpp"

#include <glm/geometric.hpp>

#include "physics/integrator.hpp"  // body_flags::active
#include "sensors/rings.hpp"

namespace spade::sensors {

namespace {

// ---------------------------------------------------------------------------
// THREE STANDARD NORMALS AS A VECTOR, IN A PINNED ORDER: x, then y, then z.
//
// THIS IS A FUNCTION AND NOT AN INITIALISER LIST, AND THAT IS THE WHOLE POINT.
// Writing `glm::vec3(s.next_gauss(), s.next_gauss(), s.next_gauss())` is A
// DETERMINISM BUG THAT COMPILES: argument evaluation order is unspecified, so
// the three draws could be consumed in any order, and the same source would
// produce different streams under different compilers. Three named locals
// cannot be reordered. imu.cpp:22-27 says the same thing for the same reason,
// and sensor_gnss.slang repeats it a third time because the hazard is identical
// in Slang.
// ---------------------------------------------------------------------------
[[nodiscard]] glm::vec3 draw_gauss3(rng::Stream& stream) noexcept {
    const float x = stream.next_gauss();
    const float y = stream.next_gauss();
    const float z = stream.next_gauss();
    return glm::vec3(x, y, z);
}

}  // namespace

// ---------------------------------------------------------------------------
// synthesize_gnss() -- the gnss.synthesize pass's work, for ONE
// world's spans. physics/schedule.cpp's pass_sensor_gnss() calls it, in the
// Sensors phase beside imu.synthesize. (Before the module API it was a second
// batched call in one SensorSynthesis pass; the two share no quantity.)
//
// ===========================================================================
// NINE DRAWS PER EMITTED FIX, AND NINE IS ODD, AND THAT IS FINE
//
// bias-walk, position-white, velocity-white, each x/y/z. Unconditionally --
// no branch on a zero sigma -- so that the stream position is a function of the
// emitted-fix count alone and configuring a receiver noise-free does not shift
// every other draw in the world. That discipline is imu.hpp section 4's and
// world/medium.hpp's, inherited deliberately.
//
// THE IMU'S TWELVE IS EVEN AND THIS NINE IS NOT, AND THE DIFFERENCE COSTS
// NOTHING. sensor_imu.slang calls its even count "a small dividend": rng::
// Stream's Box-Muller cache is empty again at the end of every sample, so the
// row's 16 stream bytes are in the same canonical shape after each emission.
// Nine leaves `has_cached == 1` and a live `cached_gauss`. That is still
// BIT-IDENTICAL ACROSS BACKENDS, because core/rng.hpp's Stream and
// shaders/shared/layouts.slang's RngStream mirror each other INCLUDING the
// cache -- both sides carry the same spare forward to the next emission.
//
// It is stated here rather than left to inference, because the neighbouring
// file's even count reads as a rule: AN UNSTATED NON-REQUIREMENT BECOMES A
// REQUIREMENT BY INHERITANCE.
//
// ===========================================================================
// NO exp() AND NO DIVISION IN THIS FUNCTION, AND THAT IS A PARITY DECISION
// RATHER THAN AN OPTIMISATION
//
// gnss.hpp section 2's model is
//
//   bias <- bias * exp(-dt/tau) + N(0, sigma_bias * sqrt(1 - exp(-2 dt/tau)))
//
// Both coefficients depend only on `dt` -- IMMUTABLE AT CREATION -- and on
// `tau`, fixed per sensor. THEY ARE PER-SENSOR CONSTANTS, so they are computed
// ONCE at add_gnss_sensor() time and stored as `bias_retention` and
// `bias_drive`. This function multiplies; it never evaluates a transcendental
// and never divides.
//
// That is what makes the Slang twin bit-identical BY CONSTRUCTION rather than
// by tolerance. shaders/fp32_math.slang:83-88 is explicit that "an OpFDiv is
// not merely less accurate, it is DEVICE-DEPENDENT" -- which is why log32_div()
// is a restoring integer long division, and it is domain-restricted and must
// not be reached for as a general divide. Other kernels do divide
// (collision_dynamic.slang), but those are BANDED physics paths where an ulp
// sits inside a band. A RECURSIVE BIAS FILTER DOES NOT TOLERATE ERROR, IT
// ACCUMULATES IT: an ulp of retention error compounds on every emission and
// walks out of any band eventually. The way to see that is to ask what VARIES,
// not what the formula says.
//
// `sigma_bias` and `bias_tau_s` are both still on the row, unused by this
// function, and they stay: the two derived constants cannot answer "derived
// from what?" on their own, and the next author would otherwise recover tau by
// inverting a float.
//
// ===========================================================================
// WHAT THIS PASS READS AND WRITES: reads body state, writes only sensor rows
// and rings. Two receivers on one body run independently and share no
// accumulator, which is what lets the Slang twin dispatch one thread per sensor
// slot with nothing to serialize.
// ---------------------------------------------------------------------------
void synthesize_gnss(std::span<const BodyState> bodies, std::span<GnssSensorRow> sensors,
                     std::span<GnssFix> rings, uint64_t tick) noexcept {
    for (std::size_t s = 0; s < sensors.size(); ++s) {
        GnssSensorRow& row = sensors[s];

        // Active-high, inert when zero: a free slot, or one whose initialization
        // is still queued, has kind == none. See sensors/kinds.hpp.
        if (row.kind != sensor_kind::gnss) continue;

        // Precondition (gnss.hpp, not runtime-checked): body_slot indexes
        // `bodies` directly.
        const BodyState& body = bodies[row.body_slot];
        if ((body.flags & physics::body_flags::active) == 0u) continue;

        // -------------------------------------------------------------------
        // THE RATE BOUNDARY, spelled exactly as synthesize_imu's so a
        // rate_divider of 0 degrades to "every substep" rather than looping or
        // dividing by zero. This is the first sensor for which the divider is
        // the normal case rather than very nearly always 1 (gnss.hpp section 3).
        // -------------------------------------------------------------------
        row.phase += 1u;
        if (row.phase < row.rate_divider) continue;
        row.phase = 0u;

        // THE DRAWS. Nine, in this order, unconditionally. See the header note.
        const glm::vec3 bias_walk = draw_gauss3(row.noise);
        const glm::vec3 white_pos = draw_gauss3(row.noise);
        const glm::vec3 white_vel = draw_gauss3(row.noise);

        // -------------------------------------------------------------------
        // THE GAUSS-MARKOV ADVANCE -- two multiplies and an add, no exp.
        // Advances once per EMITTED FIX, not per substep: it is the receiver's
        // own error process and runs on the receiver's own clock, the same way
        // the IMU's random walk does.
        //
        // A row with bias_retention == 0 (tau <= 0, gnss_bias_retention()'s
        // documented contract) collapses to bias = bias_drive * walk, and with
        // bias_drive == 0 as well the bias stays identically zero -- which is
        // the noise-free receiver, still consuming its nine draws.
        // -------------------------------------------------------------------
        row.bias = row.bias * row.bias_retention + row.bias_drive * bias_walk;

        // -------------------------------------------------------------------
        // THE TRUE QUANTITIES AT THE ANTENNA, WORLD frame.
        //
        // An antenna is a POINT: there is no mount orientation to conjugate
        // through, which is the structural difference from the IMU and the
        // reason GnssSensorRow is 112 bytes against ImuSensorRow's 128.
        //
        // The lever arm rotates body -> world through the body's orientation,
        // and the antenna's velocity picks up omega x r about the COM. omega is
        // stored BODY-frame, so it is rotated first and the cross product is
        // taken in world frame -- one frame for both operands, never a mixed
        // one.
        // -------------------------------------------------------------------
        const glm::vec3 lever_world = body.orient * row.mount_pos;
        const glm::vec3 omega_world = body.orient * body.omega_body;
        const glm::vec3 pos_true = body.pos + lever_world;
        const glm::vec3 vel_true = body.vel + glm::cross(omega_world, lever_world);

        // -------------------------------------------------------------------
        // THE MEASUREMENT.
        //
        // HORIZONTAL IS X AND Z; VERTICAL IS Y. The world frame is right-handed
        // and Y-UP (gnss.hpp section 1), and a receiver's vertical accuracy is
        // materially worse than its horizontal -- that asymmetry is the reason
        // sigma_h and sigma_v are separate fields, so applying one sigma to all
        // three axes would silently discard the distinction the row was shaped
        // around.
        //
        // Left-to-right: true, then bias, then noise. fp32 addition is not
        // associative, so this grouping is part of the parity contract exactly
        // like every other op order in the engine, and the Slang twin groups it
        // identically.
        // -------------------------------------------------------------------
        GnssFix fix;
        fix.position = pos_true + row.bias +
                       glm::vec3(row.sigma_h * white_pos.x, row.sigma_v * white_pos.y,
                                 row.sigma_h * white_pos.z);
        fix.sigma_h = row.sigma_h;
        fix.velocity = vel_true + row.sigma_vel * white_vel;
        fix.sigma_v = row.sigma_v;

        // THE ACCURACY ESTIMATES TRAVEL WITH THE FIX (gnss.hpp's GnssFix note):
        // a real receiver reports its own accuracy per fix and a filter weights
        // each fix by the number that came with it. Copied from the row here
        // because the model has no per-fix quality variation yet; when it does,
        // this is the line that changes and every consumer already reads the
        // per-fix value rather than the row's.
        //
        // VELOCITY IS ISOTROPIC where position is not: carrier-phase Doppler is
        // nearly immune to the delays that dominate the position error, so
        // sigma_vel is one per-axis number rather than an h/v pair (section 2).

        // Monotonic, per sensor, starting at 1 (editor tech spec TA5; rings.hpp).
        row.last_index += 1u;
        fix.index = row.last_index;
        fix.tick = tick;

        // Every one of GnssFix's six named fields is assigned above and it has
        // no implicit padding (gnss.hpp asserts exactly that), so this
        // whole-object write carries no indeterminate byte into the arena.
        ring_write(rings.subspan(s * kRingDepth, kRingDepth), fix.index, fix);
    }
}

}  // namespace spade::sensors
