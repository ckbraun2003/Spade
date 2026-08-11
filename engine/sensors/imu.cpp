#include "sensors/imu.hpp"

#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>

#include "physics/integrator.hpp"  // body_flags::active

namespace spade::sensors {
namespace {

// ---------------------------------------------------------------------------
// Three standard normals as a vector, IN A PINNED ORDER: x, then y, then z.
//
// THIS FUNCTION EXISTS TO FORCE THE SEQUENCING. Writing
// `glm::vec3(s.next_gauss(), s.next_gauss(), s.next_gauss())` would be a
// determinism bug that compiles: the evaluation order of a function's arguments
// is UNSPECIFIED in C++, so two compilers -- or two optimization levels of the
// same compiler -- could legally assign the three draws to different components.
// The three named locals below are sequenced by their initializations and
// cannot be reordered.
// ---------------------------------------------------------------------------
[[nodiscard]] glm::vec3 draw_gauss3(rng::Stream& stream) noexcept {
    const float x = stream.next_gauss();
    const float y = stream.next_gauss();
    const float z = stream.next_gauss();
    return glm::vec3(x, y, z);
}

}  // namespace

void synthesize_imu(std::span<const BodyState> bodies, std::span<ImuSensorRow> sensors,
                    std::span<ImuSample> rings, uint64_t tick) noexcept {
    for (std::size_t s = 0; s < sensors.size(); ++s) {
        ImuSensorRow& row = sensors[s];

        // Active-high, inert when zero: a free slot, or one whose initialization
        // is still queued, has kind == none. See imu.hpp.
        if (row.kind != sensor_kind::imu) continue;

        // Precondition (imu.hpp, not runtime-checked): body_slot indexes
        // `bodies` directly.
        const BodyState& body = bodies[row.body_slot];
        if ((body.flags & physics::body_flags::active) == 0u) continue;

        // -------------------------------------------------------------------
        // THE RATE BOUNDARY. `phase` counts substeps within the current sensor
        // period; a sample is emitted on the substep that completes it. Spelled
        // `<` so a rate_divider of 0 degrades to "every substep" rather than
        // looping or dividing by zero.
        // -------------------------------------------------------------------
        row.phase += 1u;
        if (row.phase < row.rate_divider) continue;
        row.phase = 0u;

        // -------------------------------------------------------------------
        // THE DRAWS. TWELVE STANDARD NORMALS PER EMITTED SAMPLE, IN THIS ORDER,
        // UNCONDITIONALLY -- accel bias walk, gyro bias walk, accel white, gyro
        // white, each x/y/z. No branch on a zero sigma, for the reason
        // world/medium.hpp gives for Dryden's five: the stream position must be
        // a function of the emitted-sample count alone, so that configuring a
        // sensor noise-free does not shift every other draw in the world.
        //
        // The count being EVEN is a small dividend: rng::Stream's Box-Muller
        // cache is empty again at the end of every sample, so a row's 16 stream
        // bytes are in the same canonical shape after each emission.
        // -------------------------------------------------------------------
        const glm::vec3 bias_walk_a = draw_gauss3(row.noise);
        const glm::vec3 bias_walk_g = draw_gauss3(row.noise);
        const glm::vec3 white_a = draw_gauss3(row.noise);
        const glm::vec3 white_g = draw_gauss3(row.noise);

        // The random walk advances once per SAMPLE (not per substep): it is the
        // sensor's own error process, so it runs on the sensor's own clock.
        row.bias_a += row.sigma_ba * bias_walk_a;
        row.bias_g += row.sigma_bg * bias_walk_g;

        // -------------------------------------------------------------------
        // THE TRUE QUANTITIES AT THE MOUNT, body frame.
        //
        // The centripetal term omega x (omega x r) is the lever arm's whole
        // v2 contribution; the Euler term alpha x r is deliberately absent (see
        // imu.hpp section 2). It is zero for a COM mount (r == 0) and zero at
        // zero spin, so the common case pays nothing for it either way.
        //
        // THE TWO READS BELOW ARE NOT AT THE SAME INSTANT, and that is a
        // documented, accepted asymmetry rather than an oversight -- read
        // imu.hpp section 3 before "fixing" it. This pass runs after Integrate,
        // so `omega_body` is the SUBSTEP-END value (integrator.cpp step 4c has
        // already added this substep's angular acceleration to it) while
        // `specific_force` is a SUBSTEP-START quantity, captured by
        // integrator.cpp step 2 against the pre-update orientation on purpose.
        // The gap is exactly alpha*h, zero at zero torque, and the state holds
        // no pre-update omega to close it with. test_imu.cpp's
        // GyroReadsThePostUpdateOmegaOfItsSubstep and
        // AccelUsesTheSubstepStartOrientation pin which value each channel
        // reads, so changing either instant fails loudly.
        // -------------------------------------------------------------------
        const glm::vec3 omega = body.omega_body;
        const glm::vec3 centripetal = glm::cross(omega, glm::cross(omega, row.mount_pos));
        const glm::vec3 specific_force_mount_body = body.specific_force + centripetal;

        // mount_orient is mount -> body, so its conjugate reads a body-frame
        // vector in the mount frame -- the same construction, for the same
        // reason, as integrator.cpp's conjugate(orient) reaching the body frame
        // from the world frame.
        const glm::quat body_to_mount = glm::conjugate(row.mount_orient);
        const glm::vec3 accel_true = body_to_mount * specific_force_mount_body;
        const glm::vec3 gyro_true = body_to_mount * omega;

        // -------------------------------------------------------------------
        // THE MEASUREMENT. Bias and white noise are added AFTER the rotation
        // into the mount frame, because both are properties of the SENSOR (its
        // own axes), not of the body. Left-to-right: true, then bias, then
        // noise -- fp32 addition is not associative, so this grouping is part
        // of the parity contract like every other op order in the engine.
        // -------------------------------------------------------------------
        ImuSample sample;
        sample.accel = accel_true + row.bias_a + row.sigma_a * white_a;
        sample._p0 = 0.0f;
        sample.gyro = gyro_true + row.bias_g + row.sigma_g * white_g;
        sample._p1 = 0.0f;

        // Monotonic, per sensor, starting at 1 (editor tech spec TA5; see sensors/rings.hpp).
        row.last_index += 1u;
        sample.index = row.last_index;
        sample.tick = tick;

        // Every one of ImuSample's six named fields is assigned above and it has
        // no implicit padding (imu.hpp asserts that), so this whole-object write
        // carries no indeterminate byte into the arena.
        ring_write(rings.subspan(s * kRingDepth, kRingDepth), sample.index, sample);
    }
}

}  // namespace spade::sensors
