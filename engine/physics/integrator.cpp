#include "physics/integrator.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/math_ops.hpp"

namespace spade::physics {

void integrate_bodies(std::span<BodyState> bodies, glm::vec3 gravity, float h) noexcept {
    // `gravity` is per-world, not per-body: the caller reads it once (the
    // schedule's Integrate pass, from the gravity field), which keeps the
    // per-body op sequence below identical to what a GPU thread executes (it
    // reads its world's field row once, before the body loop body).

    for (BodyState& body : bodies) {
        // Inert slots -- freed, tombstoned, or never spawned -- are skipped
        // whole. `continue` BEFORE step 1 is deliberate: it means a skipped
        // body's force_acc/torque_acc are not cleared by step 7 either, so a
        // body that is deactivated mid-run keeps whatever the force passes
        // last wrote instead of silently losing it.
        if ((body.flags & body_flags::active) == 0u) {
            continue;
        }

        // -------------------------------------------------------------------
        // The per-substep op order. THIS SEQUENCE IS THE PARITY CONTRACT
        // (see integrator.hpp): the Slang mirror must perform these same
        // seven steps, in this order, with these groupings, in fp32. Two
        // groupings that look like free choices but are not:
        //
        //   * step 1 DIVIDES by mass. It does not multiply by a precomputed
        //     1/mass: for a general float m, fl(x / m) != fl(x * fl(1/m)),
        //     so a reciprocal "optimization" on either path breaks bit
        //     parity. Step 4, by contrast, MULTIPLIES, because the inverse
        //     inertia is what the layout stores.
        //   * the parentheses in steps 3 and 4 are load-bearing. Float
        //     addition and multiplication are not associative, so
        //     (a + g) * h and a*h + g*h are different numbers.
        // -------------------------------------------------------------------

        // 1. External (non-gravitational) acceleration, world frame.
        //    m/s^2 = N / kg. Gravity is deliberately NOT in here yet: this
        //    quantity is precisely what an accelerometer measures, and step 2
        //    depends on capturing it before step 3 contaminates it.
        const glm::vec3 accel_ext = body.force_acc / body.mass;

        // 2. Specific force, body frame -- what the IMU pass reads.
        //    Spec §5: "Specific force (acceleration minus gravity, body
        //    frame) is captured inside Integrate for the IMU pass -- computed
        //    once, not reconstructed." Total acceleration this substep is
        //    (accel_ext + gravity), so "acceleration minus gravity" IS
        //    accel_ext exactly; capturing it here rather than subtracting
        //    gravity back out afterwards avoids the catastrophic cancellation
        //    that a near-free-fall body would otherwise suffer in fp32 (two
        //    ~9.81 quantities differencing to ~0).
        //
        //    The rotation is by the CONJUGATE of the orientation: `orient` is
        //    body->world, so its inverse (== its conjugate for a unit
        //    quaternion) takes the world-frame acceleration into the body
        //    frame the sensor is bolted to.
        //
        //    WHICH ORIENTATION: the PRE-update one (this is why step 2 sits
        //    ahead of step 6, not after it). Two independent reasons, both
        //    pointing the same way. Physically, this sample is the reading at
        //    the instant the substep begins, and the body's attitude at that
        //    instant is the pre-update `orient`. Numerically, force_acc was
        //    BUILT by the force passes earlier this substep, which rotated
        //    each body-frame element force (rotor thrust, drag) into world
        //    frame using that same pre-update `orient`; undoing it with the
        //    conjugate of the very same quaternion is an exact round trip
        //    back to the body-frame sum. Using the post-update orientation
        //    would instead smear one substep of rotation into every IMU
        //    sample -- a bias proportional to omega*h that no downstream
        //    filter could distinguish from a real accelerometer error.
        body.specific_force = glm::conjugate(body.orient) * accel_ext;

        // 3. Linear velocity, from the total acceleration. Gravity enters the
        //    dynamics here and only here (see integrator.hpp on the Gravity
        //    pass not double-applying it).
        body.vel += (accel_ext + gravity) * h;

        // 4. Angular velocity, from Euler's rigid-body equation in the body
        //    frame:  I_body * dOmega/dt = tau_body - omega x (I_body*omega).
        //
        //    4a. The layout stores the INVERSE principal moments, so recover
        //        I_body for the gyroscopic term by componentwise reciprocal.
        //        A zero component means infinite inertia about that axis
        //        (the axis cannot be angularly accelerated); 1/0 would be an
        //        infinity that step 4c's 0 * inf turns into a NaN, so the
        //        reciprocal is guarded to a finite 0 instead. That is exactly
        //        right for the supported degenerate case -- inv_inertia_diag
        //        all zero, i.e. a rotation-locked body -- where I_body
        //        becomes the zero matrix, the gyroscopic term is identically
        //        zero, and 4c's multiply by a zero inverse freezes omega, as
        //        intended. A PARTIALLY zero inv_inertia_diag is not a
        //        supported configuration: the locked axis's omega is still
        //        correctly frozen, but the free axes would see their
        //        gyroscopic coupling computed with that axis's moment read as
        //        0 rather than infinite. Vehicles either rotate or they do
        //        not; mixing the two per axis is not a rigid body.
        const glm::vec3 inv_I = body.inv_inertia_diag;
        const glm::vec3 I_diag(
            inv_I.x != 0.0f ? 1.0f / inv_I.x : 0.0f,
            inv_I.y != 0.0f ? 1.0f / inv_I.y : 0.0f,
            inv_I.z != 0.0f ? 1.0f / inv_I.z : 0.0f);
        const glm::mat3 I_body(
            I_diag.x, 0.0f, 0.0f,
            0.0f, I_diag.y, 0.0f,
            0.0f, 0.0f, I_diag.z);

        //    4b. Total body-frame torque. gyroscopic_torque() is evaluated at
        //        the PRE-update omega -- `body.omega_body` is still this
        //        substep's incoming value here, because 4c is the assignment
        //        that changes it. That is what makes this an explicit Euler
        //        step on the angular DOF (the rotational half is not
        //        symplectic; D1 buys accuracy with substeps, not with an
        //        implicit solve), and it is the ordering math_ops.hpp's
        //        header spells out: fold gyroscopic_torque's result into the
        //        SAME substep's torque accumulator before deriving dOmega/dt.
        const glm::vec3 torque_total = body.torque_acc + math::gyroscopic_torque(I_body, body.omega_body);

        //    4c. Componentwise: the inertia is diagonal in the body frame, so
        //        I_body^-1 * tau is a per-axis scale, not a matrix solve.
        body.omega_body += (inv_I * torque_total) * h;

        // 5. Position, from the UPDATED velocity -- this is the whole of
        //    "semi-implicit"/"symplectic" Euler, and the reason it does not
        //    pump energy on the translational DOF the way explicit Euler
        //    (which would use the pre-update vel) does.
        body.pos += body.vel * h;

        // 6. Orientation, from the UPDATED angular velocity -- the rotational
        //    counterpart of step 5's velocity-then-position ordering.
        //    integrate_orientation applies q (x) exp(0.5*h*omega_body), which
        //    is exact for omega constant across the substep, and renormalizes
        //    on the way out so quaternion drift cannot accumulate.
        body.orient = math::integrate_orientation(body.orient, body.omega_body, h);

        // 7. Consume the wrench. The accumulators are per-substep, so the
        //    next substep's force passes start from a clean zero rather than
        //    each having to remember to reset. Their previous contents have
        //    already been fully used by steps 1 and 4b.
        body.force_acc = glm::vec3(0.0f);
        body.torque_acc = glm::vec3(0.0f);
    }
}

}  // namespace spade::physics
