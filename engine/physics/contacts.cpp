#include "physics/contacts.hpp"

#include <glm/glm.hpp>

#include "physics/integrator.hpp"  // body_flags::active
#include "world/sdf.hpp"

namespace spade::physics {

namespace {

// L9: the set's point that penetrates most -- each point placed by
// contact_point_world() and sampled, the smallest distance winning by STRICT
// `<` from SdfSample's kSdfEmptyDistance, so the first point in the table's
// order wins a tie and a NaN never wins -- or `found` false when the body's
// lane names no range of points in `table` (the body then takes the sphere).
[[nodiscard]] SdfSample deepest_point(const BodyState& body, const SdfProgram& world_sdf,
                                      std::span<const float> table, bool& found) noexcept {
    SdfSample deepest{};
    found = false;
    if (body.contact_set == 0u) return deepest;
    const std::size_t range = (std::size_t{body.contact_set} - 1u) * kContactSetRowFloats;
    if (range + kContactSetRowFloats > table.size()) return deepest;
    // Exact-integer floats below 2^24 (the layout's), so the casts are exact.
    const auto first = static_cast<std::size_t>(table[range]);
    const auto count = static_cast<std::size_t>(table[range + 1u]);
    if (count == 0u || (first + count) * kContactSetRowFloats > table.size()) return deepest;
    found = true;
    for (std::size_t k = 0; k < count; ++k) {
        const std::size_t at = (first + k) * kContactSetRowFloats;
        const glm::vec3 local(table[at], table[at + 1u], table[at + 2u]);
        const SdfSample s = spade::sample(world_sdf, contact_point_world(body, local));
        if (s.distance < deepest.distance) deepest = s;
    }
    return deepest;
}

}  // namespace

void resolve_static_contacts(std::span<BodyState> bodies, const SdfProgram& world_sdf,
                             const ContactParams& params, [[maybe_unused]] float h,
                             std::span<ContactDvRow> contact_dv, std::span<const float> contact_sets) noexcept {
    // Hoisted once: per-world, not per-body, so the per-body op sequence below
    // is exactly what a GPU thread executes after reading its world's param
    // row. (`h` is unused by the pinned model -- see contacts.hpp.) `radius`
    // is NOT hoisted with these four (D-S6-2): it is now a per-body quantity,
    // computed once per body below via effective_proxy_radius() rather than
    // once per call, which is the one-branch cost of the per-body override.
    const float e = params.restitution_e;
    const float mu = params.friction_mu;
    const float beta = params.baumgarte_beta;
    const float slop = params.slop;

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        BodyState& body = bodies[i];
        // Same skip contract as Integrate: an inert slot -- freed, tombstoned
        // or never spawned -- is left byte-for-byte untouched. flags == 0 is
        // the zero-filled arena state, so an unallocated slot costs one bit
        // test and no SDF walk.
        if ((body.flags & body_flags::active) == 0u) {
            continue;
        }

        // D-S6-2: this body's own override, or the world's default -- see
        // contacts.hpp's effective_proxy_radius(). A GPU thread performs the
        // same single select after reading its body's row and its world's
        // param row; nothing here diverges by thread.
        float radius = effective_proxy_radius(body, params.proxy_radius);

        // L9: a body with a contact set rests on its most penetrated point,
        // which takes the sphere's response below at radius 0 (contacts.hpp's
        // A BODY WITH A CONTACT SET). Otherwise one walk of the program at the
        // body's position yields both the distance and the gradient
        // (world/sdf.hpp); sample().distance is bit-identical to eval().
        bool from_set = false;
        SdfSample s = deepest_point(body, world_sdf, contact_sets, from_set);
        if (from_set) {
            radius = 0.0f;
        } else {
            s = spade::sample(world_sdf, body.pos);
        }

        // Contact test. STRICT `<`: a body exactly tangent to the surface
        // (phi == radius, depth == 0) is not in contact, so a body resting
        // exactly on the tangent point gets no impulse and no correction --
        // there is nothing to correct, and the strict form keeps the "no
        // contact" and "zero-depth contact" cases from differing by an ulp.
        //
        // Spelled as `!(a < b)` rather than `a >= b` so a NaN distance FAILS
        // to be a contact and is skipped, instead of comparing false on both
        // sides and falling through into a normalize() of a NaN gradient. An
        // empty program takes the same exit: eval reports kSdfEmptyDistance,
        // which is not less than any sane radius.
        if (!(s.distance < radius)) {
            continue;
        }

        // Penetration depth of the sphere proxy, metres. Positive by the test
        // above. Note phi may itself be negative (the body ORIGIN inside the
        // solid), in which case depth exceeds the radius -- the arithmetic is
        // the same and needs no special case.
        const float depth = radius - s.distance;

        // Outward unit normal. The gradient of a signed distance field points
        // towards INCREASING distance, i.e. out of the solid, which is the
        // direction a contact must push. Normalized unconditionally -- see
        // contacts.hpp on the heightfield and smooth_union.
        // `!(x > 0)` for the same NaN reason as the distance test above.
        const float grad_len = glm::length(s.gradient);
        if (!(grad_len > 0.0f)) {
            // No defined normal here (sphere centre, opposed-face seam, empty
            // program). Skipping is the honest answer; an invented axis would
            // inject momentum in a direction nothing asked for.
            continue;
        }
        const glm::vec3 n = s.gradient / grad_len;

        // -------------------------------------------------------------------
        // The per-contact op order. THIS SEQUENCE IS THE PARITY CONTRACT (see
        // contacts.hpp): the Slang mirror must perform these same three steps,
        // in this order, in fp32.
        //
        // EVERYTHING BELOW WORKS IN VELOCITY SPACE, and that is a modelling
        // choice worth spelling out because it removes the mass from the
        // arithmetic. The Newton impulse for a sphere against a STATIC,
        // infinitely massive world is
        //
        //     J = -(1 + e) * v_n / (1/m + 1/M + (moment-arm terms))
        //
        // with M -> infinity (the world does not recoil) and the moment-arm
        // terms absent (limitation 1: no contact torque). The effective mass
        // is therefore exactly m, so the velocity change is
        //
        //     dv = J/m * n = -(1 + e) * v_n * n
        //
        // -- the mass cancels identically. `j_n` below is that mass-normalized
        // impulse, so it is a SPEED (m/s), not an impulse in N s. The same
        // cancellation is what lets the Coulomb cap be written directly as
        // mu * j_n. A future per-body-mass term (a body pushing a movable
        // obstacle) is Task 12's business, not this pass's.
        // -------------------------------------------------------------------

        // 1. NORMAL IMPULSE. Only an APPROACHING contact gets one: v_n >= 0
        //    means the body is already separating (or sliding exactly along
        //    the surface), and impulsing it again would be the classic
        //    double-hit that turns a single bounce into an energy pump. The
        //    guard is what makes it safe to keep resolving a contact that
        //    persists for several substeps after the bounce while the
        //    positional correction finishes pushing the body clear.
        //
        //    Post-condition when it fires: dot(vel, n) == -e * v_n, i.e. the
        //    separating speed is e times the approach speed -- the definition
        //    of the coefficient of restitution.
        const glm::vec3 vel_in = body.vel;  // PHY-7: the change below is the IMU's
        const float v_n = glm::dot(body.vel, n);
        float j_n = 0.0f;
        if (v_n < 0.0f) {
            j_n = -(1.0f + e) * v_n;
            body.vel += j_n * n;
        }

        // 2. COULOMB FRICTION, applied to the tangential velocity that remains
        //    AFTER step 1. (The normal impulse only adds along n, so the
        //    tangential part is analytically unchanged by it; recomputing here
        //    rather than before step 1 is what the pinned order says, and it
        //    is the form a solver with a non-trivial effective-mass matrix
        //    would need.)
        //
        //    The cap is mu * j_n -- the velocity-space reading of
        //    |f_t| <= mu * |f_n|. Two consequences:
        //
        //      * A SEPARATING CONTACT HAS NO FRICTION. j_n is 0 when step 1
        //        did not fire, so the cap is 0 and this step is a no-op. That
        //        falls out of the algebra rather than needing its own branch,
        //        and it is physically right: no normal force, no friction.
        //      * THE DECELERATION IS EXACTLY mu*g. Because this pass runs
        //        BEFORE Integrate, a body resting on a surface arrives with
        //        exactly one substep of gravity in its normal velocity
        //        (v_n = -g*h), so j_n = g*h and the tangential loss is
        //        mu*g*h per substep -- mu*g per second, independent of h.
        //
        //    The `min` against |v_t| is the STICTION FLOOR and it is
        //    load-bearing: without it a large mu (or a large j_n from a hard
        //    landing) would subtract more than the whole tangential velocity
        //    and REVERSE it, i.e. friction would accelerate the body
        //    backwards. Capped at |v_t|, the worst case is an exact stop --
        //    exact up to fp32 rounding, that is: when the cap binds, the
        //    subtracted vector is |v_t| * fl(v_t/|v_t|), which reproduces v_t
        //    to within about two roundings, so a residue of order eps*|v_t|
        //    (~1e-6 m/s off a 3 m/s slide) can survive with either sign. It
        //    cannot grow: the next substep's cap is at least as large as the
        //    residue and stops it again. test_contacts.cpp bounds this
        //    explicitly rather than pretending the stop is bit-exact.
        const float v_n_post = glm::dot(body.vel, n);
        const glm::vec3 v_t = body.vel - v_n_post * n;
        const float v_t_len = glm::length(v_t);
        if (v_t_len > 0.0f) {
            const float dv_t = glm::min(mu * j_n, v_t_len);
            body.vel -= dv_t * (v_t / v_t_len);
        }
        if (!contact_dv.empty()) {
            contact_dv[i].dv += body.vel - vel_in;
        }

        // 3. POSITIONAL CORRECTION (Baumgarte-style, applied to `pos`
        //    directly). Steps 1 and 2 are velocity-level: they stop the body
        //    sinking further but cannot undo the penetration already present
        //    when the contact was discovered, which for a fast body is up to
        //    |v| * h. This pushes a beta-fraction of the EXCESS penetration
        //    out along the normal each substep, so a deep first hit decays
        //    geometrically as (1 - beta)^n.
        //
        //    THREE PROPERTIES, ALL DELIBERATE:
        //      * NO VELOCITY CONTRIBUTION. Moving `pos` without touching `vel`
        //        is what keeps this from adding energy: the classic
        //        velocity-form bias (beta/h * depth added to v_n) would show
        //        up as a real, restitution-scaled bounce. Here a corrected
        //        body has the same momentum it had before.
        //      * CLAMPED AT ZERO by the max: penetration inside the slop band
        //        is left exactly alone, so a body that is overlapping but not
        //        being driven deeper is a bit-exact no-op for this pass.
        //        NOTE WHAT THIS DOES *NOT* SAY. A body resting UNDER GRAVITY
        //        does not come to rest inside the band: it settles just below
        //        it, at depth = slop + g*h^2/beta, where the correction fires
        //        EVERY substep and exactly cancels the g*h^2 that the previous
        //        Integrate sank it. That equilibrium is a fixed point -- the
        //        position is stationary, and measurably so -- but it is an
        //        actively balanced one, not a dormant one. The slop's job is
        //        to bound the resting penetration and keep the correction
        //        small, not to switch it off. contacts.hpp's `slop`
        //        documentation states the same equilibrium; the two must
        //        agree.
        //      * APPLIED WHETHER OR NOT STEP 1 FIRED. A rebounding body that
        //        is still overlapping keeps being pushed clear; that is the
        //        point of separating the position and velocity levels.
        //
        //    max(depth - slop, 0) is spelled with glm::max on the SUBTRACTION
        //    (not a branch on depth > slop) so the GPU mirror is branch-free.
        const float correction = beta * glm::max(depth - slop, 0.0f);
        body.pos += n * correction;
    }
}

}  // namespace spade::physics
