#pragma once

#include <cstddef>
#include <span>
#include <type_traits>

#include "state/layout.hpp"
#include "world/sdf.hpp"

// ---------------------------------------------------------------------------
// The static_contact.resolve pass (engine design spec D3 "analytic SDF worlds", §5
// "Collision"). Every active body is tested against the world's signed
// distance field as a SPHERE PROXY, and a penetrating body gets one impulse
// (restitution + Coulomb friction) plus a Baumgarte-style positional
// correction.
//
// WHERE THIS SITS IN THE SCHEDULE. The module schedule's six phases, per substep:
//
//     Fields -> Forces -> Constraints/Contacts (static, then dynamic) ->
//     Integrate -> Sensors -> Publish (sim/standard_modules.cpp)
//
// so resolve_static_contacts() runs BEFORE integrate_bodies(), on the velocity
// the PREVIOUS substep's Integrate produced. That ordering is what makes the
// friction law come out exactly right without this pass ever seeing a force:
// the incoming normal velocity of a resting body is precisely one substep of
// gravity (v_n = -g*h, applied by Integrate step 3), so the normal impulse
// that cancels it has magnitude g*h and the Coulomb cap mu*g*h per substep
// integrates to a tangential deceleration of exactly mu*g. See the derivation
// on step 2 in contacts.cpp.
//
// PURE FUNCTION OF (state, world, params). Like the integrator, this pass
// holds no state, allocates nothing, reads no clock, draws no randomness, and
// visits bodies in slot order -- so two runs on equal inputs produce
// byte-equal outputs (test_contacts.cpp pins that with memcmp). Bodies without
// body_flags::active are left byte-for-byte untouched.
//
// PARITY. As with the integrator, the numbered op order in contacts.cpp is the
// CPU<->GPU parity contract (P1/P2, D11): the Slang mirror at S6 must perform
// the same operations, in the same order, with the same groupings, in fp32.
//
// ---------------------------------------------------------------------------
// THREE V1 LIMITATIONS, STATED LOUDLY. Each is a deliberate scope ruling, not
// an oversight, and each has a named place where it gets revisited.
//
//  1. THE RESPONSE IS LINEAR-ONLY. Impulses change `vel` and nothing else:
//     `omega_body` and `torque_acc` are never touched, and no contact torque
//     r x J is generated. A sphere dragged across a floor by friction
//     therefore SLIDES FOREVER AND NEVER STARTS ROLLING, and a body landing
//     on one corner does not tip. This is v1-faithful (v1's contact model was
//     linear too) and it keeps the pass free of the moment-arm term, which
//     needs a real per-body contact point rather than the body origin. If
//     landing/tip-over dynamics turn out to matter, the place to revisit is
//     the VEHICLE LAYER (D4) -- that is where per-body contact geometry
//     arrives.
//
//  2. ONE PROXY RADIUS FOR THE WHOLE SPAN, BY DEFAULT -- WITH A PER-BODY
//     OVERRIDE (D-S6-2). `ContactParams::proxy_radius` is now the world's
//     FALLBACK, read only when a body's own `BodyState::proxy_radius` is the
//     0 sentinel -- which is every plain body's slot, and a vehicle's slot
//     whose model left `proxy_radius` at its own 0 default. See
//     effective_proxy_radius() below and state/layout.hpp's BodyState note.
//     Nothing here partitions the span by radius class any more: the per-body
//     read is a single branch per body, no different in shape from the
//     `body_flags::active` test already in the loop.
//
//  3. ONE CONTACT PER BODY PER SUBSTEP. The world SDF is a single scalar
//     field: a union of two walls reports ONE distance (the smaller) and ONE
//     gradient (that branch's), so a sphere wedged in a corner sees only the
//     nearer surface each substep. There is no manifold and no warm starting.
//     What makes this stable rather than merely lossy is that the branch the
//     union selects is always the MOST PENETRATED one -- correcting it makes
//     its distance grow until the other becomes the nearest, so successive
//     substeps alternate between the corner's faces in a Gauss-Seidel-like
//     sweep that drives BOTH penetrations down to the slop band. The corner
//     test pins that (it asserts stability and a bounded resting penetration
//     on both faces, deliberately NOT an exact trajectory, since the
//     alternation order is a property of the union, not of physics).
//     Multi-contact manifolds are not planned for the CPU twin; the GPU pass
//     at S6 is where a manifold would be affordable.
//
//  4. DETECTION IS DISCRETE -- THERE IS NO CONTINUOUS COLLISION DETECTION.
//     The test samples the field at the body's CURRENT position only, so a
//     body that crosses more than its proxy radius in one substep can pass
//     through a surface entirely. The usable envelope is therefore
//     |v| * h < proxy_radius, which at the pinned 1 kHz substep and a 100 mm
//     proxy is 100 m/s -- an order above the 13.9 m/s a 10 m drop reaches, and
//     the margin the drop test measures rather than assumes. THIN GEOMETRY IS
//     THE SHARPER LIMIT: a wall thinner than the per-substep advance can be
//     missed even at moderate speed, because the field goes negative and back
//     positive between two samples. Speculative contacts (looking ahead
//     vel*h, which is what the unused `h` parameter below is reserved for) are
//     the cheap fix if a world ever needs one.
//
// Body-vs-body contact is dynamic_contact.resolve (physics/grid.*), not this file.
// ---------------------------------------------------------------------------

namespace spade::physics {

// ---------------------------------------------------------------------------
// Per-world contact-solver parameters. One record for the whole span, matching
// WorldParams' per-world granularity -- material properties are a property of
// the world in v1, not of the body.
//
// WHY THIS STRUCT IS NOT IN state/layout.hpp, AND WHY IT CARRIES LAYOUT.HPP'S
// ASSERT BATTERY ANYWAY (coordinator ruling, 2026-08-09). The global
// constraints put "all shared POD state structs" in layout.hpp with
// static_asserts on size and offset. The ruling draws the line by RESIDENCY,
// not by shape: layout.hpp holds memory-resident, snapshot-walked engine
// STATE (what ArenaSet allocates and Task 7's registry walk serializes);
// PASS-PARAMETER VALUE STRUCTS like this one stay in their pass's header --
// they are arguments, not state, and nothing snapshots them -- but they carry
// the full layout discipline HERE so the S6 Slang mirror has something pinned
// to mirror. THE GRADUATION PATH: when the schedule makes this a
// device-resident per-world row (an indexed param buffer alongside
// WorldParams, which is what happens when static contact becomes a GPU
// dispatch), it moves to layout.hpp / the Slang shared module and these
// asserts move with it unchanged. Until then this is the single source.
//
// THE LAYOUT ITSELF. Eight floats in two 16-byte std430 rows:
//
//   row 0  restitution_e | friction_mu | baumgarte_beta | slop
//   row 1  proxy_radius  | _r0         | _r1            | _r2
//
// alignas(16) gives std430's base alignment for a struct in an array, and 32
// bytes is already a whole number of rows, so sizeof IS the array stride and
// no implicit tail padding exists (asserted below both ways). The three
// reserved lanes are named, default-initialized to zero, and reserved for
// versioned growth -- per-material or per-axis friction is the obvious next
// field -- so growth consumes them without moving any existing field or
// changing the stride.
//
// FIELD ORDER IS THE PINNED ORDER, UNCHANGED. This was the one free moment to
// reorder, and there is nothing to gain: every member is a scalar float, and
// std430 gives scalars a 4-byte base alignment, so all 5! orderings pack to
// byte-identical images. No vec2/vec3/vec4 member exists to create the
// straddling-a-16-byte-boundary hazard that motivates reordering in the first
// place. Reordering would have been churn with no std430 benefit.
//
// UNITS: restitution_e and friction_mu are dimensionless; baumgarte_beta is
// dimensionless (a FRACTION of the excess penetration removed per substep, not
// the 1/s stiffness of the velocity-form Baumgarte stabilization); slop and
// proxy_radius are metres.
//
// PRECONDITIONS, in the same not-validated-at-runtime spirit as the rest of
// this file (the physics inner loop does not check its arguments; a world
// author or the config layer owns this): restitution_e in [0, 1],
// friction_mu >= 0, baumgarte_beta in [0, 1], slop >= 0, proxy_radius >= 0.
// A negative friction_mu makes the Coulomb cap negative, which turns step 2's
// `min` into a tangential ACCELERATION along v_t; a negative baumgarte_beta
// drives the positional correction INTO the surface. Neither is diagnosed --
// they are simply not configurations this pass has meaning for. Values of
// restitution_e above 1 are likewise not rejected but pump energy without
// bound; beta above 1 overshoots the surface and oscillates.
//
// The defaults describe an inert pass: a zero-radius proxy only ever contacts
// when the body ORIGIN is inside the solid, with a perfectly inelastic,
// frictionless response. A default-constructed ContactParams therefore cannot
// silently inject energy into a world whose author forgot to configure it, and
// its 32-byte image is fully determined (every lane has an initializer), which
// is what makes the record hashable and uploadable byte-wise.
// ---------------------------------------------------------------------------
struct alignas(kStd430StructAlignment) ContactParams {
    // Coefficient of restitution. 0 = perfectly inelastic (the resting case,
    // and the only value for which "at rest" is reachable in finite time);
    // 1 = lossless bounce. Values > 1 are not rejected here but are not
    // physical and will pump energy without bound.
    float restitution_e = 0.0f;

    // Coulomb friction coefficient. The tangential velocity change is capped
    // at mu times the NORMAL IMPULSE (see step 2), which is the velocity-space
    // form of |f_t| <= mu*|f_n|. There is no static/dynamic split: one mu.
    float friction_mu = 0.0f;

    // Baumgarte positional-correction gain: the fraction of the EXCESS
    // penetration (depth - slop) pushed out along the normal each substep.
    // 0.2 is the pinned default -- large enough that a deep first-contact
    // penetration decays geometrically (0.8^n) within a few dozen substeps,
    // small enough not to fight the impulse or add visible energy. The
    // correction is applied per SUBSTEP, so a caller that changes the substep
    // count changes the effective correction rate; that is intended (the
    // correction is a position filter, not a force).
    float baumgarte_beta = 0.2f;

    // Penetration allowance, metres. Penetration up to `slop` is left alone,
    // so a resting body sits in a shallow, quiet band instead of being pushed
    // out and re-penetrating every substep. A resting body under gravity
    // equilibrates at depth = slop + g*h^2/beta (the sink per substep balanced
    // against the correction) -- 1.05 mm at h = 1 ms, beta = 0.2, slop = 1 mm.
    float slop = 1e-3f;

    // Sphere-proxy radius, metres -- this world's DEFAULT (D-S6-2: a body's own
    // BodyState::proxy_radius overrides it when that field is nonzero; see
    // effective_proxy_radius() below and limitation 2 above).
    float proxy_radius = 0.0f;

    // Reserved for versioned growth; must stay 0. Named rather than left as
    // implicit tail padding so that EVERY byte of this record belongs to a
    // field -- the property the "named bytes" assert below pins, and the one
    // thing a byte-wise comparison or hash of the record cannot reason about
    // otherwise.
    float _r0 = 0.0f;
    float _r1 = 0.0f;
    float _r2 = 0.0f;
};

// ---------------------------------------------------------------------------
// The layout battery, in state/layout.hpp's style and for its stated reason:
// these asserts are the enforcement, not decoration. A layout change a human
// would have missed becomes a build error here, and they survive the S6
// handover to a slangc-generated header unchanged -- they are what makes the
// generated header's agreement with these hand-written expectations checkable.
// ---------------------------------------------------------------------------
static_assert(std::is_standard_layout_v<ContactParams>,
              "ContactParams must be standard-layout for offsetof to be meaningful");
static_assert(std::is_trivially_copyable_v<ContactParams>,
              "ContactParams must be memcpy-able: it uploads to a device buffer verbatim at S6");
static_assert(std::is_trivially_destructible_v<ContactParams>,
              "ContactParams is a value record, never individually destroyed");
static_assert(alignof(ContactParams) == 16, "std430 base alignment");
static_assert(sizeof(ContactParams) == 32, "std430 array stride (two 16-byte rows, no tail pad)");

static_assert(offsetof(ContactParams, restitution_e) == 0);
static_assert(offsetof(ContactParams, friction_mu) == 4);
static_assert(offsetof(ContactParams, baumgarte_beta) == 8);
static_assert(offsetof(ContactParams, slop) == 12);
static_assert(offsetof(ContactParams, proxy_radius) == 16);
static_assert(offsetof(ContactParams, _r0) == 20);
static_assert(offsetof(ContactParams, _r1) == 24);
static_assert(offsetof(ContactParams, _r2) == 28);

// Each 16-byte row starts on a 16-byte boundary -- trivially true for an
// all-scalar struct, asserted anyway so that adding a vec3 lane later cannot
// silently straddle a row the way std430 forbids.
static_assert(offsetof(ContactParams, restitution_e) % 16 == 0);
static_assert(offsetof(ContactParams, proxy_radius) % 16 == 0);

// Named fields account for every byte: no IMPLICIT padding anywhere in
// ContactParams. This is the assert that catches alignas(16) having quietly
// added a tail pad (as it legitimately does to WorldParams, whose 56 named
// bytes round to a 64-byte stride) -- here the named total must BE the stride,
// with nothing unaccounted for.
static_assert(sizeof(ContactParams::restitution_e) + sizeof(ContactParams::friction_mu) +
                  sizeof(ContactParams::baumgarte_beta) + sizeof(ContactParams::slop) +
                  sizeof(ContactParams::proxy_radius) + sizeof(ContactParams::_r0) +
                  sizeof(ContactParams::_r1) + sizeof(ContactParams::_r2) ==
              sizeof(ContactParams),
              "ContactParams has implicit padding: every byte must belong to a named field");

// ---------------------------------------------------------------------------
// THE PER-BODY PROXY OVERRIDE (D-S6-2). One PAIR of functions -- a plain-float
// rule and a BodyState-reading convenience over it -- so every reader of "this
// body's radius" (static contact in contacts.cpp, dynamic contact in
// grid.cpp, AND the vehicle-spawn preconditions in sim/simulation.cpp, which
// have a model's DECLARED radius but no BodyState yet to read one from) shares
// the SAME rule and cannot drift apart on what a 0 means.
//
// `body_radius` is state/layout.hpp's BodyState::proxy_radius row-0 lane (0 in
// every slot a bare spawn() produces, and in every vehicle slot whose model
// left its own proxy_radius at 0 -- vehicles/model_type.hpp) OR, at spawn time
// before that slot exists, the model's own ModelType::proxy_radius directly --
// the two are the SAME NUMBER, since Simulation::spawn(world, ModelTypeId,
// VehicleSpawn) writes one into the other verbatim. `world_default` is the
// caller's ContactParams::proxy_radius -- pass THAT world's row, not some
// other one's.
//
// 0 IS A SENTINEL, NOT AN AUTHORED RADIUS. A body (or model) that never asked
// for an override therefore reads EXACTLY the single world-wide value every
// body read before this function existed, which is what makes a plain-body
// world's digest impossible to move by this change: MakeUnitBody()-style test
// fixtures and every pre-D-S6-2 snapshot blob leave this field at its
// zero-fill, and the fallback IS the old behaviour.
// ---------------------------------------------------------------------------
[[nodiscard]] inline float effective_proxy_radius(float body_radius, float world_default) noexcept {
    return body_radius != 0.0f ? body_radius : world_default;
}

[[nodiscard]] inline float effective_proxy_radius(const BodyState& body, float world_default) noexcept {
    return effective_proxy_radius(body.proxy_radius, world_default);
}

// ---------------------------------------------------------------------------
// Resolves every ACTIVE body in `bodies` against the static world SDF.
//
// CONTACT TEST: phi = eval(world_sdf, pos); radius = effective_proxy_radius
// (body, params.proxy_radius) (D-S6-2: the body's own override, or this
// world's default); a contact exists iff phi < radius, with depth = radius -
// phi and outward normal n = normalize(gradient(world_sdf, pos)). Distance and
// gradient come from a SINGLE sample() walk of the program (world/sdf.hpp),
// not from separate eval()/gradient() calls -- half the work, and it makes the
// two provably consistent.
//
// A body with |gradient| == 0 at its position is skipped: the field has no
// defined normal there (the centre of a sphere primitive, the seam of a
// smooth_union between opposed faces, an empty program), and an arbitrary
// axis would be worse than no impulse. Elsewhere the gradient is normalized
// UNCONDITIONALLY rather than assumed unit -- the heightfield primitive's
// field is a Lipschitz bound with |grad| <= 1, and smooth_union blends two
// unit gradients into a shorter one, so treating |grad| as 1 would scale
// every impulse in those regions by the wrong factor.
//
// `bodies` is normally one world's partition (ArenaSet::world_slice), which is
// why `params` -- the world's MATERIAL and its default radius -- is a single
// record rather than per body; the radius itself is read per body regardless
// (see effective_proxy_radius() above).
//
// FRAMES: pos, vel and the SDF are all world-frame; nothing body-frame is read
// or written (see limitation 1).
//
// `h` is the effective substep duration in seconds. It is currently UNUSED --
// every term of the pinned model (restitution, the Coulomb cap, and the
// beta-fraction positional correction) is expressed directly in velocity and
// position space and is dimensionally independent of dt. It stays in the
// signature deliberately, for two reasons: it makes this pass's shape match
// integrate_bodies(span, params, h) so the schedule can call every pass
// uniformly, and the two natural refinements to this model -- a restitution
// cutoff below ~2*g*h to suppress micro-bounces, and speculative contacts that
// look ahead vel*h to catch tunnelling at low proxy radii -- both need it.
// Adding either would change the pinned op order and is out of scope here.
//
// PRECONDITION: `world_sdf` has passed SdfProgram::validate(). This is the
// physics inner loop; it does not revalidate.
// ---------------------------------------------------------------------------
void resolve_static_contacts(std::span<BodyState> bodies, const SdfProgram& world_sdf,
                             const ContactParams& params, float h) noexcept;

}  // namespace spade::physics
