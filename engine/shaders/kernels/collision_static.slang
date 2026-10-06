// ---------------------------------------------------------------------------
// collision_static.slang (S6 Task 6) -- the static_contact.resolve pass, ported from
// engine/physics/contacts.cpp's resolve_static_contacts(). One thread per
// GLOBAL body slot: sample the world SDF at the body's position, and if the
// sphere proxy penetrates, apply one impulse (restitution + Coulomb friction)
// plus a Baumgarte-style positional correction.
//
// The three-step op order in contacts.cpp:69-187 IS the parity contract; each
// step below cites the lines it mirrors. The SDF walk itself lives in
// engine/shaders/sdf_eval.slang (imported below), which is the port of
// world/sdf.cpp.
//
// ===========================================================================
// PER-WORLD MATERIAL, FROM A BUFFER, NOT FROM THE PUSH CONSTANT
//
// The CPU pass reads `w.contacts` -- the material record of the world it is
// resolving (physics/schedule.cpp's pass_collision_static). That record is
// genuinely PER WORLD: tests/golden/scenarios/bounce.scenario.yaml is a
// restitution LADDER whose four worlds carry four different restitution_e
// values, and applying one world's material to another's bodies would be
// exactly the silent error WorldSetLayout::uniform_dynamic_params exists to
// prevent on the CPU side.
//
// So this kernel indexes `contact_params[world]` (bindings.slang section C),
// a per-world buffer uploaded once at Simulation::create(). PassParams itself
// no longer has anywhere to read a per-batch material from at all: it carried
// a `contact` field (world 0's record, meaningful only to a future BATCHED
// CollisionDynamic form) through S6 Task 6, but Task 6b's PassParams decision
// tree found no kernel ever read it and removed it (layouts.slang's
// PassParams doc comment has the ruling) -- this comment is the historical
// record of why that field could never have served CollisionStatic anyway.
//
// ===========================================================================
// THE PER-BODY PROXY RADIUS -- LIVE (S6 Task 2 / D-S6-2, merged into this
// branch at spade/s6's T2 merge commit).
//
// `BodyState::proxy_radius` (state/layout.hpp; the field this kernel's own T6
// draft carried as the padding lane `_p0`, renamed at the T2 merge) is a
// per-body override: 0 is a SENTINEL meaning "use this world's
// ContactParams::proxy_radius". Simulation::spawn(world, ModelTypeId,
// VehicleSpawn) is the only writer of a nonzero value (the spawning model's
// own ModelType::proxy_radius); a bare spawn(world, BodySpawn) always leaves
// it at the zero-fill.
//
// THE SENTINEL PREDICATE MIRRORS physics::effective_proxy_radius()
// (physics/contacts.hpp) CHARACTER-FOR-CHARACTER -- that C++ function is the
// ONE canonical rule (coordinator ruling, T2 merge carry item 3):
//
//     body_radius != 0.0f ? body_radius : world_default
//
// so this kernel spells the same comparison (`!=`, not `>`) and the same
// fallback below. A negative radius is not validated against here, same as
// the C++ side: authoring-time validation (ModelType, vehicles/model_type.hpp)
// is what keeps one from ever reaching either side of this seam.
//
// ===========================================================================
// DIVISION AND SQRT SITES in THIS file (sdf_eval.slang lists its own):
//   SQRT  glm::length(gradient)                    (contacts.cpp:60)
//         glm::length(v_t)                         (contacts.cpp:146)
//   DIV   gradient / grad_len                      (contacts.cpp:67)
//         v_t / v_t_len                            (contacts.cpp:149)
// Every one mirrors the CPU's own operation at the same place. Downstream
// kernels may use both (global constraint); this is why CPU<->GPU here is
// banded rather than bit-identical, and the SDF's own sqrt chain is the
// dominant contributor.
//
// NO WORKGROUP-SHARED ANYTHING: one body per thread, no cross-lane traffic.
// ---------------------------------------------------------------------------

// `import layouts` AS WELL AS `import bindings`: Slang's `import` is NOT
// transitive -- bindings.slang imports layouts.slang for its own declarations,
// but that does not re-export the STRUCT NAMES to a module importing bindings.
// A module that only SUBSCRIPTS a buffer never needs them; one that names an
// element type (`const SdfNodeRow node = ...`) does. (The example this note
// used to give was pipeline_smoke.slang, the stub kernel S6 Task 8 deleted;
// every module in the tree now names element types, so the distinction is
// stated rather than illustrated.)
import layouts;
import bindings;
import sdf_eval;
// glm_min/glm_max/glm_dot3/glm_length3 AND effective_proxy_radius, which this
// kernel declared for itself through S6 Task 7. Extracted to one shared module
// by Task 8 (T7 review minor M3): the old comment claimed the duplicated copies
// "must not be able to read the same law two different ways", which was an
// intention rather than a mechanism. Importing one definition is the mechanism.
import mirrors;

static const uint kInvalidWorld = 0xFFFFFFFFu;
static const uint kBodyFlagActive = 1u;

[shader("compute")]
// LOCAL SIZE COMES FROM THE BUILD, NOT FROM THIS LINE (S6 Task 9b).
// cmake/SpadeSlang.cmake's spade_slang_kernel_variants() compiles this kernel
// once per size in SPADE_SLANG_WORKGROUP_SIZES (= compute/backend.hpp's
// kSupportedWorkgroupSizes), passing -DSPADE_WG=<size>, so that
// BackendDesc::workgroup_size selects a module that was actually compiled for
// it. There is DELIBERATELY no #ifndef fallback: a missing -D must fail the
// build loudly, not silently produce a 64-wide kernel that step_recorder.cpp
// then dispatches on a grid divided by some other number.
[numthreads(SPADE_WG, 1, 1)]
void collision_static(uint3 tid : SV_DispatchThreadID) {
    const uint slot = tid.x;
    if (slot >= pass_params.world_count * pass_params.body_capacity) {
        return;
    }

    const uint world = bodies_slot_to_world[slot];
    if (world == kInvalidWorld) {
        return;
    }

    // contacts.cpp:26-28 -- an inert slot is left byte-for-byte untouched.
    if ((bodies[slot].flags & kBodyFlagActive) == 0u) {
        return;
    }

    // contacts.cpp:15-19 -- hoisted once, per world, exactly as the CPU hoists
    // them outside the body loop.
    const ContactParams material = contact_params[world];
    const float e = material.restitution_e;
    const float mu = material.friction_mu;
    const float beta = material.baumgarte_beta;
    const float slop = material.slop;

    // The per-body proxy radius with its sentinel-0 fallback -- see the header
    // note. mirrors.slang's effective_proxy_radius() is the ONE definition both
    // contact kernels call, mirroring physics::effective_proxy_radius()
    // (physics/contacts.hpp) character-for-character.
    const float radius = effective_proxy_radius(bodies[slot].proxy_radius, material.proxy_radius);

    const float3 pos = bodies[slot].pos;

    // contacts.cpp:32 -- ONE walk of the program yields both the distance and
    // the gradient, which is what makes the two provably consistent.
    const SdfSample s = sdf_sample(world, pos);

    // contacts.cpp:36-47 -- STRICT `<`, spelled `!(a < b)` so a NaN distance
    // FAILS to be a contact and is skipped rather than falling through into a
    // normalize of a NaN gradient. An empty program takes the same exit:
    // kSdfEmptyDistance is not less than any sane radius.
    if (!(s.distance < radius)) {
        return;
    }

    // contacts.cpp:49-53 -- positive by the test above; phi may itself be
    // negative (origin inside the solid), and the arithmetic needs no case.
    const float depth = radius - s.distance;

    // contacts.cpp:55-67 -- the outward unit normal. Normalized
    // UNCONDITIONALLY (the heightfield's field is a Lipschitz bound with
    // |grad| <= 1, and smooth_union blends two unit gradients into a shorter
    // one), and `!(x > 0)` for the same NaN reason as above.
    const float grad_len = glm_length3(s.gradient);
    if (!(grad_len > 0.0f)) {
        return;
    }
    const float3 n = float3(s.gradient.x / grad_len, s.gradient.y / grad_len, s.gradient.z / grad_len);

    float3 vel = bodies[slot].vel;
    const float3 vel_in = vel;  // PHY-7: the change below is the IMU's

    // 1. NORMAL IMPULSE, only for an APPROACHING contact (contacts.cpp:94-110).
    //    `j_n` is MASS-NORMALIZED -- a speed, not an impulse in N s: against a
    //    static, infinitely massive world the effective mass is exactly m and
    //    it cancels identically.
    const float v_n = glm_dot3(vel, n);
    float j_n = 0.0f;
    if (v_n < 0.0f) {
        j_n = -(1.0f + e) * v_n;
        vel = vel + j_n * n;
    }

    // 2. COULOMB FRICTION on what remains AFTER step 1 (contacts.cpp:112-150).
    //    The cap mu * j_n is 0 when step 1 did not fire, so a separating
    //    contact has no friction -- out of the algebra, not out of a branch.
    //    The `min` against |v_t| is the STICTION FLOOR: without it a large mu
    //    would subtract more than the whole tangential velocity and reverse it.
    const float v_n_post = glm_dot3(vel, n);
    const float3 v_t = vel - v_n_post * n;
    const float v_t_len = glm_length3(v_t);
    if (v_t_len > 0.0f) {
        const float dv_t = glm_min(mu * j_n, v_t_len);
        vel = vel - dv_t * float3(v_t.x / v_t_len, v_t.y / v_t_len, v_t.z / v_t_len);
    }
    bodies[slot].vel = vel;
    contact_dv[slot] = float4(contact_dv[slot].xyz + (vel - vel_in), 0.0f);

    // 3. POSITIONAL CORRECTION (contacts.cpp:152-187). No velocity
    //    contribution -- that is what keeps it from adding energy. Spelled with
    //    max on the SUBTRACTION rather than a branch on depth > slop, so this
    //    is branch-free exactly as the CPU comment says the GPU mirror should
    //    be. Applied whether or not step 1 fired.
    const float correction = beta * glm_max(depth - slop, 0.0f);
    bodies[slot].pos = pos + n * correction;
}
