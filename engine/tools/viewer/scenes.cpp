// ---------------------------------------------------------------------------
// scenes.cpp -- the demo scene catalogue (drop, bounce, shower, gate).
//
// v1-FREE, deliberately (see bridge.cpp's file comment): only bridge.hpp,
// world/builder.hpp and glm are needed to describe a Scene as data. Building
// one touches no engine or GL state -- it just fills in WorldSetDesc +
// BodyPlacement values for bridge.cpp's Viewer to spawn and render.
// ---------------------------------------------------------------------------

#include "bridge.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/rng.hpp"
#include "vehicles/model_type.hpp"
#include "vehicles/quadrotor.hpp"
#include "world/builder.hpp"
#include "world/medium.hpp"

namespace spade::viewer {
namespace {

// Every scene builds its worlds through WorldBuilder::build(), which cannot
// fail here (the parameters below are all in range, by construction) -- but
// "cannot fail" is a claim worth checking. A failure means a bug in this
// file, not a runtime/user condition, hence throw rather than propagate a
// Result through a catalogue whose only fallible axis is "is this a known
// scene name" (see make_scene()'s std::optional).
[[nodiscard]] spade::WorldDesc unwrap_world(spade::Result<spade::WorldDesc> result, const char* what) {
    if (!result) {
        throw std::runtime_error(std::string("spade_viewer scene setup: ") + what + ": " +
                                  result.error().context);
    }
    return std::move(*result);
}

[[nodiscard]] spade::BodySpawn body_at(glm::vec3 pos, glm::vec3 vel = glm::vec3(0.0f)) {
    spade::BodySpawn spawn;
    spawn.pos = pos;
    spawn.vel = vel;
    spawn.mass = 1.0f;
    spawn.inv_inertia_diag = glm::vec3(1.0f);
    return spawn;
}

// Same "cannot fail here, but check anyway" posture as unwrap_world() above,
// for the model-type layer's own Result (Task 20's vehicle scenes).
[[nodiscard]] spade::vehicles::ModelType unwrap_model(spade::Result<spade::vehicles::ModelType> result,
                                                       const char* what) {
    if (!result) {
        throw std::runtime_error(std::string("spade_viewer scene setup: ") + what + ": " +
                                  result.error().context);
    }
    return std::move(*result);
}

// ---------------------------------------------------------------------------
// The demo airframe every vehicle scene below (hover/wind/flight/swarm) flies
// -- Task 20. ONE set of numbers, shared, so these four scenes are the SAME
// quadrotor under different conditions rather than four unrelated ones.
//
// A real, asymmetric-by-construction set of numbers (vehicles/quadrotor.hpp's
// own test-airframe note applies here too -- equal moments would let an
// axis-swap bug hide): mass 1 kg, three DISTINCT principal moments, 18 cm
// arms, 13 cm props -- a small racing-class quadrotor. hover_command() for
// these numbers is ~450 rad/s; every scene spawns INTO that trim
// (VehicleSpawn::rotor_omega -- sim/simulation.hpp's doc comment on why that
// avoids a rotor-spin-up transient) EXCEPT `flight`, whose takeoff starts
// deliberately below it (the brief's own suggestion).
//
// DRAG STAYS AT THE COM (QuadrotorParams::drag's default local_pos/local_orient,
// left untouched here) -- see scene_wind()'s comment for why an off-COM mount
// was deliberately NOT used to manufacture a "lean".
// ---------------------------------------------------------------------------
[[nodiscard]] spade::vehicles::QuadrotorParams demo_quadrotor_params(std::string name) {
    spade::vehicles::QuadrotorParams p;
    p.name = std::move(name);
    p.visual_ref = "meshes/quadrotor_demo";
    p.mass = 1.0f;
    p.inertia_diag = glm::vec3(0.018f, 0.032f, 0.024f);
    p.arm_length = 0.18f;
    p.rotor_height = 0.02f;
    p.proxy_radius = 0.2f;
    for (spade::vehicles::RotorParams& rotor : p.rotors) {
        rotor.tau = 0.02f;
        rotor.radius = 0.13f;
        rotor.thrust_coeff = 1.2e-5f;
        rotor.torque_coeff = 1.9e-7f;
    }
    p.drag.mode = spade::physics::drag_mode::quadratic;
    p.drag.area = 0.05f;
    p.drag.coeffs = glm::vec3(1.6f, 0.0f, 0.0f);
    p.imu.rate_divider = 1;  // ideal sensor (sigmas default to 0), every substep
    return p;
}

// ---------------------------------------------------------------------------
// drop -- a handful of spheres onto a plane.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_drop() {
    Scene s;
    s.name = "drop";
    s.ground_half_extent = 6.0f;
    s.ground_thickness = 0.4f;

    spade::WorldBuilder wb;
    wb.name("drop_ground")
        .environment(spade::Environment{})
        .capacities(spade::Capacities{6, 1, 1, 1})
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    const spade::WorldDesc world = unwrap_world(wb.build(), "drop: build ground world");

    spade::WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 1;
    instance.contacts.restitution_e = 0.35f;
    instance.contacts.friction_mu = 0.5f;
    instance.contacts.proxy_radius = 0.3f;
    instance.grid.cell_size = 0.8f;  // >= 2 * proxy_radius
    s.worlds.worlds.push_back(instance);

    const glm::vec4 color(0.30f, 0.65f, 0.95f, 1.0f);
    const std::array<glm::vec3, 6> starts = {{
        {-1.2f, 5.0f, -1.0f},
        {0.0f, 6.0f, 0.0f},
        {1.2f, 4.5f, 1.0f},
        {-0.8f, 7.0f, 1.2f},
        {0.9f, 5.5f, -1.3f},
        {0.0f, 8.0f, 0.6f},
    }};
    for (const glm::vec3& p : starts) {
        s.bodies.push_back(BodyPlacement{0, body_at(p), color});
    }

    s.camera_position = {0.0f, 4.0f, 11.0f};
    s.camera_target = {0.0f, 1.5f, 0.0f};
    return s;
}

// ---------------------------------------------------------------------------
// bounce -- a restitution ladder: 4 worlds side by side (own ground each,
// own ContactParams::restitution_e each), one ball per lane, colored by e.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_bounce() {
    Scene s;
    s.name = "bounce";

    constexpr int kLanes = 4;
    constexpr float kSpacing = 4.0f;
    s.ground_half_extent = kSpacing * 0.5f;  // adjacent lanes' slabs meet, not overlap
    s.ground_thickness = 0.3f;

    const std::array<float, kLanes> restitutions = {0.0f, 0.25f, 0.5f, 0.75f};
    const std::array<glm::vec4, kLanes> colors = {{
        {0.35f, 0.45f, 0.90f, 1.0f},
        {0.30f, 0.75f, 0.55f, 1.0f},
        {0.95f, 0.75f, 0.25f, 1.0f},
        {0.95f, 0.35f, 0.30f, 1.0f},
    }};

    for (int lane = 0; lane < kLanes; ++lane) {
        const float x = (static_cast<float>(lane) - (kLanes - 1) * 0.5f) * kSpacing;

        // A plane primitive is position-invariant along its own surface (the
        // normal here is +Y, so an X-translated pose leaves the y<=0
        // half-space, and therefore the PHYSICS, completely unchanged) -- but
        // bridge.cpp's plane->box render anchors the slab at the pose's own
        // position, and every lane needs its OWN slab centered on its own
        // ball. Without this pose, all four lanes' default (origin-centered)
        // slabs render stacked exactly on top of each other at x=0 instead of
        // spanning the lane's true position.
        spade::SdfPose ground_pose;
        ground_pose.position = glm::vec3(x, 0.0f, 0.0f);

        spade::WorldBuilder wb;
        wb.name("bounce_lane_" + std::to_string(lane))
            .environment(spade::Environment{})
            .capacities(spade::Capacities{1, 1, 1, 1})
            .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f, ground_pose);
        const spade::WorldDesc world = unwrap_world(wb.build(), "bounce: build lane world");

        spade::WorldInstanceDesc instance;
        instance.world = world;
        instance.seed = static_cast<uint64_t>(lane) + 1;
        instance.contacts.restitution_e = restitutions[static_cast<size_t>(lane)];
        instance.contacts.friction_mu = 0.3f;
        instance.contacts.proxy_radius = 0.35f;
        instance.grid.cell_size = 1.0f;  // >= 2 * proxy_radius
        s.worlds.worlds.push_back(instance);

        s.bodies.push_back(BodyPlacement{static_cast<uint32_t>(lane), body_at({x, 6.0f, 0.0f}),
                                          colors[static_cast<size_t>(lane)]});
    }

    s.camera_position = {0.0f, 4.5f, 18.0f};
    s.camera_target = {0.0f, 2.0f, 0.0f};
    return s;
}

// ---------------------------------------------------------------------------
// shower -- 1000 spheres raining into an SDF bowl.
//
// GEOMETRY CHOICE, AND WHY IT IS NOT subtract(box, sphere) (the brief's
// suggested example): a first pass built the bowl exactly that way -- a solid
// box with a sphere-shaped cavity subtracted out of its top -- which is
// correct PHYSICS (resolve_static_contacts() resolves against the true,
// analytic carved SDF, cavity and all) but renders wrong. bridge.cpp's
// visible_leaves() deliberately excludes a subtract()'s second operand from
// rendering (drawing the cutting sphere as its own opaque shape would occlude
// the very cavity it carves -- see that function's comment), which leaves
// only the OUTER box rendered as one solid, watertight mesh. That box has no
// actual hole cut into it (v1 cannot render true CSG), so once a sphere's
// position sinks below the box's rendered top face -- which is exactly what
// "settling in the bowl" means -- the box's opaque exterior hides it from
// every outside viewing angle. Verified by launching this scene: spheres
// visibly rain for the first ~2 s, then vanish entirely as they settle,
// which looks broken even though the physics underneath is exactly right.
//
// The fix here keeps the "SDF contact showcase" (five primitives, contacts
// resolved against a real multi-primitive union) but drops the occlusion:
// a thin floor plus four thin walls, all unioned, forming an OPEN-TOP tub.
// Nothing is subtracted, so every primitive stays in visible_leaves() and
// nothing hides what is resting inside -- the spheres settle in full view on
// the floor, framed by walls that are themselves thin enough not to occlude.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_shower() {
    Scene s;
    s.name = "shower";

    spade::SdfPose floor_pose;
    floor_pose.position = glm::vec3(0.0f, -0.3f, 0.0f);
    spade::SdfPose wall_left;
    wall_left.position = glm::vec3(-4.15f, 1.2f, 0.0f);
    spade::SdfPose wall_right;
    wall_right.position = glm::vec3(4.15f, 1.2f, 0.0f);
    spade::SdfPose wall_back;
    wall_back.position = glm::vec3(0.0f, 1.2f, -4.15f);
    spade::SdfPose wall_front;
    wall_front.position = glm::vec3(0.0f, 1.2f, 4.15f);

    spade::WorldBuilder wb;
    wb.name("shower_bowl")
        .environment(spade::Environment{})
        .capacities(spade::Capacities{1000, 1, 1, 1})
        .box(glm::vec3(4.0f, 0.3f, 4.0f), floor_pose)
        .box(glm::vec3(0.15f, 1.5f, 4.15f), wall_left)
        .union_()
        .box(glm::vec3(0.15f, 1.5f, 4.15f), wall_right)
        .union_()
        .box(glm::vec3(4.15f, 1.5f, 0.15f), wall_back)
        .union_()
        .box(glm::vec3(4.15f, 1.5f, 0.15f), wall_front)
        .union_();
    const spade::WorldDesc world = unwrap_world(wb.build(), "shower: build bowl world");

    spade::WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 7;
    instance.contacts.restitution_e = 0.15f;
    instance.contacts.friction_mu = 0.6f;
    instance.contacts.proxy_radius = 0.12f;
    instance.grid.cell_size = 0.3f;  // >= 2 * proxy_radius
    s.worlds.worlds.push_back(instance);

    // A 10x10x10 grid, tall in Y: every sphere starts at a distinct height,
    // so the fixed-at-t=0 spawn (spec: "spawn everything at t=0") still reads
    // as a staggered rain rather than one simultaneous splash -- the same
    // "arrive at different times" effect the brief describes for scripted
    // velocities, achieved here by height instead.
    constexpr int kPerAxis = 10;
    constexpr float kXZSpacing = 0.42f;
    constexpr float kYSpacing = 2.0f;
    constexpr float kYBase = 6.0f;
    const glm::vec4 color(0.55f, 0.80f, 1.0f, 1.0f);

    // STANDING RULE (user amendment A2, applies to every future scene with a
    // large body count, not just this one): a perfectly regular lattice --
    // every column exactly aligned in X/Z -- reads as a rendering artifact
    // (bodies visibly stacked column-on-column) rather than a physical rain,
    // UNLESS exact stacking is a deliberate physics-fault test (this scene is
    // not one). The fix is a small per-body X/Z perturbation, magnitude small
    // relative to kXZSpacing so the tub/rain composition is unchanged and
    // every body still starts well inside the tub walls (checked below).
    //
    // The jitter MUST be, and is, DETERMINISTIC: it is drawn from a
    // core/rng.hpp Stream seeded by THIS scene's own world seed
    // (`instance.seed`, set above) under a dedicated domain tag, one Stream
    // per body index -- the same derivation discipline every other
    // stochastic draw in the engine uses (core/rng.hpp's file comment).
    // NOT std::random_device, NOT wall-clock, NOT global rand(): the same
    // launch produces the exact same jittered scene every time, which is
    // what makes this scene usable as a visual regression reference at all.
    constexpr float kJitterFraction = 0.18f;  // of kXZSpacing; brief's range is 10-25%
    constexpr float kJitterMagnitude = kJitterFraction * kXZSpacing;

    uint64_t body_index = 0;
    for (int iy = 0; iy < kPerAxis; ++iy) {
        for (int iz = 0; iz < kPerAxis; ++iz) {
            for (int ix = 0; ix < kPerAxis; ++ix) {
                rng::Stream jitter_stream = rng::make_stream(instance.seed, "viewer.shower.spawn_jitter", body_index);
                const float jitter_x = (jitter_stream.next_float() * 2.0f - 1.0f) * kJitterMagnitude;
                const float jitter_z = (jitter_stream.next_float() * 2.0f - 1.0f) * kJitterMagnitude;
                ++body_index;

                const float x = (static_cast<float>(ix) - 4.5f) * kXZSpacing + jitter_x;
                const float z = (static_cast<float>(iz) - 4.5f) * kXZSpacing + jitter_z;
                const float y = kYBase + static_cast<float>(iy) * kYSpacing;
                s.bodies.push_back(BodyPlacement{0, body_at({x, y, z}), color});
            }
        }
    }

    s.camera_position = {0.0f, 11.0f, 19.0f};
    s.camera_target = {0.0f, 3.0f, 0.0f};
    return s;
}

// The torus+box gate assembly (world/builder.hpp's own doc-comment example:
// ".torus(...).box(post, left).union_().box(post, right).union_()"), factored
// out of scene_gate() so scene_flight() (Task 20) builds the IDENTICAL
// geometry -- a body's flight path through the ring is only meaningful if the
// ring is the same ring gate's own tests measure against.
//
// `capacities` is the caller's own -- gate throws five balls and needs no
// force elements or sensors; flight flies one quadrotor and needs five force
// elements and one sensor (vehicles/model_type.hpp's ModelType::
// force_element_count()) -- so it is the one parameter left open.
[[nodiscard]] spade::WorldDesc build_gate_world(const char* name, const spade::Capacities& capacities) {
    spade::SdfPose ring_pose;
    ring_pose.position = glm::vec3(0.0f, 1.8f, 0.0f);
    // Default torus hole axis is local +Y (world/sdf.hpp); rotate it onto
    // world +Z so a body flying along Z passes through the ring.
    //
    // SPELLED AS THE EXACT QUATERNION (sqrt(1/2), sqrt(1/2), 0, 0), NOT
    // glm::angleAxis(radians(90), +X) (S5 T9 ticket A -- unifies this with
    // tests/test_world_file.cpp's gate_world(), which committed
    // tests/golden/worlds/gate.world.yaml with this exact spelling and left
    // this call site as a ticketed follow-up; see that file's
    // GateWorldHasTheGateAssemblysStructure comment for the original
    // divergence). std::sqrt is IEEE-754-mandated correctly rounded;
    // std::sin/std::cos (what glm::angleAxis calls) are libm and merely
    // LIKELY to agree across compilers -- on this box the two spellings
    // happened to land bit-identical, but "happened to" is not a contract.
    //
    // THIS IS NOT A DETERMINISM FIX, and it would be dishonest to imply
    // otherwise: spade_viewer is a SPADE_BUILD_V1-gated TOOL, not a library
    // linked into spade_tests, and this function's output is render-only --
    // it never touches registered state or the digest (global-constraints.md's
    // bit-portability rule governs what feeds those, and this is not on that
    // path). What this spelling change buys is ONE SOURCE OF TRUTH for "what
    // does the gate's ring rotation mean": before this change there were two
    // independent transcriptions of the same authored pose (this file's and
    // gate_world()'s), free to drift apart the moment either one was touched
    // without the other; after it, both read the identical quaternion
    // literal, so there is nothing left TO drift.
    const float h = std::sqrt(0.5f);
    ring_pose.rotation = glm::quat(h, h, 0.0f, 0.0f);

    const glm::vec3 post_half(0.15f, 1.5f, 0.15f);
    spade::SdfPose left_post;
    left_post.position = glm::vec3(-1.8f, 1.5f, 0.0f);
    spade::SdfPose right_post;
    right_post.position = glm::vec3(1.8f, 1.5f, 0.0f);

    spade::WorldBuilder wb;
    wb.name(name)
        .environment(spade::Environment{})
        .capacities(capacities)
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f)
        .torus(1.5f, 0.15f, ring_pose)
        .box(post_half, left_post)
        .union_()
        .box(post_half, right_post)
        .union_()
        .union_();
    return unwrap_world(wb.build(), "build_gate_world");
}

// ---------------------------------------------------------------------------
// gate -- spheres thrown through a torus+box gate (SDF contact showcase).
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_gate() {
    Scene s;
    s.name = "gate";
    s.ground_half_extent = 16.0f;
    s.ground_thickness = 0.4f;

    const spade::WorldDesc world = build_gate_world("gate_world", spade::Capacities{5, 1, 1, 1});

    spade::WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 3;
    instance.contacts.restitution_e = 0.4f;
    instance.contacts.friction_mu = 0.3f;
    instance.contacts.proxy_radius = 0.25f;
    instance.grid.cell_size = 0.6f;  // >= 2 * proxy_radius
    s.worlds.worlds.push_back(instance);

    // Spawned as LOBBED ARCS aimed through the ring, so despite all being
    // spawned at t=0 (fixed body count) they arrive at the gate at staggered
    // real times -- the brief's "thrown later" effect via initial velocity
    // rather than a scripted per-tick impulse.
    //
    // A first pass threw these nearly horizontally (small vy) from y ~ ring
    // height -- launching them straight at the ring's clear opening in a
    // vacuum. Verified by launching the scene: gravity pulls a body down
    // ~4.9*t^2 over its flight, so by the time each one reaches z=0 it had
    // already dropped to the GROUND (short of the gate, still well behind
    // z=0), then rolled/slid the rest of the way under friction -- balls
    // piled up at the posts' base instead of flying through the ring.
    //
    // Fixed by giving each throw an actual BALLISTIC ARC: for a throw at
    // (x0, y0=1.8, z0) with horizontal speed vz, time-to-gate is
    // t_gate = -z0 / vz, and launching with vy = g * t_gate (g = 9.80665,
    // Environment{}'s default) makes y(t_gate) = y0 + vy*t_gate - 0.5*g*t_gate^2
    // = y0 exactly -- a symmetric lob that returns to ring height precisely
    // when it reaches the gate, well inside the ring's ~1.35 m clear radius.
    struct Throw {
        glm::vec3 start;
        glm::vec3 vel;
    };
    constexpr float kGravity = 9.80665f;
    constexpr float kRingHeight = 1.8f;
    // {x0, z0, vz} -- vy is derived below so every throw arcs back to
    // kRingHeight exactly as it crosses z=0.
    const std::array<std::array<float, 3>, 5> lobs = {{
        {-0.30f, -6.0f, 6.0f},
        {0.20f, -6.5f, 5.0f},
        {0.00f, -5.0f, 6.25f},
        {-0.25f, -8.0f, 5.0f},
        {0.30f, -7.0f, 7.0f},
    }};
    std::array<Throw, 5> throws{};
    for (size_t i = 0; i < lobs.size(); ++i) {
        const float x0 = lobs[i][0];
        const float z0 = lobs[i][1];
        const float vz = lobs[i][2];
        const float t_gate = -z0 / vz;
        const float vy = kGravity * t_gate;
        throws[i] = Throw{{x0, kRingHeight, z0}, {0.0f, vy, vz}};
    }
    const glm::vec4 color(0.95f, 0.55f, 0.25f, 1.0f);
    for (const Throw& t : throws) {
        s.bodies.push_back(BodyPlacement{0, body_at(t.start, t.vel), color});
    }

    s.camera_position = {6.0f, 3.5f, -11.0f};
    s.camera_target = {0.0f, 1.8f, 3.0f};
    return s;
}

// ---------------------------------------------------------------------------
// hover -- the quadrotor at its hover command, camera orbiting slowly around
// it (Task 20).
//
// Spawned WITH rotor_omega == hover_command(params): sim/simulation.hpp's
// VehicleSpawn doc comment is explicit that this spawns "IN TRIM" -- both the
// live shaft speed and the standing command start at the exact hover value,
// so there is no rotor-spin-up transient to watch (or wait out) and the very
// first frame already shows a level, holding hover. vehicles/quadrotor.hpp's
// own derivation (and test_quadrotor.cpp's ten-second hold) is what makes
// "holding" an honest word here rather than an aspiration: for a SYMMETRIC
// airframe at this trim, the net moment is EXACTLY zero every substep (equal
// terms cancel bit-for-bit, ROLL/PITCH/YAW all zero) and the residual
// position drift is on the order of microns over ten seconds -- CHECKPOINT
// №2 should see a rock-steady hover, not a barely-controlled wobble.
//
// WHAT SHOULD BE VISIBLE: a quadrotor hanging level at y = 3 m over the
// ground plane, the FPS counter printing to the console once a second (A1,
// inherited from bridge.cpp's run() loop unchanged), and the camera slowly
// circling it. No turbulence (TurbulenceLevel::none): this scene is the
// CONTROL a viewer compares `wind` against.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_hover() {
    Scene s;
    s.name = "hover";
    s.ground_half_extent = 6.0f;
    s.ground_thickness = 0.3f;

    spade::WorldBuilder wb;
    wb.name("hover_ground")
        .environment(spade::Environment{})
        .capacities(spade::Capacities{1, 5, 1, 1})
        .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f);
    const spade::WorldDesc world = unwrap_world(wb.build(), "hover: build ground world");

    spade::WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 11;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.2f;
    instance.contacts.friction_mu = 0.5f;
    instance.contacts.proxy_radius = 0.2f;
    instance.grid.cell_size = 0.5f;  // >= 2 * proxy_radius
    s.worlds.worlds.push_back(instance);

    const spade::vehicles::QuadrotorParams params = demo_quadrotor_params("hover_quad");
    s.models.push_back(unwrap_model(spade::vehicles::make_quadrotor(params), "hover: make_quadrotor"));

    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 3.0f, 0.0f);
    where.rotor_omega = spade::vehicles::hover_command(params);  // IN TRIM -- see the comment above
    s.vehicles.push_back(VehiclePlacement{0, 0, where, glm::vec4(0.35f, 0.75f, 0.95f, 1.0f)});

    s.camera_orbit = true;
    s.camera_orbit_target = where.pos;
    s.camera_orbit_radius = 5.0f;
    s.camera_orbit_height = 1.6f;
    s.camera_orbit_angular_rate = 0.35f;
    // Matches the orbit's own t=0 position (angle 0 -> target + (radius, height, 0))
    // so the very first frame is not a jump-cut from wherever a plain static
    // placement would have put the camera.
    s.camera_position = where.pos + glm::vec3(s.camera_orbit_radius, s.camera_orbit_height, 0.0f);
    s.camera_target = where.pos;
    return s;
}

// ---------------------------------------------------------------------------
// wind -- the same hover, in Dryden "moderate" turbulence (TurbulenceLevel::
// moderate == level 2 of world/medium.hpp's four discrete levels 0=none /
// 1=light / 2=moderate / 3=severe) -- Task 20's brief "level 2".
//
// READ THIS BEFORE WATCHING THE SCENE. A first draft of this comment guessed
// at the mechanism instead of measuring it, and guessed wrong (predicted a
// gentle sideways-only drift from body drag); what is written below is
// checked against a headless replay of this exact configuration across 30
// rng seeds (see the task report), not assumed.
//
//   * NO LEAN, EVER -- VERIFIED, NOT JUST DERIVED. This scene keeps the drag
//     element at the airframe's COM (demo_quadrotor_params() leaves
//     QuadrotorParams::drag's local_pos/local_orient at their identity
//     defaults), so a force there produces zero torque
//     (torque_acc += cross(local_pos, F_body), cross(0, F) == 0), and
//     vehicles/quadrotor.hpp section 2 shows a symmetric collective's rotor
//     moment is exactly zero regardless of attitude too -- so nothing in this
//     vehicle's force path can tilt it. This engine also has NO passive
//     attitude-RESTORING torque (no aerodynamic damping model, no gravity
//     torque -- gravity always acts at the COM), so a design that DID
//     introduce a torque here would spin up WITHOUT BOUND rather than settle
//     into a lean, which is worse, not more honest. The 30-seed sweep's
//     measured max attitude deviation is 0.000 degrees, every seed, which is
//     what "by construction" is a claim about a mechanism (not a measurement)
//     is FOR -- here it is both.
//   * WHAT ACTUALLY DOMINATES: ALTITUDE, THROUGH THE ROTORS -- NOT DRAG.
//     vehicles/rotor.cpp's apply_rotors() samples the Medium AT EACH ROTOR'S
//     OWN WORLD POSITION and folds the wind into v_axial (spec S6: "v_axial =
//     axis . (v_body + w_b x r - medium.wind)") BEFORE the momentum-theory
//     inflow correction scales thrust -- so a VERTICAL gust component reads
//     to the rotor exactly like a vertical velocity the vehicle does not
//     actually have, and the correction responds to it as such. For THIS
//     demo airframe the hover induced velocity is v_h = sqrt(T/(2 rho A)) ~=
//     4.3 m/s (small props, `demo_quadrotor_params()`'s 13 cm radius), and
//     moderate turbulence's vertical sigma is 1.5433 m/s (world/medium.hpp's
//     own table) -- roughly a third of v_h, which the inflow curve's
//     near-hover slope (lambda(x) ~= 1 - x/2) turns into a THRUST swing large
//     enough to move the vehicle several METRES vertically over a few
//     seconds. Horizontal gusts move it too, but through body drag alone
//     (bounded, and an order of magnitude gentler for this airframe) --
//     rotors stay level (previous bullet), so a horizontal gust never reaches
//     v_axial at all.
//   * THIS IS NOT A BUG; IT IS THE ABSENCE OF A CONTROLLER, CORRECTLY. A real
//     autopilot counters exactly this with an altitude hold loop; v2's
//     vehicle stack is open-loop rotor commands only (vehicles/quadrotor.hpp:
//     "turning a controller's ... into four shaft speeds is a mixer's job,
//     one layer up" -- and there is no mixer here, scripted or otherwise).
//     So a multi-metre excursion for several seconds, with attitude never
//     leaving level, is the correct uncontrolled response, not a
//     misconfiguration.
//   * AND IT DOES SETTLE, even without a controller: the same inflow
//     correction that reads the gust as a false climb rate ALSO damps a
//     REAL one (test_quadrotor.cpp's ~1 s time constant, 2 v_h/g), so the
//     climb this seed (23, fixed and reproducible) produces peaks around
//     t ~= 3.4 s and eases back down afterward rather than climbing forever.
//
// CAMERA: STATIC, not orbiting (the coordinator's documented fallback for
// exactly this situation -- v1's frozen free-fly controls remain available).
// An orbit anchored on the spawn point would lose the vehicle as it climbs;
// a fixed, pulled-back frame with the excursion's measured range in view does
// not.
//
// So: CHECKPOINT №2 should see a level (never tilting) quadrotor visibly
// climb several metres over the first few seconds and begin easing back --
// moderate turbulence's real effect on an OPEN-LOOP hover, not a bug.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_wind() {
    Scene s = scene_hover();
    s.name = "wind";
    s.worlds.worlds[0].seed = 23;  // fixed and reproducible -- see the comment above
    s.worlds.worlds[0].turbulence = spade::dryden_params(spade::TurbulenceLevel::moderate);
    s.ground_half_extent = 10.0f;  // room for the measured excursion, not just the spawn point

    s.camera_orbit = false;  // static fallback -- see the comment above
    s.camera_position = {8.0f, 6.5f, 10.0f};
    s.camera_target = {0.0f, 5.0f, 0.0f};  // centered on the measured y in [3, 8] range
    return s;
}

namespace flight_profile {

// The viewer's fixed step rate (setup.hpp's kStepDtNs/kSubsteps -- 4 ms
// step, 4 substeps -- is 250 steps/s). It is duplicated here as a DOCUMENTED
// literal: if the viewer's step rate ever changes, this profile's tick
// boundaries must move with it, and this comment is where a reader finds out
// why they look like they did.
constexpr uint64_t kTicksPerSecond = 250;

// --- Phase A: takeoff -- a brief collective boost, then hold hover --------
// The vertical channel has REAL passive damping the attitude channel does
// not: rotor.hpp's momentum-theory inflow correction is
// lambda(v_axial/v_h) ~= 1 - v_axial/(2 v_h) near v_axial = 0, so a nonzero
// climb rate produces a restoring thrust reduction with time constant
// 2 v_h/g (~1 s for this airframe -- see test_quadrotor.cpp's hover-bound
// derivation, which measures exactly this constant). So a short boost above
// hover, followed by an immediate return to EXACT hover collective, does not
// need a matching deceleration leg the way the attitude pulse below does --
// the climb rate the boost leaves behind decays on its own once collective
// returns to trim. Verified against a headless replay of this exact
// profile before being wired into the viewer (see the task report).
constexpr float kBoostFraction = 1.10f;                             // collective, x hover
constexpr uint64_t kBoostEndTick = (3 * kTicksPerSecond) / 5;       // 0.6 s

// --- Phase B: figure-forward -- a symmetric roll-axis (M_x) pulse ---------
// quadrotor.hpp section 2: M_x = L*(T_3 - T_1), the 1/3 (+-Z arm) pair. With
// the airframe level (identity spawn orientation) a positive M_x rotates the
// rotor thrust axis (body +Y) toward world +Z -- exactly the direction this
// scene needs to carry the vehicle from its spawn behind the gate (negative
// Z) THROUGH the ring at z = 0. Rotor 1 (the +Z arm) is commanded slightly
// BELOW baseline and rotor 3 (the -Z arm) slightly ABOVE it for one short
// leg, then the two are swapped for an equal second leg -- a "there and
// back" pulse that returns the ANGULAR VELOCITY to zero (this engine has no
// passive attitude damping -- see scene_wind()'s comment -- so an
// un-mirrored pulse would spin up without bound) while leaving a small
// RESIDUAL TILT ANGLE (a rest-to-rest bang-bang profile's well known
// property: velocity returns to zero, position does not). That residual
// tilt is not cleaned up on purpose: a still-tilted thrust vector keeps
// nudging the vehicle gently forward AND costs it a `1 - cos(tilt)` sliver
// of vertical support, which is precisely this scene's "figure-forward ->
// descend" without a third scripted phase.
constexpr uint64_t kPulseLegStartTick = 3 * kTicksPerSecond;       // 3.0 s: vertical channel has settled
constexpr uint64_t kPulseLegMidTick = kPulseLegStartTick + 62;     // +0.248 s
constexpr uint64_t kPulseLegEndTick = kPulseLegMidTick + 62;       // +0.248 s
constexpr float kPulseFraction = 0.045f;                           // differential, x hover

// --- Phase C: descend -- collective trimmed a little below hover ----------
// Applied well after the pulse (the vehicle should already be past the gate
// by real-world flight time at this airspeed and pulse magnitude -- checked
// headlessly, see the task report) so the visible order stays
// takeoff -> forward -> descend and does not overlap the gate crossing with
// an intentional loss of altitude.
constexpr uint64_t kDescendStartTick = 8 * kTicksPerSecond;   // 8.0 s -- the gate is crossed by ~7.86 s
constexpr float kDescendFraction = 0.97f;                      // collective, x hover

}  // namespace flight_profile

// The airframe scene_flight() flies, factored out so the scripted command
// hook below can derive the SAME hover_command() the scene spawned into
// without a shared mutable global -- QuadrotorParams is a plain value, so
// calling this twice produces byte-identical numbers by construction.
[[nodiscard]] spade::vehicles::QuadrotorParams flight_quadrotor_params() {
    return demo_quadrotor_params("flight_quad");
}

// The flight scene's scripted per-tick command hook (bridge.hpp's
// VehicleCommandHook) -- flight_profile's three phases, translated into four
// shaft-speed commands. A PURE function of (tick): no captures, no state, no
// randomness, no clock -- see bridge.hpp's doc comment on why that is the
// whole point of this being a bare function.
void flight_command_hook(spade::Simulation& sim, uint64_t tick, std::span<const spade::VehicleRef> vehicles) {
    if (vehicles.empty()) return;  // defensive; scene_flight() always spawns exactly one

    using namespace flight_profile;
    const float hover = spade::vehicles::hover_command(flight_quadrotor_params());

    float collective = hover;
    if (tick < kBoostEndTick) {
        collective = hover * kBoostFraction;
    } else if (tick >= kDescendStartTick) {
        collective = hover * kDescendFraction;
    }

    float w1 = collective;  // +Z arm
    float w3 = collective;  // -Z arm
    const float pulse = hover * kPulseFraction;
    if (tick >= kPulseLegStartTick && tick < kPulseLegMidTick) {
        w1 -= pulse;
        w3 += pulse;  // leg 1: M_x > 0, tilts +Y toward +Z
    } else if (tick >= kPulseLegMidTick && tick < kPulseLegEndTick) {
        w1 += pulse;
        w3 -= pulse;  // leg 2: the mirrored pulse, nulls the angular velocity leg 1 left behind
    }

    const std::array<float, 4> commands{collective, w1, collective, w3};
    if (const spade::Result<void> r = sim.set_rotor_commands(vehicles[0], commands); !r) {
        throw std::runtime_error(std::string("spade_viewer scene setup: flight: set_rotor_commands: ") +
                                  r.error().context);
    }
}

// ---------------------------------------------------------------------------
// flight -- a scripted command profile through the gate world: takeoff ->
// figure-forward -> descend (Task 20). See flight_profile's and
// flight_command_hook()'s comments above for the mechanics of each phase.
//
// SPAWNED BELOW TRIM, DELIBERATELY (the brief's own suggestion, and the
// opposite of hover/wind/swarm's in-trim spawns): rotor_omega starts at 85%
// of hover, just above the ground, so the first ~1 s is a visible takeoff --
// not the "already flying" look every other vehicle scene deliberately has.
//
// WHAT SHOULD BE VISIBLE: the quadrotor lifts off the ground near
// (0, 0.3, -8), climbs to roughly gate height, glides forward, passes through
// the torus ring around z = 0, and noses gently downward afterward. The IMU
// caveat (sensors/imu.hpp section 5, restated in this task's context) does
// not bite here: the vehicle is airborne and under thrust for the whole
// scene, never resting on the ground SDF, so there is no "reads ~0 instead of
// +g" segment to misread as a bug.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_flight() {
    Scene s;
    s.name = "flight";
    s.ground_half_extent = 16.0f;
    s.ground_thickness = 0.4f;

    const spade::WorldDesc world = build_gate_world("flight_gate_world", spade::Capacities{1, 5, 1, 1});

    spade::WorldInstanceDesc instance;
    instance.world = world;
    instance.seed = 23;
    instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
    instance.contacts.restitution_e = 0.3f;
    instance.contacts.friction_mu = 0.5f;
    instance.contacts.proxy_radius = 0.2f;
    instance.grid.cell_size = 0.6f;
    s.worlds.worlds.push_back(instance);

    const spade::vehicles::QuadrotorParams params = flight_quadrotor_params();
    s.models.push_back(unwrap_model(spade::vehicles::make_quadrotor(params), "flight: make_quadrotor"));

    spade::VehicleSpawn where;
    where.pos = glm::vec3(0.0f, 0.3f, -8.0f);
    where.rotor_omega = spade::vehicles::hover_command(params) * 0.85f;  // below trim -- a real takeoff
    s.vehicles.push_back(VehiclePlacement{0, 0, where, glm::vec4(0.95f, 0.55f, 0.25f, 1.0f)});
    s.command_hook = &flight_command_hook;

    s.camera_position = {6.0f, 3.5f, -11.0f};  // gate's own camera -- same ring, same vantage
    s.camera_target = {0.0f, 1.8f, 3.0f};
    return s;
}

// ---------------------------------------------------------------------------
// swarm -- a 4-world set, one hovering quadrotor per world, stepped in ONE
// batched call and rendered side by side (Task 20's world-batching demo).
//
// EACH LANE IS ITS OWN WORLD, not four vehicles in one world: that is the
// property being demonstrated (sim/world_set.hpp's WorldSetLayout::
// uniform_dynamic_params -- four worlds sharing one ContactParams/GridParams
// take the CollisionDynamic pass's single sorted-grid sweep over the whole
// set, D8's "one dispatch steps N worlds", rather than one sweep per world).
// Four separate, non-interacting hovers is the simplest scene that actually
// exercises it -- and the four worlds' quadrotors staying rock-steady (each
// is `hover` again, just x4) is itself part of the proof: nothing about
// batching perturbs a world's own trajectory (that equivalence is also what
// test_determinism.cpp's batching-invariance tests pin numerically).
//
// THE GRID OFFSET IS LAYOUT, NOT STACKING (user amendment A2 names this scene
// by name as the exception: "the swarm's per-world grid offsets are layout").
// Every lane's ground plane AND vehicle spawn share the SAME per-lane offset
// (mirroring scene_bounce()'s per-lane pattern) so the four worlds -- each an
// independent coordinate space to the simulation -- render apart rather than
// stacked on top of each other in the one shared viewport. No jitter is
// added: with exactly one vehicle per world there is nothing that would
// otherwise land exactly stacked.
// ---------------------------------------------------------------------------
[[nodiscard]] Scene scene_swarm() {
    Scene s;
    s.name = "swarm";

    constexpr int kLanes = 4;
    constexpr float kSpacing = 6.0f;
    s.ground_half_extent = 2.5f;  // < kSpacing/2, so adjacent lanes' slabs never touch
    s.ground_thickness = 0.3f;

    const std::array<glm::vec2, kLanes> offsets = {{
        {-kSpacing * 0.5f, -kSpacing * 0.5f},
        {kSpacing * 0.5f, -kSpacing * 0.5f},
        {-kSpacing * 0.5f, kSpacing * 0.5f},
        {kSpacing * 0.5f, kSpacing * 0.5f},
    }};
    const std::array<glm::vec4, kLanes> colors = {{
        {0.35f, 0.75f, 0.95f, 1.0f},
        {0.35f, 0.90f, 0.55f, 1.0f},
        {0.95f, 0.75f, 0.30f, 1.0f},
        {0.90f, 0.40f, 0.85f, 1.0f},
    }};

    const spade::vehicles::QuadrotorParams params = demo_quadrotor_params("swarm_quad");
    const spade::vehicles::ModelType model = unwrap_model(spade::vehicles::make_quadrotor(params),
                                                           "swarm: make_quadrotor");
    s.models.push_back(model);
    const float hover = spade::vehicles::hover_command(params);

    for (int lane = 0; lane < kLanes; ++lane) {
        const float x = offsets[static_cast<size_t>(lane)].x;
        const float z = offsets[static_cast<size_t>(lane)].y;

        // Same lane-local pose reasoning as scene_bounce(): the ground plane
        // is physically position-invariant along its own surface, but every
        // lane needs its own RENDERED slab centered on its own hover point.
        spade::SdfPose ground_pose;
        ground_pose.position = glm::vec3(x, 0.0f, z);

        spade::WorldBuilder wb;
        wb.name("swarm_lane_" + std::to_string(lane))
            .environment(spade::Environment{})
            .capacities(spade::Capacities{1, 5, 1, 1})
            .plane(glm::vec3(0.0f, 1.0f, 0.0f), 0.0f, ground_pose);
        const spade::WorldDesc world = unwrap_world(wb.build(), "swarm: build lane world");

        spade::WorldInstanceDesc instance;
        instance.world = world;
        instance.seed = static_cast<uint64_t>(lane) + 101;
        instance.turbulence = spade::dryden_params(spade::TurbulenceLevel::none);
        instance.contacts.restitution_e = 0.2f;
        instance.contacts.friction_mu = 0.5f;
        instance.contacts.proxy_radius = 0.2f;
        instance.grid.cell_size = 0.5f;
        s.worlds.worlds.push_back(instance);

        spade::VehicleSpawn where;
        where.pos = glm::vec3(x, 3.0f, z);
        where.rotor_omega = hover;  // IN TRIM, same as hover -- see that scene's comment
        s.vehicles.push_back(
            VehiclePlacement{static_cast<uint32_t>(lane), 0, where, colors[static_cast<size_t>(lane)]});
    }

    s.camera_position = {0.0f, 12.0f, 15.0f};
    s.camera_target = {0.0f, 2.0f, 0.0f};
    return s;
}

}  // namespace

std::optional<Scene> make_scene(std::string_view name) {
    if (name == "drop") return scene_drop();
    if (name == "bounce") return scene_bounce();
    if (name == "shower") return scene_shower();
    if (name == "gate") return scene_gate();
    if (name == "hover") return scene_hover();
    if (name == "wind") return scene_wind();
    if (name == "flight") return scene_flight();
    if (name == "swarm") return scene_swarm();
    return std::nullopt;
}

const std::vector<std::string>& scene_names() {
    static const std::vector<std::string> kNames = {"drop",  "bounce", "shower", "gate",
                                                     "hover", "wind",   "flight", "swarm"};
    return kNames;
}

}  // namespace spade::viewer
