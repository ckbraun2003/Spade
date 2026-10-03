#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "core/error.hpp"
#include "physics/forces.hpp"  // drag_mode:: -- the element mode a DragBodyDesc names

// ===========================================================================
// ModelType -- the MODEL-TYPE LAYER (engine design spec S6 "Vehicles -- the
// model-type layer"; D4).
//
// A ModelType is the ENGINE'S ASSET CLASS ONE TIER ABOVE SHAPES AND MESHES: a
// complete, reusable description of what a vehicle IS, from which the
// Simulation can manufacture instances. Spec S6 names its contents and this
// header is that list, one field at a time:
//
//     body template . proxy . element descs . sensor mounts . visual refs
//     . param schema id
//
// ---------------------------------------------------------------------------
// WHAT A ModelType IS NOT, STATED FIRST BECAUSE IT IS THE WHOLE DESIGN
//
// IT IS A DESCRIPTION AND HOLDS NO STATE. No arena, no slot, no handle, no
// pointer into a Simulation. It is a plain value: copyable, comparable by
// inspection, cheap to build in a test and (from S5) loadable from a file. The
// instance -- the body slot, the rotor rows, the sensor rows and their live
// shaft speeds -- lives entirely in the Simulation's arenas, which is what
// keeps every byte of a vehicle inside the snapshot walk
// (state/registry.hpp's "no unregistered state" invariant).
//
// THE CONSEQUENCE, AND IT IS LOAD BEARING FOR REPLAY: a Simulation's model
// REGISTRY is CONFIGURATION, in exactly the sense sim/world_set.hpp's material
// and turbulence records are configuration. It is not in the snapshot, and
// restore() does not rebuild it. A caller replaying a blob into a fresh
// Simulation must register the same models in the same order, the same way it
// must create that Simulation from the same WorldSetDesc. Simulation::restore
// says the same thing about the rest of the configuration; this is one more
// item on that list, and it is stated here as well as there.
//
// ---------------------------------------------------------------------------
// WHY IT LIVES UNDER vehicles/ AND NOT UNDER sim/
//
// Spec S3's tree comment is the rule: "nothing vehicle-specific below here",
// i.e. the dependency arrow points DOWN from vehicles into physics/world/state
// and never back. A ModelType is the most vehicle-specific object in the
// engine, so it lives at the top of that arrow -- and, crucially, it does NOT
// depend on sim/. The Simulation depends on IT.
//
// THAT DIRECTION COSTS TWO SMALL DUPLICATIONS, and they are deliberate rather
// than overlooked. DragBodyDesc and ImuMountDesc below mirror
// sim/simulation.hpp's DragElementSpawn and ImuSensorSpawn field for field.
// The alternative -- describing a model in terms of the Simulation's spawn
// records -- would make vehicles/ depend on sim/ and invert the arrow for the
// sake of ten fields. The mapping from desc to spawn record therefore lives in
// exactly one place (Simulation's model spawn), and these two structs are the
// authoring side of it. If a third consumer ever appears, the answer is to
// move the spawn records DOWN into physics/forces.hpp and sensors/imu.hpp
// beside the rows they describe, not to move these up.
//
// ---------------------------------------------------------------------------
// ONE FIELD IS CONSUMED AS OF D-S6-2; ONE STILL IS NOT, and saying so for both
// is the point of carrying them:
//
//   * `proxy_radius`. Through S5, v2's CollisionStatic took its sphere-proxy
//     radius only from the WORLD's physics::ContactParams (one radius for
//     every body in a world), so a per-MODEL radius had nothing to read it.
//     D-S6-2 closed that: Simulation::spawn(world, ModelTypeId, VehicleSpawn)
//     now writes this value into the spawned body's BodyState::proxy_radius
//     (state/layout.hpp), and both contact passes read it there,
//     per body, via physics::effective_proxy_radius() (physics/contacts.hpp)
//     -- falling back to the world's default when a model leaves this field at
//     its own 0.0f default. It was always validated at authoring time (a
//     nonsensical value is rejected before it matters), which is what made it
//     safe to start consuming without a second validation pass.
//   * `visual_ref`. The name a viewer or editor resolves to a mesh. The engine
//     is headless and never resolves it; it rides here so that the world/model
//     files S5 loads do not need a parallel table keyed on model name.
//
// Both are description, which is what a ModelType is. `visual_ref` is not a
// stub for behaviour that is silently missing: the behaviour it would drive
// does not exist anywhere in v2, in any form.
// ===========================================================================

namespace spade::vehicles {

// ---------------------------------------------------------------------------
// Fixed upper bounds on how many of each thing one model may declare.
//
// They exist so a VehicleRef (sim/simulation.hpp) can carry its instance's
// slots INLINE, with no allocation and no second lookup -- which is what makes
// set_rotor_commands() an O(1) array write instead of a scan over a world's
// rotor partition. They are model-authoring limits, NOT world capacities: a
// world's capacity for rotors and drag bodies is its declared
// Capacities::force_elements, shared across every vehicle in it.
//
// Raising one is a one-line change here plus a bigger VehicleRef. Eight rotors
// covers quad/hexa/octo; four IMUs covers a redundant triad plus a spare.
// ---------------------------------------------------------------------------
inline constexpr std::size_t kMaxModelRotors = 8;
inline constexpr std::size_t kMaxModelDragBodies = 4;
inline constexpr std::size_t kMaxModelImuMounts = 4;

// ---------------------------------------------------------------------------
// The body template: what every instance's BodyState starts as, minus the
// pose (which is per-spawn).
//
// `inertia_diag` IS THE INERTIA, NOT ITS INVERSE. state/layout.hpp stores
// inv_inertia_diag because that is what the integrator multiplies by; a model
// author writes kg m^2 and the spawn path inverts it, once, in one place. A
// zero component is REJECTED here rather than reinterpreted as the
// integrator's "this axis cannot be angularly accelerated" -- a vehicle with a
// locked rotational axis is not a vehicle, and admitting it would make
// 1/I silently mean two different things.
// ---------------------------------------------------------------------------
struct BodyTemplate {
    float mass = 1.0f;             // kg, > 0
    glm::vec3 inertia_diag{1.0f};  // body-frame PRINCIPAL moments, kg m^2, componentwise > 0
};

// ---------------------------------------------------------------------------
// One rotor, as a description: vehicles/rotor.hpp's RotorRow minus everything
// that is state or identity.
//
// Dropped relative to RotorRow, and why: `body_slot` and `enabled` are
// instance identity (the spawn writes them), `omega`/`omega_cmd` are dynamic
// state (the spawn seeds them from VehicleSpawn::rotor_omega and the
// RotorElement pass owns them thereafter), and the four reserved lanes stay 0
// until a task claims them (rotor.hpp's reservation note) so there is nothing
// for an author to say about them.
//
// UNITS AND CONVENTIONS ARE RotorRow's, verbatim -- see vehicles/rotor.hpp,
// which is the authority for all four of the model's constants and for what
// `spin_dir` means. In particular `radius` MUST be > 0: rotor.hpp's
// apply_rotors() documents that radius == 0 silently disables BOTH aero
// corrections (no disc area, so no induced velocity and no image source) and
// hands back a rotor producing its static thrust in every flight state with
// nothing anywhere reporting a problem. Rejecting it is this layer's job, and
// validate() below is where that happens.
// ---------------------------------------------------------------------------
struct RotorDesc {
    glm::vec3 local_pos{0.0f};                       // r, BODY-frame hub offset from the COM, m
    glm::quat local_orient{1.0f, 0.0f, 0.0f, 0.0f};  // local -> body; thrust axis = local_orient * +Y
    float spin_dir = 1.0f;                           // +1 / -1 / 0; a DIRECTION, not a scale
    float tau = 0.0f;                                // RPM-lag time constant, s; <= 0 means no lag
    float radius = 0.0f;                             // disc radius R, m; MUST be > 0 (see above)
    float thrust_coeff = 0.0f;                       // k_T in T_static = k_T w^2, N s^2; > 0
    float torque_coeff = 0.0f;                       // k_Q in Q = k_Q w^2, N m s^2; >= 0
};

// One drag element, as a description. Mirrors sim/simulation.hpp's
// DragElementSpawn field for field -- see this header's note on why that
// duplication is the layering's price and where the mapping lives.
struct DragBodyDesc {
    uint32_t mode = physics::drag_mode::quadratic;
    float area = 0.0f;                               // m^2, quadratic mode only
    glm::vec3 coeffs{0.0f};                          // see physics::drag_mode:: for the per-mode meaning
    glm::vec3 local_pos{0.0f};                       // BODY-frame offset from the COM, m
    glm::quat local_orient{1.0f, 0.0f, 0.0f, 0.0f};  // local -> body
};

// One IMU mount, as a description. Mirrors sim/simulation.hpp's
// ImuSensorSpawn field for field, same reasoning. Units and frames are
// sensors/imu.hpp's: the four sigmas are PER-SAMPLE standard deviations, not
// spectral densities, and the default is an ideal sensor at the COM sampling
// every substep.
struct ImuMountDesc {
    glm::vec3 mount_pos{0.0f};                       // BODY-frame offset from the COM, m
    glm::quat mount_orient{1.0f, 0.0f, 0.0f, 0.0f};  // mount -> body
    uint32_t rate_divider = 1;                       // emit one sample every N substeps; >= 1
    float sigma_a = 0.0f;                            // accel white noise, m/s^2 per sample
    float sigma_g = 0.0f;                            // gyro white noise, rad/s per sample
    float sigma_ba = 0.0f;                           // accel bias walk step, m/s^2 per sample
    float sigma_bg = 0.0f;                           // gyro bias walk step, rad/s per sample
};

// ---------------------------------------------------------------------------
// The model type itself.
//
// A std::vector per element kind rather than a fixed array: a ModelType is
// AUTHORING-SIDE, built once outside the step loop and copied into the
// Simulation's registry, so it is allowed to be convenient in ways an arena
// row is not. The fixed bounds above are enforced by validate(), which is what
// lets VehicleRef carry an instance's slots inline.
//
// ELEMENT ORDER IS PART OF THE CONTRACT. `rotors[i]` is rotor i for the whole
// life of an instance: it is the slot VehicleRef::rotor_slots[i] names and the
// command Simulation::set_rotor_commands() takes at index i. A model that
// reorders its rotors between two runs is a different model.
// ---------------------------------------------------------------------------
struct ModelType {
    std::string name;                  // non-empty; identity for a human, not for the engine
    uint32_t param_schema_id = 0;      // which parameter schema these values were authored against
    std::string visual_ref;            // viewer/editor mesh reference; NOT read by the engine
    uint32_t version = 1;              // the compiled model's version (DBE-005); >= 1

    // THE DESIGN FRAME (drone-builder physics DBP-44..47). The model is
    // compiled into its principal body frame -- every mount below is expressed
    // there -- but a vehicle is spawned and read in its DESIGN frame, the
    // flight controller's. These two say how the frames relate; only this
    // struct, Simulation::spawn() and Simulation::vehicle_state() see them
    // (DBP-45; sim/design_frame.hpp has the conversion). The defaults -- the
    // identity and zero -- make the frames coincide, which every model built
    // before the design frame existed does.
    glm::quat design_to_principal{1.0f, 0.0f, 0.0f, 0.0f};  // q_bd: design-frame vectors -> body frame
    glm::vec3 com_offset{0.0f};  // the centre of mass from the design origin, design frame, m

    BodyTemplate body{};
    // m, >= 0. CONSUMED as of D-S6-2 (see the header note): every vehicle
    // spawn() writes this value into its body's BodyState::proxy_radius
    // (state/layout.hpp), which is what both contact passes
    // then read via physics::effective_proxy_radius(). Left at its 0.0f
    // default, a model's bodies fall back to the WORLD's ContactParams::
    // proxy_radius -- the same answer this field gave before it was consumed.
    float proxy_radius = 0.0f;

    std::vector<RotorDesc> rotors;
    std::vector<DragBodyDesc> drag_bodies;
    std::vector<ImuMountDesc> imu_mounts;

    // How many of this world's declared Capacities::force_elements one
    // instance consumes. Rotors and drag bodies live in SEPARATE arena arrays
    // but share ONE declared budget, because spec S3 calls them both force
    // elements ("ForceElements // rotors -> drag -> lift surfaces (each
    // element type = one batched pass)") and a world file says how many force
    // elements a world may hold, not how many of each kind.
    [[nodiscard]] std::size_t force_element_count() const noexcept {
        return rotors.size() + drag_bodies.size();
    }

    // -----------------------------------------------------------------------
    // Everything that makes a model unbuildable, checked ONCE, here.
    //
    // This is the layer that owns the checking, in the same sense
    // sim/world_set.hpp's validate_world_set() owns it for a world: the
    // per-substep passes document their preconditions and do not check them
    // (they are the inner loop), so a misconfigured model must be an error at
    // registration rather than a NaN, or worse a plausible-looking wrong
    // number, forty substeps later.
    //
    // Every predicate is spelled `!(x > 0)` rather than `x <= 0` so a NaN
    // REJECTS instead of comparing false on both sides -- the discipline
    // world_set.cpp's in_range() and contacts.cpp's guards both use.
    //
    // Codes: invalid_argument for every failure (a bad model is a caller
    // error, not a capacity one).
    // -----------------------------------------------------------------------
    [[nodiscard]] Result<void> validate() const;
};

// The design-to-principal rotation as a model stores it: `q` normalized, and
// negated if the first non-zero of (w, x, y, z) is negative, so q and -q -- one
// rotation -- are one value. The identity is returned bit for bit. The one rule
// (TD-9) that Simulation::register_model() and the airframe compiler both
// apply, so a model's stored rotation never depends on which sign its author
// wrote.
[[nodiscard]] glm::quat canonical_design_rotation(const glm::quat& q) noexcept;

}  // namespace spade::vehicles
