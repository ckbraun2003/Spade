#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "core/time.hpp"
#include "physics/contacts.hpp"
#include "physics/forces.hpp"
#include "physics/grid.hpp"
#include "sensors/imu.hpp"
#include "state/layout.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"

// vehicles::RotorRow -- forward-declared, not included (S5 T9 schedule seam
// ticket, mechanical per the T18 reviewer's sketch): WorldSubstepView::rotors
// below is a std::span<vehicles::RotorRow>, and a span of an incomplete type
// is valid until element access -- this header never dereferences a rotor,
// only names the type for the span's declaration. The complete definition is
// pulled in by physics/schedule.cpp, the one TU that actually walks
// `rotors` (via vehicles::apply_rotors()). Proven with both compilers via
// CI (this task's report has the run); revert-and-document if a compiler
// disagreed -- see this comment's own presence for which outcome landed.
namespace spade::vehicles {
struct RotorRow;
}  // namespace spade::vehicles

// ---------------------------------------------------------------------------
// THE SUBSTEP PASS SCHEDULE (engine design spec §3 "The step model").
//
// §3 is quoted here verbatim because this file's entire job is to be that
// quote, executable:
//
//     // per substep, in this order, always:
//     MediumUpdate       // Dryden gust states advance (seeded, per world)
//     ForceElements      // rotors -> drag -> lift surfaces (each element type
//                        //   = one batched pass)
//     Gravity
//     CollisionStatic    // dynamic proxies vs world SDF: distance + gradient
//                        //   -> contact impulses
//     CollisionDynamic   // sorted-grid pair path (exact cell compare)
//     Integrate          // symplectic Euler + quaternion exp-map; captures
//                        //   specific force for sensors
//     SensorSynthesis    // only on sensor-rate boundaries; writes sensor
//                        //   output rings
//     Publish            // tick counter++, snapshot ring hook, frame-state
//                        //   copy point
//
// "THE SCHEDULE OBJECT IS DATA; PASSES CANNOT REORDER THEMSELVES" (§3). That
// sentence is the reason substep_schedule() returns a span over a fixed,
// file-scope array of {name, function} records rather than the step loop simply
// calling eight functions in a row. The difference is not stylistic:
//
//   * the ORDER is one reviewable, testable object -- test_determinism.cpp
//     asserts the eight names in sequence against the spec text above, so a
//     reordering is a red test rather than a physics regression found weeks
//     later in a parity diff;
//   * a pass is a plain function pointer over a context struct, which is the
//     shape §3's "every pass is one interface, two implementations --
//     execute_cpu(WorldSpan) and record_gpu(CommandRecorder&)" grows into at
//     S6: the record_gpu half becomes a second function pointer in the same
//     record, and the schedule array does not change;
//   * nothing downstream can inject a pass, skip one, or run them in a
//     different order, because there is no API to do so.
//
// A PASS TAKES THE WHOLE WORLD SET, NOT ONE WORLD. §3's execute_cpu(WorldSpan)
// signature is deliberate and this header follows it: CollisionDynamic is a
// SINGLE sorted-grid sweep keyed on (world_id, cell) (D8), so it cannot be
// expressed as a per-world callback, while the other passes iterate the world
// span serially ("The CPU twin iterates worlds serially with identical per-world
// math -- parity comparisons are per-world, so a divergence names its world",
// §4). Making every pass take the set keeps one uniform signature instead of
// two.
//
// WHAT IS *NOT* HERE. No substep loop, no tick, no structural queue, no
// allocation, no clock, and no ownership of anything. The schedule advances
// state by exactly one substep of duration h and returns; owning the substep
// loop, the step boundary and the tick counter is sim/simulation.hpp's job,
// exactly as physics/integrator.hpp says ("NO SUBSTEP LOOP HERE ... Owning the
// substep loop, the step boundary, and the tick counter is the
// Simulation/schedule layer's job").
// ---------------------------------------------------------------------------

namespace spade::physics {

// ---------------------------------------------------------------------------
// One world's slice of the substep, as the passes see it.
//
// Every member is a NON-OWNING view into state the Simulation owns: arena
// partitions (spans), per-world rows (pointers), and the two value records
// small enough to carry by value. Nothing here is allocated, and a view is
// rebuilt from the arenas at the top of every step -- so a snapshot restore,
// which rewrites arena bytes in place but never moves them, cannot leave a
// stale view behind.
//
// `bodies` and `drag_elements` are the SAME world's partitions, which is what
// makes DragBodyRow::body_slot -- documented as an index into the world's body
// slice, not a global arena slot -- correct by construction.
// ---------------------------------------------------------------------------
#if defined(_MSC_VER)
#pragma warning(push)
// C4324: "structure was padded due to alignment specifier". ContactParams and
// GridParams are alignas(16) because they are std430 records the S6 device
// buffers mirror, so any host aggregate holding one inherits that alignment and
// gains tail padding. Intended, and free here -- these two structs are transient
// call frames rebuilt every step: nothing uploads, hashes or snapshots them.
// Same disable, same reason, as state/layout.hpp's WorldParams.
#pragma warning(disable : 4324)
#endif
struct WorldSubstepView {
    const WorldParams* params = nullptr;         // this world's row of the param array
    // This world's body partition. NO SEPARATE PER-BODY RADIUS SPAN LIVES
    // HERE (D-S6-2): each BodyState already carries its own `proxy_radius`
    // (state/layout.hpp), so CollisionStatic and CollisionDynamic read it
    // directly off the elements of THIS span via physics::effective_proxy_
    // radius() -- one obvious place, rather than a second span a pass would
    // have to index in lockstep with this one. THE OWNERSHIP SPLIT: a
    // vehicle's own model owns its body's proxy (written once, at spawn, from
    // ModelType::proxy_radius -- see Simulation::spawn(world, ModelTypeId,
    // VehicleSpawn)); `contacts.proxy_radius` below owns the PLAIN-body and
    // fallback default. 0 in a body's lane means "defer to the world's
    // default", which is every slot a bare spawn() ever produces.
    std::span<BodyState> bodies;
    std::span<const DragBodyRow> drag_elements;  // this world's drag-element partition
    std::span<const uint32_t> body_slot_to_world;  // this world's slice of the bodies map
    DrydenState* dryden = nullptr;               // this world's turbulence filter row
    const DrydenParams* dryden_params = nullptr; // that filter's configuration
    const SdfProgram* sdf = nullptr;             // this world's static geometry
    // This world's material/solver record. `proxy_radius` here is the
    // FALLBACK a per-body override (`bodies[i].proxy_radius`, above) defers
    // to when it is the 0 sentinel -- see physics::effective_proxy_radius()
    // (physics/contacts.hpp), which both CollisionStatic and CollisionDynamic
    // call so the two passes cannot disagree about a body's size.
    ContactParams contacts{};
    GridParams grid{};                           // this world's broad-phase cell size

    // This world's sensor table and its output-ring storage (Task 19).
    // MUTABLE, unlike drag_elements: a sensor row carries its own rate-divider
    // phase, bias random walk, rng stream and ring write cursor, all of which
    // the SensorSynthesis pass advances. `imu_ring` holds exactly
    // imu_sensors.size() * sensors::kRingDepth samples -- sensor i owns the
    // window starting at i * kRingDepth (sensors/imu.hpp: the ring reference is
    // implicit in the slot).
    std::span<sensors::ImuSensorRow> imu_sensors;
    std::span<sensors::ImuSample> imu_ring;

    // This world's rotor partition (Task 18). MUTABLE, unlike drag_elements
    // and for the same reason imu_sensors is: a rotor row carries its own
    // lagged shaft speed and the command it is lagging toward, and the
    // RotorElement half of the ForceElements pass advances both
    // (vehicles/rotor.hpp section 5).
    //
    // WHY A vehicles/ TYPE APPEARS IN A physics/ HEADER, since spec S3's tree
    // comment ("nothing vehicle-specific below here") reads like it forbids
    // exactly this. It does not, and the distinction is worth stating: this
    // header's COMPILATION UNIT is spade_sim, not spade_physics (see
    // engine/CMakeLists.txt, which already records that the schedule's header
    // residency and its target deliberately differ), and spec S3's own pass
    // list names rotors as the FIRST thing the ForceElements pass applies --
    // "ForceElements // rotors -> drag -> lift surfaces". The schedule is
    // where the engine says which element kinds exist and in what order they
    // run, so it is the one place above physics/ that has to know. The
    // dependency arrow still points down: vehicles/ knows nothing about the
    // schedule, and spade_physics itself does not compile this header.
    std::span<vehicles::RotorRow> rotors;
};

// ---------------------------------------------------------------------------
// Everything one substep of one world set needs. Passed by const reference:
// the passes mutate STATE (through the spans and pointers inside), never the
// context itself.
// ---------------------------------------------------------------------------
struct SubstepContext {
    // Per-world views, indexed by world id. Order is world index order, which
    // is the only order anything in this engine iterates worlds in.
    std::span<const WorldSubstepView> worlds;

    // The WHOLE bodies array and its slot->world map, all worlds, in slot
    // order -- what the batched CollisionDynamic sweep takes (D8: "one dispatch
    // steps N worlds"). Index i of both spans is global arena slot i.
    std::span<BodyState> all_bodies;
    std::span<const uint32_t> all_slot_to_world;

    // DECIDED at create(), from WorldSetLayout::uniform_dynamic_params, and
    // copied into each step's context by Simulation::step(). The distinction
    // matters: the value is a function of CONFIG alone and is fixed for the
    // Simulation's lifetime, so this can never become a state-dependent branch
    // in the step loop. See the CollisionDynamic pass below for what it selects
    // and why both branches are numerically identical.
    bool batch_dynamic_collision = false;
    ContactParams dynamic_contacts{};  // meaningful only when batch_dynamic_collision
    GridParams dynamic_grid{};         // meaningful only when batch_dynamic_collision

    // The broad phase's working storage, owned by the Simulation so the steady
    // state is allocation-free (physics/grid.hpp's GridScratch note). Transient:
    // rebuilt from body positions on every call, therefore not registered state
    // and not in any snapshot.
    GridScratch* scratch = nullptr;

    // Effective substep duration, SECONDS. dt / substeps -- the same quantity
    // integrate_bodies(), resolve_static_contacts() and dryden_advance() all
    // take. See Simulation::substep_h() for the ns -> s conversion contract.
    float h = 0.0f;

    // The STEP being executed -- what SensorSynthesis stamps its samples with.
    //
    // NOT A CLOCK. Tick is the engine's only notion of time (core/time.hpp) and
    // this is a copy of the Simulation's counter, taken once per step; nothing
    // here reads a wall clock. Every substep of step k carries k, because Tick
    // counts STEPS and Simulation::step() increments it after the last substep
    // (see pass_publish()'s note on §3's shorthand). A consumer that needs
    // sub-step resolution reads the sample's monotonically increasing
    // SampleIndex, which is what it is for.
    Tick tick{};
};
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

// ---------------------------------------------------------------------------
// A pass: a name and a function. `noexcept` is part of the type -- these run at
// substep rate below a `Result`-returning API and have nothing to report; a
// pass that needs to fail needs a design conversation, not an exception.
// ---------------------------------------------------------------------------
using PassFn = void (*)(const SubstepContext&) noexcept;

struct Pass {
    std::string_view name;
    PassFn run;
};

// The eight passes of §3, in order. Fixed at compile time; there is no API to
// add, remove or reorder one.
inline constexpr std::size_t kSubstepPassCount = 8;

[[nodiscard]] std::span<const Pass> substep_schedule() noexcept;

// Runs the whole schedule once, in order. Equivalent to iterating
// substep_schedule() and calling each entry -- and that equivalence is asserted
// by a test, so the convenience wrapper cannot drift away from the declared
// order it is a shorthand for.
void run_substep(const SubstepContext& ctx) noexcept;

// ---------------------------------------------------------------------------
// The passes themselves. Declared individually (rather than left as anonymous
// entries in the schedule array) so that each one is a named, separately
// testable, separately GPU-mirrorable unit -- §3's "every pass is one
// interface, two implementations".
// ---------------------------------------------------------------------------

// MediumUpdate -- advance every world's Dryden gust filter by one substep.
//
// FIRST, AND EXACTLY ONCE PER SUBSTEP. world/medium.hpp states the contract
// this pass exists to honour: "dryden_advance() must be called EXACTLY ONCE PER
// SUBSTEP for each world, in the MediumUpdate pass ... and therefore BEFORE any
// force element reads sample(). Calling it twice per substep halves the
// correlation time and inflates the number of gust draws; calling it zero times
// freezes the gust for that substep. Both are silent, so the pass that owns the
// schedule owns this invariant." This is that pass.
//
// UNCONDITIONAL, INCLUDING AT TURBULENCE LEVEL none. dryden_advance() draws
// exactly five gaussians per call whatever the sigmas are, by design, so the
// stream position is a function of the substep count alone and not of the
// turbulence level. Skipping the advance for a zero-sigma world would make two
// runs of the same scenario at different turbulence levels diverge in every
// OTHER stochastic system as well. So: no branch here, ever.
void pass_medium_update(const SubstepContext&) noexcept;

// ForceElements -- accumulate every force element's wrench into its body.
//
// TWO ELEMENT KINDS, IN §3'S ORDER: RotorElement (vehicles/rotor.hpp, Task 17,
// wired here by Task 18) then DragBody (physics/forces.hpp, Task 15). Lift
// surfaces are post-v2 and become a third batched call in this same function.
//
// THE ORDER IS THE SPEC'S AND IT IS A PARITY CONTRACT, not a preference. §3
// says "rotors -> drag -> lift surfaces", and both kinds accumulate into the
// SAME two float accumulators with `+=`. Float addition is not associative, so
// applying drag before rotors would produce a different force_acc in the last
// bits -- a difference no test of either element alone can see, and one that
// would surface at S6 as a CPU/GPU parity mystery. Hence: rotors first,
// always, and the golden corpus is what pins it.
//
// Reads the medium through a per-world DrydenMedium view constructed HERE,
// on the stack, from this world's filter row and parameters. That is a
// deliberate choice over caching the views in the Simulation: a cached view is
// two raw pointers that a snapshot restore would have to be remembered to
// rebind, and "remembered to" is how replay guarantees die. Constructing it
// costs two pointer stores per world per substep and cannot go stale.
void pass_force_elements(const SubstepContext&) noexcept;

// Gravity -- REPRESENTED BUT INERT. THIS PASS DELIBERATELY DOES NOTHING.
//
// Gravity application lives inside Integrate (Task 9 op-order contract); this
// pass slot is retained for schedule-shape parity with spec §3 and MUST NOT
// also accumulate m*g.
//
// The long form, from physics/integrator.hpp's own header: integrate_bodies()
// adds `params.gravity` to the acceleration directly rather than reading m*g
// out of force_acc, "and it is what makes the specific-force capture exact
// rather than a subtraction. The schedule's Gravity pass must therefore NOT
// also add m*g to force_acc, or gravity is applied twice." Deleting this slot
// instead of emptying it would have been the other defensible choice; keeping
// it is what makes this file a literal transcription of §3, so that a reader
// diffing the two finds eight names against eight names and this comment
// explaining the one that is empty. A test asserts the emptiness (a body under
// gravity falls by exactly one g, not two).
void pass_gravity(const SubstepContext&) noexcept;

// CollisionStatic -- every world's active bodies against its world SDF.
void pass_collision_static(const SubstepContext&) noexcept;

// CollisionDynamic -- the sorted-grid body-body sweep.
//
// TWO FORMS, ONE ANSWER. When every world shares a ContactParams and a
// GridParams (WorldSetLayout::uniform_dynamic_params, decided once at create()
// from CONFIG), this issues ONE sweep over the whole bodies array -- D8's "one
// dispatch steps N worlds", and the form S6 mirrors. Otherwise it issues one
// sweep per world over that world's slice, because resolve_dynamic_contacts()
// takes a single material record for the whole call and a heterogeneous set
// would otherwise have one world's restitution applied to another's bodies.
//
// The two forms produce byte-identical per-world results. The grid key is
// (world, cell) compared exactly and the sort's final tiebreak is the slot, so
// within one world the sorted sub-sequence -- and therefore the Gauss-Seidel
// sweep order that IS this pass's parity contract -- is the same whether that
// world was swept alone or as part of a batch, and pairs never cross worlds by
// construction. test_determinism.cpp's batching-invariance test is the
// enforcement.
void pass_collision_dynamic(const SubstepContext&) noexcept;

// Integrate -- one substep of symplectic Euler per world. Applies gravity (see
// pass_gravity above), captures specific force, and CLEARS the accumulators so
// the next substep's force passes start from zero.
void pass_integrate(const SubstepContext&) noexcept;

// SensorSynthesis -- §3: "only on sensor-rate boundaries; writes sensor output
// rings". Task 19 filled the slot the schedule had been holding for it, and it
// is ONE function body's worth of change to this file, exactly as intended.
//
// AFTER Integrate, AND THAT ORDERING IS THE WHOLE POINT: the accelerometer's
// physical input is BodyState::specific_force, which integrate_bodies()
// captures inside the substep it belongs to (spec §5: "computed once, not
// reconstructed"). Running this pass before Integrate would sample the previous
// substep's forces against this substep's attitude.
//
// Iterates worlds serially, each world's sensors in slot order, and calls
// sensors::synthesize_imu() -- v2's only sensor kind. A second kind becomes a
// second batched call HERE, the way ForceElements will grow rotors and lift
// surfaces, and not a second pass.
void pass_sensor_synthesis(const SubstepContext&) noexcept;

// Publish -- NO-OP STUB; the S6 hook point.
//
// §3 gives this pass three jobs: "tick counter++, snapshot ring hook,
// frame-state copy point". None of the three belongs here yet, and one of them
// never will:
//
//   * THE TICK COUNTER IS NOT INCREMENTED HERE, and that is a correction to
//     §3's shorthand rather than an omission. Tick counts STEPS (core/time.hpp:
//     "a monotonic count of physics steps"), while this pass runs once per
//     SUBSTEP -- incrementing here would tick `substeps` times per step and
//     make every snapshot header, every replay resume point and every
//     structural-queue boundary disagree with the API's own step count.
//     Simulation::step() owns the increment, once per step, after the last
//     substep.
//   * THE SNAPSHOT RING is the editor's continuous-rewind buffer (§4: "async
//     double-buffered staging copies every K ticks"), which is a GPU-path
//     object; on the CPU twin an on-demand Simulation::snapshot() is the whole
//     story.
//   * THE FRAME-STATE COPY POINT is the render/readback seam, which arrives
//     with the Vulkan backend at S6.
//
// It stays in the schedule because it is the one place those three hooks may
// ever attach, and having them attach at a declared point is the difference
// between a schedule and a call sequence.
void pass_publish(const SubstepContext&) noexcept;

}  // namespace spade::physics
