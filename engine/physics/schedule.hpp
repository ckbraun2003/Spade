#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "core/time.hpp"
#include "physics/contacts.hpp"
#include "physics/field_row.hpp"
#include "physics/forces.hpp"
#include "physics/grid.hpp"
#include "sensors/gnss.hpp"
#include "sensors/imu.hpp"
#include "state/layout.hpp"
#include "state/state_view.hpp"
#include "vehicles/rotor.hpp"
#include "world/medium.hpp"
#include "world/sdf.hpp"

// vehicles/rotor.hpp is included because a span of an incomplete type breaks MSVC.
// WorldSubstepView::rotors below is a std::span<vehicles::RotorRow>. Until
// 2026-10-03 RotorRow was only forward-declared here, and MSVC rejected some
// uses of that span, such as indexing ctx.worlds (C2036, "unknown size"), in
// any TU that had not included rotor.hpp itself. schedule.cpp and
// objects/behaviors/kinematic_mover.cpp each carried that include as a
// workaround. Core found it. rotor.hpp includes nothing from physics/schedule, so
// there is no cycle, and spade_sim (which compiles schedule.cpp) already links
// spade::vehicles.

// objects::BehaviorRegistry -- forward-declared because including it here
// would be a CYCLE: objects/behavior.hpp names SubstepContext. SubstepContext below
// holds only a POINTER to a registry and this header never dereferences one;
// physics/schedule.cpp, the single TU that calls run_slot(), includes the real
// header. objects/behavior.hpp forward-declares SubstepContext symmetrically.
namespace spade::objects {
class BehaviorRegistry;
}  // namespace spade::objects

// ---------------------------------------------------------------------------
// THE SUBSTEP PASSES. Each function below is one built-in module's CPU pass
// (sim/standard_modules.cpp). Which phase it runs in, what it reads and
// writes, and which passes it must follow are declared there, and
// sim/module.hpp's compile_schedule() turns a module set into the one ordered
// list every substep runs. The order is a parity contract (fp32 addition is
// not associative), so it is data the compiler checks and the identity hash
// records, never a call sequence written by hand.
//
// A PASS TAKES THE WHOLE WORLD SET, NOT ONE WORLD. §3's execute_cpu(WorldSpan)
// signature is deliberate and this header follows it: dynamic_contact.resolve is a
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
    // (state/layout.hpp), so both contact passes read it
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
    // (physics/contacts.hpp), which both contact passes, static and dynamic,
    // call so the two passes cannot disagree about a body's size.
    ContactParams contacts{};
    GridParams grid{};                           // this world's broad-phase cell size

    // This world's sensor table and its output-ring storage (Task 19).
    // MUTABLE, unlike drag_elements: a sensor row carries its own rate-divider
    // phase, bias random walk, rng stream and ring write cursor, all of which
    // the imu.synthesize pass advances. `imu_ring` holds exactly
    // imu_sensors.size() * sensors::kRingDepth samples -- sensor i owns the
    // window starting at i * kRingDepth (sensors/imu.hpp: the ring reference is
    // implicit in the slot).
    std::span<sensors::ImuSensorRow> imu_sensors;
    std::span<sensors::ImuSample> imu_ring;
    // The second sensor kind, same shape and the same ring arithmetic:
    // `gnss_ring` holds exactly gnss_sensors.size() * sensors::kRingDepth
    // fixes, receiver i owning [i*kRingDepth, (i+1)*kRingDepth). Two spans
    // rather than a variant because the two row types differ in size and
    // layout, which is also why they are two arenas.
    std::span<sensors::GnssSensorRow> gnss_sensors;
    std::span<sensors::GnssFix> gnss_ring;

    // This world's rotor partition (Task 18). MUTABLE, unlike drag_elements
    // and for the same reason imu_sensors is: a rotor row carries its own
    // lagged shaft speed and the command it is lagging toward, and the
    // rotor.forces pass advances both
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

    // This world's field sample row (physics/field_row.hpp): the compiled
    // schedule's field_stride floats. Scratch, not state -- the Fields phase's
    // provider passes write it every substep before any reader reads it.
    std::span<float> fields;
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
    // order -- what the batched dynamic-contact sweep takes (D8: "one dispatch
    // steps N worlds"). Index i of both spans is global arena slot i.
    std::span<BodyState> all_bodies;
    std::span<const uint32_t> all_slot_to_world;

    // DECIDED at create(), from WorldSetLayout::uniform_dynamic_params, and
    // copied into each step's context by Simulation::step(). The distinction
    // matters: the value is a function of CONFIG alone and is fixed for the
    // Simulation's lifetime, so this can never become a state-dependent branch
    // in the step loop. See dynamic_contact.resolve below for what it selects
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

    // The behaviors this world set runs, or NULL for a set that registers
    // none -- which is every scenario in the golden corpus, and therefore the
    // inert path Task 7 pinned. Non-owning, like every other view here: the
    // registry outlives the step, and a substep never mutates it (run_slot is
    // const), so behaviors cannot register behaviors.
    const objects::BehaviorRegistry* behaviors = nullptr;

    // The STEP duration in seconds -- dt, NOT the substep h above. Config
    // alone, fixed for the Simulation's lifetime, and converted exactly as
    // substep_h() converts (sim/simulation.hpp's ns -> s contract).
    //
    // It exists for behaviors, which derive their pose from `tick` and must
    // therefore know what one tick is worth. Tick counts STEPS, so multiplying
    // by `h` would be off by the substep count -- a bug that looks right and
    // scales silently with a scenario's substeps setting. No pass that predates
    // the behavior slots reads this.
    float dt_s = 0.0f;

    // The STEP being executed -- what the sensor passes stamp their samples with.
    //
    // NOT A CLOCK. Tick is the engine's only notion of time (core/time.hpp) and
    // this is a copy of the Simulation's counter, taken once per step; nothing
    // here reads a wall clock. Every substep of step k carries k, because Tick
    // counts STEPS and Simulation::step() increments it after the last substep
    // (engine A9). A consumer that needs
    // sub-step resolution reads the sample's monotonically increasing
    // SampleIndex, which is what it is for.
    Tick tick{};

    // The RUNNING PASS's declared state (module-API stage 4, Task 6): state[i]
    // views what the pass's i-th declared access names (sim/module.hpp's
    // CompiledBinding) -- a module array, which may be another module's; a
    // configuration table, read-only, one row of floats every world shares
    // (Task 7); or an absent view for a core quantity, a field, a stateless
    // module's token or an optional read of a module the set does not hold.
    // So a pass indexes it by its own access list, and sees nothing it did not
    // declare.
    // Simulation::step() sets it before each pass from views it refills in
    // place; nothing is allocated.
    //
    // The built-in passes do not read it: they keep their typed members of
    // WorldSubstepView above until the GPU module ABI (stage-4 plan, open
    // question 5), so no pass body moved.
    std::span<const StateView> state{};
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

// THE ORDER LIVES IN THE MODULES. These passes are the built-in modules' CPU
// functions; sim/standard_modules.cpp declares each one's phase, access and
// edges, and sim/module.hpp's compile_schedule() orders them. There is no
// fixed pass array any more (module-API plan, stage 1).

// ---------------------------------------------------------------------------
// The passes themselves. Declared individually (rather than left as anonymous
// entries in the schedule array) so that each one is a named, separately
// testable, separately GPU-mirrorable unit -- §3's "every pass is one
// interface, two implementations".
// ---------------------------------------------------------------------------

// dryden.advance -- advance every world's Dryden gust filter by one substep.
//
// IN FIELDS, AND EXACTLY ONCE PER SUBSTEP. world/medium.hpp states the contract
// this pass exists to honour: "dryden_advance() must be called EXACTLY ONCE PER
// SUBSTEP for each world, in the dryden.advance pass ... and therefore BEFORE any
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

// behaviors.kinematic -- runs the attached BehaviorRegistry's kinematic slot;
// inert when none is attached (every golden scenario).
//
// The module schedule places this FIRST in Fields (user ruling Q2): a pose written
// by a kinematic behavior must be set before anything reads it, and both
// collision passes read poses. Placing it later would let a body collide
// against the position it held last substep.
//
// Inert because the thing that fills it does not exist yet. It is represented
// rather than absent so the schedule stays one reviewable object. (Before the
// module API it shared that reason with an empty Gravity slot; the module
// schedule has no gravity pass.) See pass_behaviors_force below for the proof
// obligation.
void pass_behaviors_kinematic(const SubstepContext&) noexcept;

// Rotor forces, then drag -- the force elements, one module each. Both add
// into the same two float accumulators, so their order is a parity contract
// (rotors first; drag declares `after: rotor.forces`). Each reads the medium
// from its world's field row through a SampledMedium view built on the stack:
// the density and wind the providers below wrote this substep (module-API
// stage 3). The row is rewritten every substep, so a snapshot restore cannot
// leave it stale for a reader.
void pass_rotor_forces(const SubstepContext&) noexcept;
void pass_drag(const SubstepContext&) noexcept;

// The built-in field providers (module-API stage 3), in Fields. Each writes its
// fields into every world's sample row:
//   pass_environment_sample  gravity and density, copied from WorldParams;
//   pass_dryden_sample       wind = WorldParams::wind + the world's Dryden gust,
//                            the expression DrydenMedium::sample() returns.
// Both are position-independent: one value per world.
void pass_environment_sample(const SubstepContext&) noexcept;
void pass_dryden_sample(const SubstepContext&) noexcept;

// behaviors.force -- runs the attached BehaviorRegistry's force slot; inert
// when none is attached (every golden scenario).
//
// SL6 places this LAST in Forces so behavior wrenches accumulate after the
// rotors-then-drag order the golden corpus pins. Float addition is not
// associative, so running before would change force_acc's last bits -- a
// difference no test of either element alone can see, and one that would
// surface later as a CPU/GPU parity mystery. Same reasoning, same contract, as
// the "rotors -> drag" ordering documented on pass_rotor_forces above.
//
// With nothing registered this MUST be a provable no-op, and the proof is the
// golden corpus: Determinism.DigestsMatchTheCommittedGoldenCorpus and, more
// strongly, GoldenCorpus.TheDataScenariosReproduceTheRetiredBuilderCorpus,
// whose four digests are spelled independently in C++ and therefore cannot be
// satisfied by re-blessing a scenario file.
void pass_behaviors_force(const SubstepContext&) noexcept;

// static_contact.resolve -- every world's active bodies against its world SDF.
void pass_collision_static(const SubstepContext&) noexcept;

// dynamic_contact.resolve -- the sorted-grid body-body sweep.
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

// Integrate -- one substep of symplectic Euler per world. Applies gravity itself
// (engine A9: there is no separate gravity pass, so nothing may also add m*g),
// captures specific force, and CLEARS the accumulators so the next substep's
// force passes start from zero.
void pass_integrate(const SubstepContext&) noexcept;

// IMU, then GNSS -- sensor synthesis, one module each, in the Sensors phase,
// AFTER Integrate: the accelerometer's input is BodyState::specific_force,
// which Integrate captures inside the substep it belongs to. The two commute
// (disjoint arrays, separately tagged streams, and state_digest folds in
// registration order), so their order is readability, not contract.
void pass_sensor_imu(const SubstepContext&) noexcept;
void pass_sensor_gnss(const SubstepContext&) noexcept;

}  // namespace spade::physics
