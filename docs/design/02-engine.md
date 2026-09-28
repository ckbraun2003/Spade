# Spade v2 -- Engine Core

**What this document is.** The engineering design of the Spade v2 engine core: the decision record, module
architecture, the step model and substep schedule, state layout and many-worlds batching, snapshot and readback,
physics content, the vehicle model-type layer, sensors, the compute backends and their parity mechanics, the
public API conventions, the rate envelope, and the test estate. Spade is designed as a general extreme-fidelity,
high-throughput physics engine -- Kat is its first consumer, not its shape. The long-term arc is modeling and
rendering the aerodynamics of motorized vehicles; the quadrotor is the first vehicle model class, not the point.

**What it harvests.** The ninth spec, *Spade v2 Engine Design* (approved v1.0, 2026-08-08), in full, including
Addendum A (signed 2026-08-10, `A1`-`A11` operative, `A3`/`A8` user-ruled). Plus the substep-schedule amendment
`SL6` from the Spade library and sandbox spec (user ruling, 2026-08-30), which is the single most important
correction carried in this document.

**Ruling series owned here.** `D1`-`D12` (the design-session decision record) and `A1`-`A11` (Addendum A). The **schedule half of `SL6`**
(which slots exist, and why each position is a parity contract) is owned here, because it amends this
document's schedule; `SL6`'s *registry* half -- registration order, GPU eligibility, determinism
obligations -- is owned by `04-objects.md`.

**Binding context.** Charter pillars `P1`-`P8` govern: GPU-authoritative plus CPU reference twin, three-grade
determinism, Vulkan, fixed-step and seeded, fat-Spade / thin-dronesim, offscreen render, the effects-to-aero
promotion rule, snapshot first-class. The editor technical spec's `TA1` (C5 presentation semantics) and `TA5`
(sensor-poll convention) bind the frame and sensor surfaces. Where this document details a mechanism the charter
only named, this document governs.

**Owned elsewhere.** The world file format, the scene model and rendering (03); the object/component model (04);
the C5 host seam (05); the sandbox and v1 retirement (06); phase status `S1`-`S8` (07).

---

## 1. The decision record

Twelve decisions taken in the 2026-08-07/08 design session. Each is pinned as written; the reasoning is what
makes it re-derivable, so it is recorded with the decision.

### `D1` -- Integrator

Semi-implicit (symplectic) Euler; quaternion advanced by the exponential map; inertia kept in the body frame;
accuracy bought with substeps rather than a higher-order scheme. The integrator API is a pure function of
`(state, wrench, dt)`.

*Why.* Symplectic Euler has bounded rather than secularly growing energy error, which is what a long-running
vehicle sim needs and what the physics-validation suite measures directly. The exponential map is exact for
constant angular velocity over a substep, so orientation does not accumulate the small-angle error a linearized
update would. Substeps, not order, buy accuracy because a single simple per-substep operation order is the thing
that can be pinned bit-for-bit across two backends -- a higher-order scheme multiplies the reduction orders that
would have to agree. Keeping the integrator a pure function means new integrator families can be added without
touching state layout.

### `D2` -- Precision

fp32 on **both** paths, with pinned reduction order. Worlds are local-frame by rule; no planetary coordinates.

*Why.* `P1`/`P2` make the GPU authoritative and the CPU its reference twin. A twin computing in fp64 would not
be a reference for an fp32 GPU -- every comparison would be measuring the precision gap instead of the
implementation. fp32 both sides is what makes the parity number mean something. Local-frame worlds keep fp32's
mantissa adequate for the scales the engine actually simulates; planetary coordinates would spend it on the
offset.

### `D3` -- Collision

Analytic SDF static world (primitives, transforms, CSG); sphere and capsule dynamic proxies; sorted grid for
dynamic-dynamic. Triangle-mesh colliders are a later add-on.

*Why.* Race gates are box and torus compositions -- an SDF gives them exactly, with zero mesh discretization
error, and the SDF is cheap to evaluate identically on both backends. The field is reused rather than
duplicated: rotor ground effect samples the same `phi_world` the collision pass reads, so the collision field
*is* the proximity query. Rendering the same program by raymarching (document 03) means the visual world and the
physics world cannot disagree.

### `D4` -- Vehicles

A model-type layer: engine asset classes one tier above meshes -- body template, collision proxy, force
elements, visual refs, parameter schema. Quadrotor first; generic `ForceElements` beneath.

*Why.* Nothing vehicle-specific may exist below `vehicles/`. Cars and fixed-wing then arrive as new element
types plus new model classes rather than as branches threaded through the physics core.

### `D5` -- Rotor aero

v2 = thrust/torque curves + RPM lag + momentum-theory inflow + SDF-sampled ground effect. BEMT and flow-field
coupling are roadmap, behind `P7`.

*Why.* Curves plus a momentum-theory inflow correction capture the climb/descent/hover thrust variation that
dominates quadrotor behavior, at a cost that fits the rate envelope. BEMT is a refinement of the same interface,
not a replacement of it, so deferring it costs no rework -- and the parameter schema reserves its fields now
(section 8) so the upgrade never changes layout.

### `D6` -- Medium

Per-world density and temperature, uniform wind, seeded Dryden turbulence, behind one `Medium` interface that
`P7`'s flow fields will later implement.

*Why.* The interface is the point. Aero code calls `sample(world, position)` and never learns which medium
answered, so promoting an effects-layer flow field to authoritative is an implementation swap rather than a
rewrite of every force element.

### `D7` -- Worlds

A `WorldBuilder` API plus a versioned YAML world file that serializes exactly what the builder builds. glTF
visual imports arrive later, render-only.

*Why.* One construction path with two front doors (fluent API, file) means the file format cannot describe a
world the builder cannot build, and validation is written once. Detail is owned by document 03.

### `D8` -- Many-worlds

Native world-index batching in every physics buffer from day one. N=1 is the degenerate case. The CPU twin loops
worlds serially.

*Why.* Retrofitting batching into a single-world layout is the expensive failure -- it touches every buffer,
every dispatch and every test. Building it in from the first commit costs nothing at N=1 and is the only way the
training fleet ever runs one dispatch over many worlds.

### `D9` -- Shaders

Slang modules. Shared layout definitions single-sourced in Slang, with C++ headers generated at build carrying
`static_assert`s on offsets and sizes. A generated binding registry.

*Why.* v1's two worst layout hazards were a struct duplicated across nine files and a GLM quaternion-order
coincidence that happened to work. Both are silent corruption when they break. Single-sourcing turns both into
build errors.

### `D10` -- Transition

Strangler: the v1 GL engine and sandbox stay alive untouched as a harness until the Vulkan renderer replaces
them (`S7`).

*Why.* v1 may not be quarantined while it is the only implementation of anything. Strangler code is explicitly
not maintained -- it dies with GL rather than being carried forward.

> **SUPERSEDED BY WHAT SHIPPED (`C4`):** `D10` as approved says the strangler runs until the
> **Vulkan** renderer replaces v1. It did not happen that way. S7a (`9778260`) shipped a **CPU
> rasterizer** and met the exit proof; verified 2026-09-07, `spade/engine/render/` contains no
> Vulkan backend at all (Vulkan here is compute-only). `RS1` is operative because it describes what
> exists -- see `03-world-and-render.md`. `D10` itself still HOLDS: v1 is present and untouched.

### `D11` -- Tests and CI

GTest plus CTest from the first v2 commit. Determinism replay and CPU-GPU parity as the spine. Bench baselines
in-repo. Spade's own CI legs (ubuntu CPU twin plus windows build), with GPU tests on the dev box.

> ⚠ **Carrier note, 2026-09-18 -- the decision is unchanged, the file it named is gone.** Those legs were
> `.github/workflows/spade.yml`; hosted CI was deleted on the user's ruling and they are now the
> `spade-linux` / `spade-windows` sections of `docker/run-ci-linux.sh` and `docker/run-ci-windows.ps1`.
> The bare `spade` ctest label outside the `T0`-`T3` vocabulary is unchanged and is asserted by
> `tests/test_tiers_single_home.py`, which now also proves a runner still ISSUES that label -- an
> exemption for a label nobody selects on is a hole with no remaining reason.

*Why.* Determinism and parity are properties that decay silently; they have to be measured on every commit from
the beginning or the first measurement becomes an archaeology project. See section 12.

### `D12` -- Toolchain

MSVC + Ninja + CMake presets. C++23, with `std::expected` as the error spine. The MinGW/CLion path is retired
for v2 targets.

*Why.* One toolchain per platform target, chosen for the Vulkan and Slang tooling the engine depends on.
`std::expected` is what lets the public API return errors without exceptions crossing the boundary (section 11).

---

## 2. Module architecture

```
spade/
  engine/          // v2 -- all new code lives here
    core/          // math (GLM + engine wrappers), typed IDs, Error/std::expected, time, config
    ecs/           // generational-handle entities, pools with stable slots, views; lifecycle real
    state/         // authoritative-state registry, world-partitioned SoA arenas, snapshot blobs
    world/         // World, WorldBuilder, YAML world file, SDF scene program, environment/Medium
    physics/       // integrator, force-element tables, collision (SDF + sorted grid), pass schedule
    vehicles/      // model-type layer: ModelType, Quadrotor; nothing vehicle-specific below here
    sensors/       // sensor tables + synthesis passes; IMU first, camera seat reserved
    compute/       // backend seam: pass interface, CPU backend, Vulkan backend (device/queues/arenas)
    shaders/       // Slang modules incl. shared/ layout definitions (single source of truth)
    render/        // scene model, rasterizer, FramePool  (document 03)
    testing/       // replay/parity/bench harness support
  src/ include/ examples/ assets/   // v1 GL engine -- strangler harness (document 06)
  tests/           // gtest suites, golden corpora, bench baselines
```

One CMake project, targets per module (`spade_core`, `spade_ecs`, ... `spade_cpu`, `spade_vulkan`,
`spade_tests`); presets for MSVC+Ninja (debug / release / relwithdebinfo). Install and export rules with
`spade::`-namespaced targets are the Kat build wiring: dronesim consumes Spade via `add_subdirectory` or
`find_package(spade)`, public API only.

**Dependency rules, enforced by target link visibility rather than by convention.** `core` depends on nothing.
`ecs`, `state`, `world`, `physics`, `vehicles` and `sensors` never include Vulkan or windowing headers. Only
`compute/vulkan` and `render` touch the GPU API. Nothing anywhere includes `windows.h` in a public header --
v1's `psapi` mistake is the counterexample that made this a rule.

FetchContent pins move to exact tags and commits (GLM, GLFW for the v1 harness, the Slang toolchain, GTest,
google-benchmark, yaml-cpp, Dear ImGui). The CMP0169-deprecated ImGui block is replaced.

**v1 patterns carried forward deliberately:** the POD / std430 single-mapping discipline (now generated, `D9`);
`instanceStartIndex` flattening; composable pass verbs (now a declared schedule, section 4); the atomics-free
sorted grid; the Upload/Update allocation split (now typed). The full v1 keep/kill adjudication belongs to
document 06.

---

## 3. The step model

v1 had no concept of a step -- the substep loop lived in the sandbox. v2 makes the step the engine's central
object: deterministic, declared, inspectable.

`dt` is fixed and immutable at creation (a C5 requirement). A `Simulation` owns a `WorldSet` and steps it with
`step(n_steps)`. Substep count is per-world config; the effective substep dt is `dt / substeps`.

**Determinism obligations that hold for everything below `Simulation`:**

- No wall-clock anywhere. Not for timing, not for seeding, not for adaptive anything.
- Every stochastic element -- Dryden turbulence, sensor noise, spawn randomization -- draws from per-world,
  per-system splitmix64 streams, domain-separated from the world seed. This is the errata-`R4` discipline,
  restated here as engine-generic.
- Structural changes (spawn, despawn, model load) are **queued** and applied only at step boundaries.
- No allocation inside a step.

**The tick counts steps, not substeps.** `Simulation::step()` owns the increment, once per step, after the last
substep. See `A9` in section 4.

---

## 4. The substep schedule

**The schedule is TEN passes, in this order, always.** This is the operative shape and it is already built
(`spade/engine/physics/schedule.hpp`, `kSubstepPassCount = 10`).

```
// per substep, in this order, always:
MediumUpdate         // Dryden gust states advance (seeded, per world)
BehaviorsKinematic   // SL6 slot -- declared kinematic behaviors (pose writes)
ForceElements        // rotors -> drag -> lift surfaces (each element type = one batched pass)
BehaviorsForce       // SL6 slot -- declared force behaviors (wrench accumulation)
Gravity              // REPRESENTED BUT INERT -- see A9
CollisionStatic      // dynamic proxies vs world SDF: distance + gradient -> contact impulses
CollisionDynamic     // sorted-grid pair path (exact cell compare -- v1 hash-collision bug fixed)
Integrate            // symplectic Euler + quaternion exp-map; applies gravity; captures specific force
SensorSynthesis      // only on sensor-rate boundaries; writes sensor output rings
Publish              // snapshot ring hook, frame-state copy point (S6); NOT the tick increment
```

Every pass is one interface with two implementations -- `execute_cpu(WorldSpan)` and
`record_gpu(CommandRecorder&)` -- same math, same operation order, fp32 both (`P1`/`P2`). The schedule object is
data; **passes cannot reorder themselves**, and there is no API to add, remove or reorder one. The order is a
parity contract, not a preference.

### `SL6` -- how eight became ten

The approved ninth spec described **eight** passes and stated that they are fixed at compile time with no API to
add, remove or reorder one. The library-and-sandbox spec's `SL6` (user ruling, 2026-08-30) grew the array to
**ten**. `SL6` is the later operative layer and amends this document's schedule; the "fixed at compile time, no
API to add/remove/reorder" rule is *intact* -- that rule is precisely why the slots are fixed array entries
rather than runtime insertions.

`SL6`'s argument: "Behavior" had no precedent anywhere in the corpus, and its naive form is incompatible with
guarantees the engine is built on. A Unity-style MonoBehaviour -- arbitrary user code, arbitrary order,
per-frame -- violates the fixed schedule, the no-wall-clock rule and the two-implementations rule at once, "and
would not fail loudly. It would quietly destroy CPU-GPU parity, which is the property the entire GPU program was
built to establish." So a behavior does not declare a schedule position; it declares which of **two fixed
slots** it runs in.

**Both slot positions are parity requirements, because float addition is not associative.**

- **`BehaviorsKinematic` -- after `MediumUpdate`, before `ForceElements`.** A pose written by a kinematic
  behavior must be set before *anything* reads it, and both collision passes read poses. Placing the slot later
  would let a body collide against the position it held last substep.
- **`BehaviorsForce` -- after `ForceElements`.** Behavior wrenches accumulate *after* the built-in
  rotors-then-drag order that the golden corpus already pins. Running before would change `force_acc`'s last
  bits for every existing element, and the corpus would go red for a change that added nothing to the physics.

**A behavior declares at registration:** a stable name; which of the two slots it runs in; the components it
reads and writes; and an `execute_cpu` implementation. It inherits the determinism obligations rather than
renegotiating them -- no wall-clock, randomness only from the existing per-world, per-system splitmix64 streams
under `R4` domain-separation, no allocation inside a step, structural effects queued to step boundaries.

**GPU eligibility is a refusal, not a fallback.** A behavior with no `record_gpu` is CPU-only and must declare
itself so. A world containing a CPU-only behavior is not eligible for the GPU-authoritative path, and this is
refused rather than silently downgraded -- the parity corpus must never be able to include a world whose
behavior did not run identically on both backends.

*Cost if wrong:* declared read/write sets that are too coarse make behaviors serialize more than necessary -- a
performance cost. The reverse error, permitting undeclared access, is silent divergence. That asymmetry is why
the declaration is mandatory rather than advisory.

### Inertness is a standing proof, not a one-time observation

**Both slots are inert whenever no registry is attached.** The dispatch is a null check:

```
void pass_behaviors_kinematic(const SubstepContext& ctx) noexcept {
    if (ctx.behaviors != nullptr) {
        ctx.behaviors->run_slot(objects::BehaviorSlot::kinematic, ctx);
    }
}
```

**Every scenario in the golden corpus attaches no registry.** So both passes remain the empty passes that were
proved byte-identical when the slots landed, and the corpus re-proves that inertness on **every run** -- rather
than the proof having been a one-time observation at merge time. That is what makes `SL6`'s obligation ("with no
behaviors registered, every existing golden must remain byte-identical") testable rather than asserted. It is
the same pattern the deliberately-inert `Gravity` slot already establishes, for a different reason.

### `A9` -- two corrections to the approved shorthand, ratified

Two corrections implemented in code became the spec (2026-08-09 verification review, `VI-5`):

1. **The tick counter increments once per step, not in the per-substep `Publish` pass.** A literal reading of
   the approved shorthand would tick `substeps` times per step and make every snapshot header, every replay
   resume point and every structural-queue boundary disagree with the API's own step count. `Simulation::step()`
   owns the increment.
2. **The `Gravity` pass is deliberately inert.** Gravity is applied inside `Integrate` (`vel += (force_acc/mass
   + g) * h`) so that the specific-force capture for the IMU is exact rather than a subtraction. A literal
   `Gravity` pass would double-apply. The slot is retained for schedule-shape parity with the approved list and
   must **not** also accumulate `m*g`; a test asserts the emptiness (a body under gravity falls by exactly one
   g, not two).

`S6`'s GPU mirror implements the landed schedule, not the shorthand.

> **STALE:** `A9` and `A1` cite `schedule.hpp` line ranges (`:328-338` for the tick correction, `:265-282` for
> the inert Gravity slot, `:310-324` for the `SensorSynthesis` pass). Those citations predate `SL6`: inserting
> the two behavior slots shifted every line below them, and the ranges no longer name what they named. Verified
> on the box -- `schedule.hpp:265-282` now lands on `SL6`'s own slot-position comment and `kSubstepPassCount =
> 10`. Cite the pass by **name**, not by line.

### `Publish` -- why an empty slot stays

`Publish` is a no-op stub today. Of its three declared jobs, the tick increment moved to `Simulation::step()`
(above); the snapshot ring is the editor's continuous-rewind buffer, a GPU-path object arriving with the Vulkan
backend; and the frame-state copy point is the render/readback seam, likewise `S6`. It stays in the schedule
because it is the one place those hooks may ever attach, "and having them attach at a declared point is the
difference between a schedule and a call sequence."

---

## 5. State, many-worlds, snapshot and readback

### State layout

Authoritative state is **SoA arenas partitioned by world**: bodies, elements and sensors occupy world-contiguous
slot ranges inside global arrays. Per-world parameters (gravity, medium, capacities, seed) live in an indexed
param buffer. A slot-to-world map buffer lets one dispatch cover all worlds.

**Fixed capacity per world at creation** (bodies, elements, sensors, contacts). Spawn and despawn allocate and
free slots deterministically in call order within capacity -- no reallocation mid-run, ever. v1's
"load-then-never-again" becomes an explicit, honest contract instead of a silent overrun.

**The ECS carries identity, not physics.** Generational handles (index + generation, reuse-safe -- v1 had no
lifecycle at all); components reference state slots. Pool iteration order no longer feeds GPU buffer order;
**slot assignment does**. That clause is load-bearing and document 04 makes it permanent: the object graph never
influences buffer layout, pass order, or any value the physics computes. Component type IDs are
compile-time-registered monotonic (v1's dead `GetUniqueComponentID` pattern, resurrected), never
`typeid().hash_code()`.

Static world geometry -- the SDF program -- is immutable per world in its own buffer; rebuilding it is a
structural change.

### Many-worlds (`D8`)

The collision grid hashes `(world_id, cell)`; sorted domains are global; range checks make cross-world
interaction **structurally impossible** rather than merely unlikely. One dispatch steps N worlds; N=1 is the
degenerate case with no overhead.

The CPU twin iterates worlds serially with identical per-world math, so parity comparisons are per-world and a
divergence **names its world**.

A `WorldSet` template ("N worlds from this world file, capacities X") is the training-fleet constructor.
Heterogeneous sets are allowed; uniform sets get the tightest layouts.

Batching invariance is a test, not a hope: the same world stepped alone must produce identical results to that
world stepped inside a set (section 12).

### Snapshot (`P8`) and readback

The state registry records every authoritative array -- name, element type, per-world extents. A snapshot is a
registry walk into a versioned blob: header (schema hash, tick, world-set shape) + arena copies + RNG stream
states. The CPU path is a memcpy. Restore is the reverse plus a structural-queue flush. Blobs save and load to
file.

**GPU path.** An on-demand snapshot is one arena-to-staging copy at a tick boundary (single `vkCmdCopyBuffer`,
fence, map). The editor's continuous rewind ring is async double-buffered staging copies every K ticks, never
blocking the physics queue.

**Sensors solve the per-tick readback problem.** Sensor outputs are tiny; they live in a dedicated output buffer
read back per tick (double-buffered). Full-state readback happens only on demand. Nothing else crosses the bus
per tick.

> **STALE:** the approved text says the determinism-replay corpus is "snapshot pairs + input scripts ... data,
> not code". `A11(iii)` records that it actually landed as **builder-scenarios plus committed digests**, and
> requires `S5`'s exit proof to absorb the substitution explicitly. The substitution is real and unretracted;
> the approved sentence describes an artifact that does not exist in that form.

### `A3` -- reset is restore plus reseed (user-ruled 2026-08-10)

`kathost_reset` is the restore of an initial snapshot taken *after roster assembly* (so the roster is preserved,
per `B12`), **plus** one new engine surface -- e.g. `Simulation::reseed(seed)` -- that re-derives all RNG
streams from a fresh seed after the restore, using `R4`'s domain-separated derivation.

*Why the second half is necessary:* `WorldParams::seed` is registered state inside the blob. Restore alone
reinstates the **old** seed, so a reset without reseed replays the same noise. The landed `restore()` semantics
are ratified as correct (structural queue discarded; config and `dt` not in the blob). dronesim, as the C5
implementer, must capture the post-roster snapshot at session start. Conformance: a "reset preserves roster"
round-trip case and a "reseed re-derives streams deterministically" case join the section 12 suites.

### `A4` -- the poll cursor split, ratified

Every per-sensor cursor-bearing state -- ring write cursor `last_index`, bias states, RNG stream, phase counter
-- is **registered state inside the world snapshot blob**; `poll(sensor, since_index)` is caller-carried and
stateless engine-side.

It follows that the host-side consumed-sample cursor -- `B13`'s "poll cursor" at the C5 level -- belongs to the
world-snapshot layer owned by dronesim (`R9`'s world/runtime/session split): the host snapshots its own
`since_index` values alongside the engine blob. The expectation that `B13` would land inside `S2`'s snapshot
round-trip suites is re-scoped: the engine-side obligation landed with its own tests (ring wrap, poll-since,
snapshot resume); the host-side cursor case lands in the C5 conformance suite.

---

## 6. Physics content

### Rigid bodies (`D1`, `D2`)

State per body: position, velocity, orientation quaternion, angular velocity (body frame), mass, inverse inertia
tensor (body frame), force and torque accumulators, flags. The layout is defined once in `shaders/shared/`
(`D9`, section 9).

Integration is semi-implicit Euler: velocities first from the accumulated wrench, then pose. Orientation
advances by `q * exp(0.5 * omega * dt)`, exact for constant omega over the substep. Euler's rigid-body term
`omega x (I omega)` is included. The quaternion is renormalized per substep, in a deterministic operation order.

**Specific force** -- acceleration minus gravity, body frame -- is captured *inside* `Integrate` for the IMU
pass. Computed once, not reconstructed. This is the reason the `Gravity` slot is inert (`A9`) and the reason
`SensorSynthesis` runs *after* `Integrate`: running it before would sample the previous substep's acceleration.

### Collision (`D3`)

The static world is an **analytic SDF program**: primitives (plane, box, sphere, cylinder, capsule, torus,
heightfield) + transforms + CSG ops (union, intersect, subtract, and smooth variants), compiled by the
`WorldBuilder` into a flattened postfix node program. Both backends evaluate the identical program (section 9's
single-sourcing).

Contact: `phi(p) < r` gives depth; the normal is `grad phi` -- analytic where a closed form exists, central
differences otherwise, with a **pinned stencil** so the two backends difference the same way.

Race gates are box/torus compositions -- exact geometry, zero mesh error. Contact response is impulse-based with
restitution and Coulomb friction (v1's model, kept), with positional correction clamped.

**Dynamic-dynamic** ports v1's sorted-grid pipeline. The atomics-free design is kept -- "it is the engine's best
idea" -- with three fixes: exact cell-ID compare (v1's hash-bucket-neighbors bug), a bounds-checked bitonic
tail, and hash size configurable per world set rather than a constant duplicated four times.

Dynamic proxies in v2 are sphere and capsule. Convex hulls and triangle-mesh statics are the stated later rungs.

### Medium and aero (`D5`, `D6`)

The `Medium` interface is `sample(world, position) -> {density, wind_velocity}`. The v2 implementation is
per-world density + constant wind + Dryden turbulence (standard MIL-spec form, per-world seeded filter states
advanced in `MediumUpdate`). `P7`'s flow fields implement the same interface later -- **aero code never knows
which medium it reads.**

The `MediumUpdate` pass draws a fixed number of gaussians per call whatever the sigmas are, and does not branch
on a zero-sigma world. Skipping the advance for a calm world would make two runs of the same scenario at
different turbulence levels diverge in every *other* stochastic system as well, because the stream position
would stop being a function of substep count alone.

Aero force elements read the `Medium` at their body-frame station:

- `RotorElement` (section 7).
- `DragBody` -- quadratic and componentwise, matching what current Kat configs express.
- `LiftSurface` -- flat-plate plus polar tables; the fixed-wing seat, post-v2.

---

## 7. Vehicles -- the model-type layer (`D4`)

A `ModelType` is an engine asset class one tier above shapes and meshes: a rigid-body template (mass, inertia,
proxies) + a force-element set + sensor mounts + visual mesh references + a parameter schema. Instances spawn
from a `ModelType` into slot allocations. The Quadrotor is the first `ModelType`. Cars and fixed-wing arrive as
new element types and new model classes -- **nothing vehicle-specific exists below `vehicles/`.**

### `RotorElement` (`D5`)

```
// per rotor, SoA, batched across all worlds:
params: body slot . local pose . spin dir . time-constant tau (RPM lag)
        thrust curve T(w) . torque curve Q(w)        // polynomial or table
state:  w (RPM)                                      // first-order lag toward command
step:   w += (w_cmd - w) * dt / tau
        v_axial = axis . (v_body + w_b x r - medium.wind)
        T = T(w) * f_inflow(v_axial)      // momentum-theory correction:
                                          // climb/descent/hover thrust variation
        T *= f_ground(phi_world(p_rotor)) // ground effect from the WORLD SDF --
                                          // the collision field IS the proximity query
        apply wrench {T*axis, Q(w)*spin + T*axis x r} at body
```

**Quadrotor API:** a parameter struct (rotor poses or an arm-geometry shorthand, mass and inertia, per-rotor
params, drag body, sensor mounts, visual mesh ref) plus a factory returning a `ModelType`. Convenience accessors
take command vectors (per-rotor `w_cmd`, or normalized `u`). This is the class Kat's dronesim maps drone
descriptions onto.

Gyroscopic rotor terms and BEMT are per-rotor opt-ins on the roadmap; **the parameter schema reserves their
fields now, so upgrading a model never changes layout.**

**Aero roadmap (`P7`).** BEMT per rotor, then effects-layer flow fields (rotor wake, downwash) feeding
`Medium.sample` as local inflow. Promotion of an effects field to authoritative requires the CPU twin mirroring
it -- the charter's promotion rule, restated here because it guards this exact path.

---

## 8. Sensors

Sensor tables are SoA and batched: kind, body slot, mount pose, rate divider, seeded noise params, output ring
reference. Synthesis runs **inside the step**, in the `SensorSynthesis` pass, only on that sensor's rate
boundary.

**IMU (v2).** Gyro = body angular velocity at the mount, plus lever-arm terms. Accel = the specific force at the
mount, transformed by the mount pose. Noise = white plus bias random walk per axis, from per-sensor seeded
streams. Output is timestamped samples into the sensor's ring.

**Output rings.** Monotonically indexed, tick-stamped samples in the per-tick readback buffer. The host polls
rings (`kathost_sensor_poll`); ring depth is at least the host's worst poll interval; the count / latest-index
convention follows `TA5`. Spade exposes samples and indices; **stamping policy above the seam belongs to
dronesim** (`P5`).

### `A1` -- camera-sensor placement (ratified as the schedule already leans)

**Camera-sensor synthesis executes inside stepping, in the schedule's reserved `SensorSynthesis` pass**, exactly
as `B1` requires.

> **SUPERSEDED by `A1`.** The approved body read: "a camera sensor is a render-to-texture at sensor rate **on
> the render queue**, output = image + tick." That camera-on-the-render-queue reading is superseded for sensors.
> It is struck, not softened.

What survives from the superseded text is the distinction, which stands: **camera sensors are not presentation
frames.** Sensor frames are simulation outputs and must exist headless; presentation frames are for humans and
stay on the render path (the `TA1` FramePool). Keeping the two frame kinds separate is what keeps render-cadence
invariance and headless sensor fidelity simultaneously true.

When the rendered camera lands, its work is dispatched from *within* the `SensorSynthesis` pass (`record_gpu`),
not from the presentation queue. Adding it **adds no pass** -- the slot already exists -- but it does change the
snapshot schema hash and the golden digests when camera state registers. That is declared-schedule discipline:
re-record per section 12.

### `A6` -- the camera cost model (model now, numbers at stage entry)

Rendered camera cost is approximately `n_drones * (W*H) * k_raymarch(SDF node count)` per sample at 25 Hz. The
synthetic-feature path (`A8`) is approximately `n_drones * n_gates_in_frustum * k_sdf_query` -- negligible
against `R10`'s envelope.

The stage entry gate benches the rendered model against the 3-drone x 25 Hz envelope; a miss triggers design
review under section 12's bench-miss rule, with the resolution options (resolution or rate reduction, GPU-only
camera) recorded at that point.

*Note:* `A6` names "S7's entry gate". Under the S7a/S7b split (document 03), the rendered camera lands with S7b,
so the gate attaches there.

### `A8` -- M1d camera vs engine S7 (user-ruled 2026-08-10)

The 25 Hz camera lane is satisfied by **host-synthesized analytic gate-feature buffers** derived from the SDF
world geometry -- `B1`'s fake-camera architecture promoted to the real host. That lane proves cadence and
package plumbing, **not pixels**. The rendered camera arrives with the render stage and is not on the critical
path, so the sequencing conflict dissolves. Transport of the feature buffer to the consuming component is
`B15`'s sensor-input ruling.

---

## 9. Compute backends, Slang, parity, frames

### Single-source layouts (`D9`)

All shared state and param structs, and the binding registry, are authored **once** as Slang modules under
`engine/shaders/shared/`. A build step (slangc reflection) generates the C++ headers with `static_assert`ed
offsets and sizes. v1's two worst layout hazards -- the nine-file struct duplication and the GLM
quaternion-order coincidence -- become build errors instead of silent corruption.

Kernels are Slang, compiled to SPIR-V at build. Pipeline layouts and descriptor bindings come from the generated
registry. **No string-name uniform lookups**; push constants and param buffers only.

### Vulkan backend structure

`compute/vulkan/` owns instance, device and queues: one compute queue for physics, one transfer queue for
readback and snapshot, and (with the renderer) one graphics queue. **Headless by construction** -- no surface is
required for physics or for camera sensors.

Command recording is per step: passes record into one command buffer per step batch, and barriers between passes
are explicit and minimal. v1's barrier-after-every-dispatch becomes stage-scoped barriers; the bitonic sort
records its full dispatch chain once per shape and reuses it.

Timestamp queries per pass feed the bench harness, so the budget numbers stop being guesses.

### Parity mechanics (`P1`/`P2`)

A **golden scenario corpus** (world + inputs + N steps) runs on both backends. Comparison is per state array,
with per-quantity tolerance classes (positions, velocities, quaternions, RPM).

The three determinism grades this comparison is graded against are **owned by `01-charter.md` section
4** and are deliberately not restated here -- they were stated four times across the source
documents, and that duplication is what let them drift.

Tolerances are **measured, then pinned with margin** -- calibrate cross-platform before pinning is standing
policy, learned the hard way. A band is never widened to accommodate a new pass: `SL6` restates the same rule
for behaviors, since "widening a parity band to accommodate a new pass would compromise the corpus for every
existing one."

### `A7` -- the backend config surface (discharges config section 18)

The accreted create-time surface is partitioned into three:

**(a) Simulation content** -- result-affecting, versioned, authored in world/sim config: `dt_ns` and `substeps`,
`ContactParams`, `GridParams.cell_size` (order-affecting through the sorted-grid comparator), `DrydenParams`,
`WorldSetDesc`.

**(b) The opaque backend block** -- **result-neutral by contract**; it may not change stepping:
`uniform_dynamic_params` batching selection (proven byte-identical by the landed tripwire), capacities and pool
sizing, device and thread-count selection (thread counts additionally bound by `R10`'s bit-stability sweep).
**The block's constitutional rule: CPU-vs-GPU path selection is an implementation property (`P2`), never a
tunable.**

**(c) Not configurable:** pass-schedule shape, integrator operation order, publish semantics.

**Conformance rule:** every knob admitted to the backend block owes an invariance test in the tripwire pattern
-- digest equality across that knob's settings. The landed tripwire is the template.

### Presentation frames (`TA1` compliance)

> **SUPERSEDED BY WHAT SHIPPED (`C10`).** The paragraph below is the approved text and it no
> longer describes the engine. **S7a added no engine-side frame pool** (`RS1`), and verification on
> 2026-09-07 found none: `spade/engine/render/` holds `raster_cpu`, `raymarch`, `scene`, `gltf`,
> `csg_mesh`, `agreement` and nothing pool-shaped. The only `FramePool` in the tree is a
> compute-side concept in `engine/compute/vulkan/context.hpp`, which is a different thing. The
> `B10` obligation is met at the C5 seam instead -- see `03-world-and-render.md`. Kept because
> `A2`'s deferral reasoning below is still the record of why.

The render module exposes a `FramePool`: explicit `render(world, camera, target)`, at least two outstanding
frames, non-blocking acquire, every frame tagged with its simulation tick. Rendering reads a **tick-boundary
state copy on its own queue**, so render cadence cannot perturb stepping -- `TA1`'s render-cadence invariance
and the C5 headless/rendered equivalence test are satisfied by construction rather than by testing after the
fact.

**`A2` -- the `B10` frame-pool deferral, recorded.** The FramePool must satisfy `B10`: frames handed to the host
remain valid across all host calls; pool slots are **deferred, not recycled**, while frames are outstanding; at
least two outstanding are supported.

**`A5` -- the `B11` threading discharge.** v1 renders on the sim thread (the strangler viewer conforms). The
target remains a second render thread consuming only the FramePool, which satisfies `B11`'s threading clause.
**Degradation order: renders drop first.** Presentation frame production may be shed under load; **sensor
synthesis inside stepping is never shed.**

The renderer itself -- the scene model, the rasterizer, materials, shadows, and where the frame pool physically
lives -- is owned by document 03.

---

## 10. API surface and conventions

`namespace spade`; C++23 (`D12`).

**Errors.** The public API returns `std::expected<T, spade::Error>`; **no exceptions cross the boundary** --
v1's uncatchable private exception classes are the counterexample. `Error` carries a code plus a context string.
An error-sink callback surfaces async and GPU-side faults. Debug builds run the Vulkan validation layers;
device-lost and OOM paths are explicit.

**Handles.** Typed RAII handles everywhere -- `WorldHandle`, `ModelTypeHandle`, `BodyRef`, `FrameHandle` --
never bare integer aliases (v1's `GLuint` soup).

**No globals, and no process-wide teardown side effects** (v1's `glfwTerminate`-in-a- destructor). **N
`Simulation`s in one process is a supported, tested configuration** -- the charter's instance-isolation gap is
closed by construction rather than by policy.

**Threading contract.** A `Simulation` is externally synchronized (one caller thread). Internal GPU queues are
the engine's business. The render module may be driven from a second thread against the FramePool only (`A5`).

**Rule-of-five discipline** on every resource-owning type -- v1's `MeshComponent` double-delete landmine is the
counterexample. Moves explicit; copies deleted unless the type is value-semantic.

---

## 11. The rate envelope

Targets, validated by bench rather than promised:

- Single world: **>= 10 kHz substeps on CPU** for a quadrotor-class scene.
- Viewport (1 world, few vehicles): **1 kHz physics + 60 fps render concurrently**.
- Batched headless: **>= 64 worlds x 1 kHz, faster than real time** on the dev GPU.

**A miss triggers design review, not silent acceptance.** That rule is cited by `A6` for the camera cost model
and is the general disposition for every bench regression.

---

## 12. Testing and benchmarks (`D11`)

| Suite | Contents | Where it runs |
| --- | --- | --- |
| unit | math vs closed forms (exp-map, inertia transforms); SDF distances vs analytic; ECS lifecycle (create/destroy/reuse/generation); state registry round-trip; world file parse/validate | every commit -- box + CI (ubuntu CPU, windows build) |
| physics validation | ballistic drop vs closed form; energy behavior of the symplectic integrator; hover trim (thrust = weight => station-keeping); climb/descent thrust vs momentum-theory reference curves; Dryden spectral sanity | every commit (CPU) |
| determinism replay | golden corpus: world + inputs + snapshots; bit-identity per platform (CPU); N-world batching invariance (same world stepped alone vs inside a set => identical) | every commit (CPU); GPU per-device on the box |
| parity | CPU-GPU per-array tolerance bands over the same corpus | box, nightly-class |
| bench | google-benchmark: per-pass timings (Vulkan timestamps), steps/sec x worlds sweep, baselines JSON in-repo with regression thresholds | box; report artifact in CI later |

Cases added by Addendum A and by `SL6`:

- "reset preserves roster" round-trip, and "reseed re-derives streams deterministically" (`A3`).
- Ring wrap, poll-since, and snapshot resume for the sensor cursor (`A4`, landed); the host-side cursor case in
  the C5 conformance suite.
- A digest-equality invariance test for **every** knob admitted to the opaque backend block (`A7`).
- Byte-identical goldens with no behavior registry attached -- which is every corpus scenario, so this runs on
  every corpus run (`SL6`, section 4).
- Re-recording of snapshot schema hash and golden digests when camera state registers (`A1`).

---

## 13. Housekeeping and execution record

**`A10` -- execution-gate record.** The `S1`-`S4` plan executed under user go (2026-08-08, SDD ledger;
serial-to-parallel-waves amendment) with Addendum A outstanding; that waiver is recorded in the addendum itself.
`S1`-`S3` gates passed 2026-08-09 (ctest 277/277). The addendum was signed before the `S4`/M1B claim, restoring
the register's description of reality. None of the eight rulings was foreclosed by the landed code (2026-08-09
verification review, `VC-1`).

**`A11` -- housekeeping.**

1. Spade's own version advances 0.1.0 -> 0.2.0 at the `S4`/M1B claim, discharging the charter's undischarged
   version promise. `world_version` and the snapshot format version remain separately versioned.
2. The charter's gap-survey column is declared stale by `A11` itself -- see the marker below.
3. The determinism corpus substitution -- see the STALE note in section 5.
4. A stale `A5` citation was reported at `spade/engine/sensors/imu.hpp:268`, to be fixed by a one-line comment
   change. **Verified on the box: that line now reads `SampleIndex index; // ... (editor tech spec TA5)`, and no
   bare `A5` citation remains in the file.** The reported defect was a misread of `TA5`; the item is discharged
   either way.

> **STALE (declared by `A11(ii)`):** the charter's gap-survey column. Fixed-step, headless, seeded RNG,
> rigid-body dynamics, snapshot/restore and the test estate all exist on the CPU path; propulsion and sensors
> landed. The charter column still describes them as gaps.

---

## 14. Traceability

Charter items answered here: the many-worlds model (section 5); buffer and state layout, readback and snapshot
(sections 5, 9); the rate envelope (section 11); the sensor primitive set (section 8); the Vulkan module
structure and frame handoff (section 9); the Kat build wiring (section 2); the effects-to-aero promotion
criteria (section 7, restated). Linux: the CI ubuntu CPU job is the first Linux surface; full Linux GPU support
rides the Vulkan backend, scheduled when cloud training needs it.

Three calls embedded in the approved design were approved in-session 2026-08-08: the YAML world file, keeping
Dear ImGui, and the SDF-raymarch debug view.

Addendum A maps to amendment batch 2 as: `B1` -> `A1`, `B10` -> `A2`, `B12` -> `A3`, `B13` -> `A4`, `B11` ->
`A5`, the camera cost model -> `A6`, the opaque backend config block -> `A7`, the M1d-camera-vs-render-stage
conflict -> `A8`, the schedule corrections -> `A9`, the execution-gate record -> `A10`, housekeeping -> `A11`.

**Pointers beyond the front-matter split.** The S7a/S7b question is document 03's; the behavior registry `SL6`
registers into is document 04's; `kathost_reset` and the host-side poll cursor are document 05's; the v1
transfer register is document 06's.

