# Module API, scheduler and fields — design

**Owner:** Core. **Status:** approved by the user 2026-10-02 (as of `4d481eb`); amended by §15, approved as principles 2026-10-05 (user, via lead). The plan is `2026-10-02-module-api-plan.md`.
**Builds:** `../../01-engine-model.md`'s foundation (`../../backlog.md`, step 3). **Rulings this rests on (user, 2026-10-02):** scope A (Q1), sample buffers (Q2), kinematic behaviors first in Fields (Q2), `CORE-4`.
**Inputs:** Physics' requirements (`../../physics/plans/2026-10-02-module-api-requirements.md`); Interface's open items (`../../interface/01-editor.md`); Rendering's field channels (`../../rendering/01-techniques-and-channels.md`).

## 1. Goal and done-when

Today's engine runs a fixed array of ten passes, and the GPU keeps a hand-written copy of it. This step makes the pass list come from **modules**: Spade's built-ins and a developer's modules plug in through one API, a scheduler places their passes in the engine model's six phases, and the GPU chain is derived from that schedule.

**Done when:**
1. Today's passes run as built-in modules through the new scheduler, on both backends.
2. Every golden digest is unchanged, and every parity band holds without widening.
3. The GPU dispatch chain is derived from the schedule; no hand-kept pass table remains.
4. A held drone runs on Vulkan: the translation lock replaces the drone box's CPU behaviors.

## 2. Scope

| Built in this step | Specified here, built later |
|---|---|
| Module descriptor, module set, registry | Regions other than the whole world (with SPH, `PHY-4`) |
| Six-phase scheduler, ordering rules, schedule in the config hash | Objects in stepping, and world-file v3 (with the editor, `INT-3`) |
| GPU chain derived from the schedule | GPU kernels supplied by developer modules (a GPU module ABI) |
| Field registry; gravity, density, wind and Dryden behind it; sample buffers | Field channels for cameras (with Rendering) |
| Grades per module and backend; the grade check at `create()` | SPH (`PHY-4`), Jacobi as a contact alternative (`PHY-5`) |
| Roles (mutually exclusive alternatives) | Swapping a module on a running world (the editor rebuilds instead, `INT-2`) |
| The translation lock, a built-in module on both backends | |
| Sensors as modules: the IMU/GNSS pairs collapse (`CORE-4`) | |
| Component-type availability (`fluid` shown as reserved) | |

The public API stays slot-based: `spawn`, `add_drag_element`, `add_imu_sensor`, `add_gnss_sensor`, `poll_imu`, `poll_gnss` keep their signatures. They become thin calls into the module path.

## 3. Modules

A **module** is a descriptor plus functions. It declares:

- **Identity:** a name and a version. Both go into the configuration hash.
- **State:** the arrays it registers. Each array has a name, an element type, and an extent: one per world, per body slot, per element, per sensor, or per sensor ring. Registered state is snapshotted, digested and mirrored to the GPU with no further work by the module.
- **Rows attached to bodies:** for state that belongs to a body (a drag element, a sensor, a rotor, a lock), the module provides `init` and `free` functions. The core runs them from the structural queue, and runs `free` when the body is despawned. This replaces the per-kind cascade in `Simulation`.
- **Seeded streams:** each `rng::Stream` it derives from the world seed, with its domain tag. `reseed()` walks these declarations, so its list of streams is no longer kept by hand (the lesson of the 2026-10-02 GNSS reseed defect).
- **Fields:** the fields it provides (with the provider) and the fields it reads (with its sample points; §6).
- **Passes:** each pass names its phase, what it reads and writes, any explicit ordering edge (§4), a CPU function, and optionally a GPU recording (§5).
- **Role:** optionally, the role it fills (§7).
- **Grade:** one per backend (§8).
- **Component types:** the component types it defines. A component type no module defines is **reserved**, with its reason, and cannot be constructed. `fluid` is reserved ("SPH, `PHY-4`") until the SPH module exists. Every enumerator of component types (a menu, a loader, a listing) reads this availability (Interface's open item).

**The module set.** `Simulation::create()` takes a module set; the default is `standard_modules()`, which is exactly today's engine. The set is shared by every world in the simulation (`L8`). It is configuration: its names, versions and role choices are hashed, and a snapshot restores only into the same set (`L2`). Optional built-ins, such as the lock, are added explicitly, so a world that does not use them registers nothing for them.

**Keeping the goldens' walk.** A digest folds the registry walk in registration order, so the standard set must register today's arrays in today's order: `world_params`, `bodies`, `body_generation` (core), then `drag_bodies`, `dryden`, `imu_sensors`, `imu_ring`, `rotors` (modules), then `replay_config` (core), then `gnss_sensors`, `gnss_ring`. Modules register in set order; the four modules that predate `replay_config` carry a legacy marker that registers them before it. No other module may carry the marker. A future snapshot-format version may drop it.

**Behaviors** keep their registry and `set_behaviors()`. Their two slots become phase positions: kinematic behaviors run **first in Fields**, before any provider samples (Q2); force behaviors run **last in Forces**. A Vulkan step still refuses an attached registry (`CORE-1`), until behaviors can carry a GPU recording.

## 4. The scheduler

Each substep runs six phases in order: **Fields → Forces → Constraints/Contacts → Integrate → Sensors → Publish** (`../../01-engine-model.md`).

- **Placement.** A pass runs in the phase it names. `create()` compiles the module set into one ordered pass list.
- **Access kinds.** A pass declares each quantity it touches as **read**, **write** or **accumulate** (adds to it, never clears it, as rotors and drag do to `force_acc`).
- **Order inside a phase**, in priority:
  1. **Hazards.** A pass that only reads a quantity runs after every pass that writes or accumulates it. Two passes that both write a quantity (a read-modify-write counts as a write) must be ordered by an edge, and so must a writer and an accumulator; otherwise `create()` refuses them as ambiguous. Two accumulators of the same quantity are not a hazard.
  2. **Ordering edges.** A pass may declare `after: <module>.<pass>`. Edges pin an accumulation order, because fp32 addition is not associative: drag declares `after: rotor.forces`. Without that edge, set order would put drag first (§3's walk order) and the goldens would move. They also order two writers: dynamic contact declares `after: contact.static`, since both correct `pos` and `vel`.
  3. **Set order.** Ties fall back to module-set order, then declaration order.
- **Behavior positions.** Kinematic behaviors run first in Fields, and force behaviors last in Forces, by position, after every accumulator. Integrate reads `force_acc` after all of them.
- **Refusals.** A cycle, or a pass that names a phase or an edge that does not exist, makes `create()` return `invalid_argument`.
- **The order is configuration.** The compiled list of module and pass names goes into the configuration hash, so a restore under a different order is refused.
- **Derived structures.** A module may build a structure that is not registered state (scratch), for example the sorted grid. Other modules read it through a declared read. The same structure may be built more than once in a substep, as one build pass per use. SPH will build the grid in Fields while dynamic contact keeps its own build in Constraints/Contacts, because static contact moves positions in between (Physics' requirement). This step keeps the one existing build inside dynamic contact.

**Today's passes, compiled.** The standard set must compile to exactly today's order:

| Phase | Passes, in order | Was |
|---|---|---|
| Fields | kinematic behaviors; Dryden advance; field sampling (§6) | BehaviorsKinematic and MediumUpdate swap; neither reads what the other writes, and no golden attaches a behavior |
| Forces | rotor forces; drag; force behaviors | ForceElements; BehaviorsForce |
| Constraints/Contacts | static contact; dynamic contact (grid build, sort, resolve) | CollisionStatic; CollisionDynamic |
| Integrate | integrate (honours the lock, §9) | Integrate. The empty Gravity slot goes: gravity is a field Integrate reads |
| Sensors | IMU; GNSS | SensorSynthesis |
| Publish | nothing yet (§10) | Publish |

## 5. The GPU chain

`StepRecorder` stops keeping a pass table. It walks the compiled schedule for each substep, and asks each pass with a GPU recording to append its dispatches: kernel, grid extent, push constants, and for chains such as the bitonic sort, one dispatch per stage. Barriers follow the emitted list (`CORE-2`).

- **Built-in kernels only, this step.** Built-in modules' kernels and buffers keep the generated binding registry (`bindings.slang`). A developer module has no GPU recording yet, so it is CPU-only, and a Vulkan world that contains it is refused at `create()` (`L6`). The GPU module ABI (bindings for module-owned state, kernel embedding) is later work.
- **Timings are per pass.** `PassDurationsNs`' fixed fields become durations keyed by pass name. The bench keeps its recorded names by mapping them.
- **A pass with no GPU recording** is refused on Vulkan unless its module declares the pass inert on that backend. Today's only such passes are the behavior slots (`CORE-1`).

## 6. Fields

- **The registry** names each field, its value type and its unit: `gravity` (vec3, m/s²), `density` (float, kg/m³), `wind` (vec3, m/s). A provider declares which fields it supplies; a reader declares which it reads.
- **Providers this step.** `gravity` and `density` come from the world's environment (constant). `wind` comes from Dryden: the mean wind plus the current gust, so turbulence level `none` is the constant case.
- **Regions.** This step builds only the whole-world region. Every field has one provider per world. The full region model is §11.
- **Sample buffers (Q2).** A reader declares its sample points as per body, per element (a rotor's position) or per world, and reads `sample(field, point)`. The per-world capacity follows from that declaration (the body capacity, the element capacity, or one), so nothing new is authored.
  - **Readers never know the provider's kind.** The registry decides how a point's value is stored. A provider that declares itself position-independent is sampled once per world, and the registry answers every point in that world with that one value. That is every provider this step, so today's costs and values stand. A position-dependent provider (SPH) fills one value per point. The reader's code is the same either way.
  - In the Fields phase, after the kinematic behaviors, each provider writes its samples, and readers read them in later phases.
  - Points are the positions after the kinematic behaviors, before contact and Integrate move anything. That is the instant `Medium::sample` reads today.
  - On the GPU, each provider has one sample kernel. No reader's kernel knows which provider answered.
- **Goldens.** Dryden's sample is the same operations on the same inputs, now stored; rotors, drag and Integrate read stored values bit-identical to what they computed inline. Gravity is today's `WorldParams::gravity`.
- **Host reads.** `Simulation::sample_medium(world, pos)` calls the provider's CPU sample function on the current state, the same path §10's camera samples use. It does not read the step's buffer. Before the first step it returns the mean wind plus the initial gust, as today, and it can answer any point once a provider depends on position.
- **A provider with state** reads its own registered state and any derived structure it declares (SPH's neighbour sums over the grid). Its sample kernel binds those buffers.

## 7. Roles

A **role** is a slot that exactly one module in the set must fill: `dynamic_contact` (today Gauss-Seidel; Jacobi later, `PHY-5`), and later an aero tier (T0–T3).

- The role choice goes into the configuration hash.
- Each alternative declares its own grade and carries its own golden and bands. Choosing an alternative never re-pins another alternative's numbers.
- A set that fills a required role twice, or not at all, is refused at `create()`.
- A scenario names its choice, defaulting to the standard set's.
- *Clarified 2026-10-03 (`../../plans/2026-10-03-drone-builder-engine-design.md`):* a tier that is only data, such as propeller CT and CQ tables, is per-row data inside its module, not a role. A role is for a tier that changes passes or state, such as blade-element (BEMT) aero.

## 8. Grades

- **Declaration.** Each module declares a grade per backend: `reference`, `banded`, `best-effort` or `absent`. `absent` means there is no implementation on that backend; Physics' "no GPU path" for SPH is the first real use (`PHY-4`). A Vulkan declaration of `reference` is refused at registration, because the GPU is never a golden source (`L4`).
- **A simulation's grade** on its backend is the lowest grade in its module set.
- **The check.** `create()` takes an optional minimum grade. A set below it is refused with `Code::unavailable`, naming the module that fell short (`L3`, `L6`). An `absent` module on the chosen backend is always refused. With no minimum given, any grade except `absent` is accepted, so existing callers are unaffected.
- **The built-ins' declarations** are Physics' and Rendering's to make (`../../physics/04-verification.md`). Core supplies the mechanism and the check.

## 9. The translation lock

**Decision:** a built-in **lock** module, honoured by Integrate (Physics' option A). It is not a projection pass in a new phase. **Why:** Integrate is already the one writer of `pos`, `vel` and `specific_force`, so the lock stays one site per invariant, and the signed engine model needs no new phase.

- **State.** The lock module registers one row per body slot: an anchor (vec3) and an axis mask (world x, y, z). It is not in the standard set, so worlds without a lock register nothing and keep their walk.
- **API.** `lock_translation(body, axes)` and `unlock(body)`. Both are queued to the step boundary. The lock row is a body-attached row (§3), so despawning the body frees it. The anchor is the body's position when the lock is applied. **If the lock module is not in the set, the call is refused with `Code::unavailable`** (`L6`): a world that asks for a lock and cannot provide one is never left silently unlocked.
- **Integrate.** It computes today's update exactly as now, then selects per locked axis: `pos = anchor`, `vel = 0`, and specific force `s = −g` on that axis (free axes keep `force_acc / m`), with `specific_force = Rᵀs`. The unlocked arithmetic is not restructured, on the CPU or in `integrate.slang`, so worlds without a lock compute today's bits. A held body's IMU reads `−Rᵀg` (+1 g "up") from any mass. Contact's position correction on a locked axis is overwritten, so the anchor holds bitwise.
- **This step's limit: contact against a locked body.** The contact solve treats a locked body as free, with its finite mass, and the lock then discards that body's response. So a body that hits a held one gets neither a wall's response nor a free body's. A locked body taken as infinite mass in the contact solve is later work, with joints. The contact test asserts today's behaviour, so the limit is pinned rather than discovered.
- **GPU.** The lock rows have a fixed binding in the generated registry. A set without the lock module binds an empty buffer and records Integrate with the lock select switched off, so its kernel computes today's bits.
- **Grade.** CPU reference, with a new golden: a held quadrotor flying a script, with its IMU reading +g. Vulkan banded, with `pos` and `vel` bit-exact because they are written, not integrated. Tested masks: all three axes, and y free (the climb rail). Other masks are legal.
- **Then** the drone box drops its two behaviors for the lock and stops refusing Vulkan (Interface's change, after this lands).

## 10. Publish and cameras

Publish stays empty in this step, but its role is fixed now: it is the one point where a consistent frame state is copied out. That state is poses, plus field samples at points a camera requests, taken with the same provider kernels from the published state. Stepping never reads them (`L5`). Field channels are built with Rendering, after this step.

## 11. Specified, built later

- **Regions.** A region is the whole world, a box or any SDF shape, with an integer priority, and it binds one provider per field. At a point, the highest-priority region that contains it answers. Two regions of equal priority that overlap are refused at `create()`. The whole-world region always exists at the lowest priority, so every field has a value everywhere. A sample point's region is resolved once per substep, in the Fields phase.
- **Objects in stepping.** `Simulation` will own an `ObjectGraph`. A component type is defined by a module, and attaching it allocates that module's row through the structural queue (the same `init`/`free` path as §3). Detaching and despawning free it. Attach and detach queue to step boundaries (`SL4`). The object graph still never influences layout, pass order or values (`SL3`).
- **World file v3.** On top of v2, it adds the required module set (names, versions, role choices), regions, and objects with their components' parameters. A v2 file loads as a world with the standard set, no extra regions and no objects.
- **The GPU module ABI.** How a developer module's kernels and arrays get bindings.
- **Contact participation (for SPH).** Core allocates named bits in `BodyState::flags` on a module's request. The fluid row's `init` sets a `fluid` bit, and dynamic contact skips a pair only when **both** bodies carry it. SPH's pressure force replaces fluid–fluid contact, while fluid–rigid contact still acts, so a floating box displaces water. Bodies without the bit keep today's pair order, so no golden moves.

## 12. What stays true

- No allocation in a step: module arrays and sample buffers are sized at `create()`.
- No clock in the step path. Streams come only from declared seeded streams (`L1`).
- Snapshot sections stay keyed by name. A restore into a different module set, role choice or schedule is refused (`L2`).
- Batching invariance, `workgroup_size` invariance and every existing test stay green.

## 13. Testing

- **The gate:** every golden digest unchanged and every parity band unchanged, on the standard set, CPU and Vulkan.
- **Schedule:**
  - the standard set compiles to the table in §4;
  - dropping drag's `after: rotor.forces` edge changes the compiled order and the configuration hash;
  - a cycle, an unknown phase, a dangling edge, or a writer and an accumulator with no edge between them is refused;
  - changing the order changes the configuration hash.
- **Restore (`L2`):** a snapshot restored into a different module set, module version, role choice or schedule is refused.
- **GPU chain:**
  - the recorded dispatch list equals the schedule's GPU passes;
  - the `CORE-2` barrier test holds.
- **Fields:** stored samples equal today's inline values bitwise; a kinematic pose written in Fields is seen by sampling in the same substep; `sample_medium` before the first step returns the mean wind plus the initial gust.
- **Grades and roles:**
  - a set below the minimum is refused, naming the module;
  - an `absent` module on Vulkan is refused;
  - a role filled twice or not at all is refused.
- **Lock:**
  - the new golden;
  - full-lock IMU `−Rᵀg` from a non-power-of-two mass;
  - a y-free rail;
  - a contact against a locked body: the anchor holds bitwise, and the other body's response is the finite-mass solve (the §9 limit);
  - Vulkan `pos` and `vel` bit-exact;
  - a lock with no lock module refused.
- **A developer module:** a CPU-only test module plugs in through the same API, steps on the CPU, and is refused on Vulkan.
- **Sensors (`CORE-4`):** the existing IMU and GNSS suites pass unchanged through the collapsed path.

## 14. Order of work (for the plan)

Each stage keeps every golden unchanged and is merged on its own:
1. Module descriptor and registry, with today's passes wrapped as built-in modules; the scheduler compiles to today's order.
2. The GPU chain derived from the schedule.
3. Fields: the registry and sample buffers.
4. Sensors as modules (`CORE-4`) and module-declared seeded streams.
5. Grades, roles and component-type availability.
6. The lock module, its golden, and the Vulkan path. Then Interface switches the drone box.

## 15. Amendment: device-resident state and hashed module configuration

**Status:** approved as principles 2026-10-05 (user, via lead), with the airflow spec (`../../physics/plans/2026-10-05-airflow-design.md`, Q11 and Q12; Core's answers, `../2026-10-05-airflow-spec-core-answers.md`). Stage 7's design section returns to the user for sign-off before any code.

- **Device-resident state.** A module may keep state resident on the device. For that state, the host arena stops being the one authoritative copy. Every host read path must sync that state from the device first, or refuse by name (`L6`): snapshots, `state_digest`, the readback calls, `sample_medium` and CPU state views. The CPU stays the reference (`L3`, `L4`): a device-resident module has a CPU reference implementation, so its goldens come from the CPU and its Vulkan results are banded against it (`TD-14`).
- **Hashed module configuration.** A module's configuration, such as an airflow grid's extent and resolution, is folded into the configuration identity and `config_hash`. A restore under another configuration is then refused (`L2`). The standard set has no configured modules, so its identity does not change.
- **Order of work.** Stages 4 → 5 → 6 → 7: stage 7 (airflow's GPU groundwork) builds on stage 5's grades and stage 6's fixed-binding pattern. Airflow comes before SPH. The user's bar for airflow is "F1 level, nasa level aerodynamics".
