# Module API Stage 4 Implementation Plan — modules own their state

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Modules declare the arrays they own, the rows they attach to bodies and the random streams they derive. `Simulation` registers, initializes, frees, reseeds and mirrors that state by walking the declarations, not by hand lists. The walk, every golden and every GPU band stay exactly where they are (`CORE-4`).

**Architecture:**
- A module declares **arrays**: a name, a row size and an extent. `compile_schedule` turns them into one table in walk order. `create()` registers from that table, and the GPU mirror sizes its buffers from it.
- Four built-ins carry the **legacy marker**, so their arrays register before `replay_config`, as today. Every other module array is appended after `gnss_ring`.
- **Attached rows** go through one generic op on the structural queue. A row's `init` is the module's; freeing is the core's zero-fill, cascaded by declaration when a body is despawned.
- **Streams** are declared with their domain tag. `reseed()` and `create()` walk the declarations.
- A pass sees the arrays it declares through `SubstepContext::state`. That includes another module's array, and an optional one that may be absent.
- For Physics' optional rows: `before` edges, configuration tables built from the model registry, and a vehicle-spawn hook.

**Tech Stack:** C++23, GoogleTest through ctest, MSVC + Ninja presets, Vulkan through volk (the mirror only; no kernel changes).

**Spec:** `2026-10-02-module-api-design.md` §3 (state, rows, streams, the walk), §12 and §13 ("Sensors"). Outline: `2026-10-02-module-api-plan.md`, "Stage 4". Physics' needs: `../../physics/plans/2026-10-03-propulsion-rows-plan.md` §4. Base: master `ebcc548`. Its code is as of `43eac56`: stages 1 to 3 (`062e1fa`, `dca7cfb`, `42b352a`) and drone-builder B, C and D (`7532259`, `7d46b6d`, `43eac56`; snapshot format v3).

## Global Constraints

- **No golden moves, none is regenerated and none is added** (`TD-1`, `TD-12` are not triggered). Every GPU parity band holds unchanged (`TD-2`), and the standard chain stays 11 + S dispatches per substep.
- **The standard walk stays byte-identical:** 22 entries, the same names, element sizes and extents in the same order, with `replay_config` at index 16. Snapshot format stays v3.
- No allocation in a step. Tables are sized at `create()`. Configuration tables are rebuilt only in `register_model()`. Views are refilled in place.
- **Determinism:** set order, then declaration order, everywhere. Use `std::vector` tables and linear scans or `std::map`, never an unordered container. Rows are walked in ascending slot order and worlds in index order. Every fp32 expression moves verbatim.
- Refuse, never skip (`L6`). Every refusal names the module, array, tag or pass.
- **Layering:** `sensors/`, `vehicles/`, `physics/`, `state/` and `compute/` include no `sim/` header. The built-ins' row functions live in `sim/builtin_state.cpp`, above their row types. `compute/` receives array shapes as data.
- These signatures do not change: Physics' model functions; the public `spawn`, `despawn`, `add_drag_element`, `add_imu_sensor`, `add_gnss_sensor`, `poll_imu`, `poll_gnss`, `set_rotor_commands` and `reseed`; `Simulation::create`.
- `-Wall -Wextra -Wpedantic` clean under gcc, and warning-free under MSVC.
- **Where the code lives:** `../spade-wt/core`, on branch `core/module-state`, cut from master when the lead approves this plan (it was written against `ebcc548`; re-check every file:line anchor at the cut). Commit only named paths. Never push. Never stash; park work in a WIP commit.
- **The task gate**, run at the end of every task before "ready for review":
  1. `scripts\build.ps1 -Preset debug -ParallelLevel 8` and `-Preset release -ParallelLevel 8`;
  2. `scripts\test.ps1 -Preset debug` and `-Preset release`: 0 failed, skips named, and counts reported with tree and commit (`TD-7`, `TD-8`);
  3. `scripts\docker-leg.ps1 -Memory 8g -Jobs 8` at the task's head commit: PASS.
  
  All in the foreground.

## Decisions this plan makes

**How the walk stays byte-identical.** This is the riskiest part, so it is spelt out.
- **What a declaration maps to.** One `ArrayDecl` is one `ArenaSet::register_bytes(name, elem_size, capacity)`. That registers exactly what `register_array<T>(name, capacity)` registers today:
  - two walk entries, the elements and then `name.slot_to_world`;
  - the same element size and per-world capacity;
  - zero-filled storage.
- **Capacities** are today's formulas (`simulation.cpp:375-517`):
  - `per_world` is 1;
  - `per_body` is `body_capacity`;
  - `per_element` is `element_capacity`;
  - `per_sensor` is `sensor_capacity`;
  - `per_row` is the owner's capacity × `depth`.
- **Order:**

  | Walk index | Array | Registered by |
  |---|---|---|
  | 0–5 | `world_params`, `bodies`, `body_generation` | core, typed, unchanged |
  | 6–15 | `drag_bodies`, `dryden`, `imu_sensors`, `imu_ring`, `rotors` | the `legacy_walk` modules, in set order (drag, dryden, imu, rotor) |
  | 16–17 | `replay_config` | core, typed, unchanged |
  | 18–21 | `gnss_sensors`, `gnss_ring` | gnss, the first non-legacy module with state |
  | 22+ | any later module's arrays | set order (none in the standard set) |

- **The legacy marker** is legal only on a module whose every array is one of `kLegacyWalkArrays` (`drag_bodies`, `dryden`, `imu_sensors`, `imu_ring`, `rotors`). So no other module can register before `replay_config`. A future snapshot-format version may drop the marker (spec §3).
- **Bytes.**
  - Each `init` is today's `apply_op` case, moved verbatim: the same field-wise writes, with the liveness flag last.
  - Freeing is `free_slot`'s zero-fill, plus a `memset` of the row's ring window. `ImuSample` and `GnssFix` have no implicit padding (asserted), so the memset writes the same bytes the field-wise clears write.
  - The despawn cascade runs in walk order, so rotors now free before GNSS where today they free after. Each free touches only its own array and free list, and `alloc_slot` consults only the free set (`arenas.hpp:261-266`). The bytes and every later allocation are therefore the same.
  - `reseed()` calls the same functions in the same order: worlds by index, then dryden, IMU, GNSS.
- **Snapshot v3 is unaffected:**
  - the header is unchanged;
  - the standard registry's `schema_hash` is unchanged, pinned in Task 2;
  - the configuration identity is unchanged, because it spells names, versions and pass order, and state declarations are not spelt (pinned in Task 1);
  - the model-registry identity is untouched.
  
  No version bump.

**Other decisions.**
- **A stateful module's quantities name its arrays.** If a module declares arrays, every `<module>.<name>` quantity must name one of them (the stage-1 review's minor 1). The standard set's tokens follow:

  | Was | Now |
  |---|---|
  | `dryden.state` | `dryden.dryden` |
  | `rotor.state` | `rotor.rotors` |
  | `imu.state` | `imu.imu_sensors` and `imu.imu_ring` |
  | `gnss.state` | `gnss.gnss_sensors` and `gnss.gnss_ring` |

  `drag.forces` gains its missing read of `drag.drag_bodies`. The hazard graph is the same, so the compiled order and the identity do not move. A stateless module's tokens stay free, as in stage 1.
- **The set must hold the seven built-in arrays.** `create()` mints the typed ids `Simulation` uses (`drag_id_` … `gnss_ring_id_`) from the table with `ArenaSet::typed<T>`. A set that lacks one, or declares it with another row size, is refused (`invalid_argument`, naming the array). Today such a set is accepted, and its `add_gnss_sensor` would attach a receiver that nothing synthesizes (open question 1).
- **One door for attached rows.** `attach_row` reserves, writes `body_slot` at reservation (the aliasing fix, `simulation.cpp:1645-1666`) and queues `init_row`. Each built-in's spawn checks move into that module's `validate` function, with the same messages. The typed `add_*` calls become thin fronts. Rotors refuse `attach_row`; they arrive only with a vehicle.
- **Budgets keep today's rules.** Every `per_element` array shares the world's force-element budget, as drag and rotors do now (spec §3). Each `per_sensor` array counts alone, as IMU and GNSS do now.
- **Streams live in a `per_world` row or in a slot-allocated row.** Their liveness is then the arena map, as `reseed()` reads it today. A tag is unique in the set and is never `"world"` (`kWorldSeedDomainTag`).
- **This plan departs from the spec and the outline in four places:**
  1. **No module `free` function.** Spec §3 has modules provide `init` and `free`. Every row today frees by zero-fill, so a declared function that only zero-fills would be a second thing to keep in step. It is added when a module needs more (open question 2).
  2. **Built-in passes keep their typed `WorldSubstepView` members.** Generic views are added beside them, so no Physics pass body changes (open question 5).
  3. **No file under `sensors/` or `vehicles/` changes.** The outline listed both. The row functions sit in `sim/`, so the lower layers never see the module types.
  4. **Beyond the outline,** Physics' §4 adds `before` edges, optional reads, two extents, configuration tables and a vehicle-spawn hook (Tasks 6 and 7).

## Physics' inputs (propulsion-rows plan §4)

1. **Optional modules register only when in the set.** *Provided, Task 2.* Arrays come only from the set's declarations, appended after `gnss_ring` in set order. A world without `propulsion` keeps its 22-entry walk.
2. **Extents of one row per rotor slot and one per body slot.** *Provided, Task 1.*
   - One per rotor slot is `per_row`, with owner `rotors` and depth 1, indexed by the rotor's global slot.
   - One per body slot is `per_body`.
   - Both are direct-indexed, like `imu_ring`.
3. **Attached rows, freed with their rotor or body.** *Provided, Tasks 4 and 7.*
   - `init` runs from the structural queue as `init_row`.
   - The despawn cascade clears `per_body` rows at the body's slot, and every `per_row` child of a freed row in any module. A motor row therefore clears with its rotor.
   - The rows are initialized by `ModuleDesc::vehicle_rows`, which `spawn(world, model, where)` calls in set order.
4. **Configuration buffers.**
   - *Provided on the CPU, Task 7.* A `ConfigTableDecl` is a float table that `register_model()` rebuilds from every registered model in registration order. A pass reads it as a bound view. It is not in the walk, the digest or the snapshot. It is a pure function of the registry, so snapshot v3's model-registry identity covers it, provided every `ModelType` field a build reads is folded into `model_identity` (`model_identity.hpp`'s rule).
   - *Deferred:* the GPU mirror waits for stage 6's fixed-binding pattern, then Physics' propulsion GPU step.
5. **A declared cross-module read, with `before: rotor.forces`.** *Provided, Task 6.*
   - `rotor.forces` declares `{"propulsion.motors", Access::read, .optional = true}`. It binds the motors when `propulsion` is present, and binds an empty view with no hazard when it is absent.
   - `propulsion.drive` declares `.before = {"rotor.forces"}`. It writes `rotor.rotors` (Physics' `rotors.omega`), so without the edge the two writers are refused. The order is unmissable.
6. **GPU fixed bindings that hold empty buffers when the module is absent.** *Deferred to stage 6.* The lock builds exactly this pattern first (spec §9). Until then `propulsion.drive` has no recipe, so a Vulkan set containing it is refused by name (stage 2's rule). That is Physics' own "absent on Vulkan" posture (§6).
7. **Host API.**
   - *Deferred to Physics' step 2*, when the rows exist: `set_motor_duty(VehicleRef, span<const float>)` and the motor and battery reads.
   - Core writes them beside `set_rotor_commands`: all-or-nothing, checking `enabled` and ownership, and calling `mark_vulkan_dirty()` (the census, `simulation.hpp:1564-1610`). Interface wires the sandbox.
   - *Provided now:* `module_rows<T>(array, world)`, the generic typed read (Task 4).

## Review Focus

1. **The walk.** The standard set registers exactly today's 22 entries, with `replay_config` at 16 and the schema hash unchanged. Only the five legacy arrays can register before `replay_config`. Pinned in Tasks 1 and 2, and gated by `Determinism.DigestsMatchTheCommittedGoldenCorpus`.
2. **Despawn frees exactly what the hand cascade freed, and nothing of another body.** A reserved row's `body_slot` is written at reservation, and a ring clears with its sensor. Pinned in Task 4.
3. **`reseed()` reaches every declared stream, a developer's included.** Two modules cannot share a tag. This is the 2026-10-02 GNSS defect, closed structurally. Pinned in Task 5.
4. **A pass sees what it declares and nothing else.** An optional read of an absent module binds nothing and orders nothing. An optional write is refused. Pinned in Task 6.
5. **The mirror is sized from the declarations.** It matches the registry entry for entry, and an unbound module array round-trips on the device. Pinned in Task 3.

---

### Task 1: Modules declare their arrays

**Files:**
- Modify: `engine/sim/module.hpp`. Add `Extent`, `ArrayDecl`, `row_size`, `attached_row_size`, `kCoreArrays`, `kLegacyWalkArrays`, `ModuleDesc::state` and `legacy_walk`, `CompiledArray`, `CompiledSchedule::arrays` and `walk_order`.
- Modify: `engine/sim/module_schedule.cpp`. Build and validate the table. `known_quantity` (`:37-50`) checks a stateful module's names. Fold in two stage-1 minors: refuse a pass name containing `.`, and test the unknown phase (`:149-150`).
- Modify: `engine/sim/standard_modules.cpp`. Declare the seven arrays, mark drag, dryden, imu and rotor `legacy_walk`, and rename the four quantities (`:13-15`, `:36`, `:58`, `:63`).
- Test: `tests/test_module_schedule.cpp`. Its two uses of `dryden.state` and `rotor.state` (`:270`, `:362-363`) take the new names.

**Interfaces:**
```cpp
enum class Extent : uint8_t { per_world = 0, per_body = 1, per_element = 2, per_sensor = 3, per_row = 4 };
struct ArrayDecl {
    std::string_view name;               // the registered name: unique in the set, no '.', not in kCoreArrays
    uint32_t elem_size = 0;              // attached_row_size<T>() for per_element and per_sensor; row_size<T>() otherwise
    Extent extent = Extent::per_world;
    std::string_view owner{};            // per_row only: the owning array (per_body, per_element or per_sensor; any module)
    uint32_t depth = 1;                  // per_row only: rows per owner row, >= 1
};
template <class T> consteval uint32_t row_size() noexcept;           // static_asserts trivially copyable/destructible, alignof <= 16
template <class T> consteval uint32_t attached_row_size() noexcept;  // row_size<T>() and a uint32_t body_slot at offset 0
inline constexpr std::string_view kCoreArrays[] = {"world_params", "bodies", "body_generation", "replay_config"};
inline constexpr std::string_view kLegacyWalkArrays[] = {"drag_bodies", "dryden", "imu_sensors", "imu_ring", "rotors"};
// ModuleDesc gains, after `fields`:  std::span<const ArrayDecl> state{};  bool legacy_walk = false;
inline constexpr uint32_t kNoArray = 0xFFFF'FFFFu;
struct CompiledArray { std::string module; std::string name; uint32_t elem_size = 0; Extent extent = Extent::per_world;
                       uint32_t owner = kNoArray; uint32_t depth = 1; bool legacy_walk = false; };
// CompiledSchedule gains:  std::vector<CompiledArray> arrays{};  // legacy arrays in set order, then the rest in set order
[[nodiscard]] std::vector<std::string> walk_order(const CompiledSchedule& schedule);  // core and module arrays, registration order
```
The identity does not change. Arrays are declarations, like access and fields.

- [ ] **Step 0: Pin master's two constants.** On master, before any change, print two values in a scratch test that is not committed:
  - `compile_schedule(standard_modules())->identity`;
  - `schema_hash(sim.arenas().registry())` for `one_body_world()` at 2 ms and 2 substeps.

  Paste them as `kStandardIdentity` (used here) and `kOneBodySchema` (used in Task 2). They are master's values, not regenerated ones.
- [ ] **Step 1: Write the failing tests.** Seven new tests:

```cpp
namespace {
struct TallyRow { uint32_t count; uint32_t _p[3]; };
constexpr spade::modules::ArrayDecl kTallyArrays[] = {
    {.name = "tally_counts", .elem_size = spade::modules::row_size<TallyRow>()}};
}  // namespace

TEST(StandardModules, DeclareTodaysArraysInTodaysWalkOrder) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(spade::modules::walk_order(*s),
              (Names{"world_params", "bodies", "body_generation", "drag_bodies", "dryden", "imu_sensors", "imu_ring",
                     "rotors", "replay_config", "gnss_sensors", "gnss_ring"}));
}

TEST(StandardModules, StateDeclarationsLeaveTheOrderAndTheIdentityAlone) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->identity, kStandardIdentity) << "master's value; CompileToTodaysOrder pins the order";
}

TEST(ModuleState, AStatefulModulesQuantitiesNameItsArrays) {
    static constexpr QuantityAccess wrong[] = {{"tally.count", Access::write}};
    static constexpr QuantityAccess right[] = {{"tally.tally_counts", Access::write}};
    static constexpr PassDecl a[] = {{.name = "count", .phase = Phase::forces, .access = wrong, .cpu = &noop}};
    static constexpr PassDecl b[] = {{.name = "count", .phase = Phase::forces, .access = right, .cpu = &noop}};
    const ModuleDesc bad[] = {{.name = "tally", .passes = a, .state = kTallyArrays}};
    const ModuleDesc good[] = {{.name = "tally", .passes = b, .state = kTallyArrays}};
    EXPECT_EQ(code_of(compile_schedule(bad)), spade::Code::invalid_argument);
    EXPECT_TRUE(compile_schedule(good).has_value());
}
```

  The other four, each building a set from `standard_modules()` plus one developer module, and expecting `invalid_argument` for every case listed:
  - `ModuleState.AnArrayNameIsDeclaredOnceAndIsNotACoreArray`: a repeated name, `rotors` declared again, `bodies`, `a.b`, and `elem_size` 0;
  - `ModuleState.APerRowArrayNeedsAnOwnerThatHoldsRows`: an owner no module declares, a `per_world` or `per_row` owner, depth 0, and an owner on a non-`per_row` array;
  - `ModuleState.OnlyTheLegacyArraysMayCarryTheLegacyMarker`: `tally` with `legacy_walk = true`;
  - `ModuleSchedule.APassNameWithADotOrAnUnknownPhaseIsRefused`: `static_cast<Phase>(6)`.
- [ ] **Step 2:** Build `spade_tests`. Expected: it fails to compile, because `ArrayDecl` and `ModuleDesc::state` are not declared. Commit the tests: `test(core): modules must declare their arrays, and the standard set today's walk`.
- [ ] **Step 3: Implement.**
  - The table is built in two passes over the set: the `legacy_walk` modules' arrays, then everyone else's, each in declaration order.
  - Owners resolve by name after both passes.
  - `walk_order` spells the core arrays in place: the three head arrays, the legacy arrays, `replay_config`, then the rest.
- [ ] **Step 4:** Build. Run `-Filter "ModuleSchedule|StandardModules|ModuleState|ModuleSimulation|ModuleSnapshot|ModuleFields"`. Expected: all pass, and `CompileToTodaysOrder` is unchanged. Commit: `feat(core): modules declare their arrays; compile_schedule tables them in walk order`.
- [ ] **Step 5: The task gate.** Expected: master's total + 7 on each preset, 0 failed.

**Done when:** the standard set's table spells today's walk, and the standard identity equals master's.

### Task 2: `create()` registers module arrays from the declarations

**Files:**
- Modify: `engine/state/arenas.{hpp,cpp}`. Add `register_bytes`, `typed<T>` and `bytes`.
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - The registration block (`:357-517`) registers the core head, then the legacy table rows, then `replay_config`, then the rest. The comments keep their append-only argument and gain the legacy marker.
  - Mint `drag_id_` … `gnss_ring_id_` (`simulation.hpp:1646-1663`) with `typed<T>`.
  - The restore identity message (`:2403-2408`) prints with `hex64`. This is the stage-1 minor.
  - Delete the seven module array-name constants (`:64-70`); their spelling lives in the declarations. The three core ones (`:61-63`) stay.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
// state/arenas.hpp, class ArenaSet
Result<ArrayIndex> register_bytes(std::string name, uint32_t elem_size, uint32_t capacity_per_world);  // register_array<T>'s registration, untyped
template <class T> [[nodiscard]] Result<ArrayId<T>> typed(ArrayIndex array) const;  // not_found; invalid_argument if elem_size != sizeof(T)
[[nodiscard]] Result<std::span<std::byte>> bytes(ArrayIndex array);
[[nodiscard]] Result<std::span<const std::byte>> bytes(ArrayIndex array) const;
// sim/simulation.hpp, class Simulation
[[nodiscard]] Result<ArrayIndex> module_array(std::string_view name) const;  // not_found, naming the array
// private:  std::vector<ArrayIndex> module_array_ids_;  // parallel to schedule_.arrays
```

- [ ] **Step 1: Write the failing tests.**

```cpp
namespace {
[[nodiscard]] std::vector<std::string> walk_names(const spade::Simulation& sim) {
    std::vector<std::string> out;
    sim.arenas().registry().for_each_array([&](const spade::RegisteredArray& a) { out.push_back(a.name); });
    return out;
}
}  // namespace

TEST(ModuleState, TheStandardWalkIsTodaysTwentyTwoEntries) {
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    std::vector<std::string> expected;
    for (const std::string& a : spade::modules::walk_order(sim->schedule())) {
        expected.push_back(a);
        expected.push_back(a + std::string(spade::kSlotToWorldSuffix));
    }
    EXPECT_EQ(walk_names(*sim), expected);
    EXPECT_EQ(spade::schema_hash(sim->arenas().registry()), kOneBodySchema) << "master's value";
}

TEST(ModuleState, ADevelopersArrayIsAppendedAfterTheStandardWalk) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "tally", .state = kTallyArrays});
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto walk = walk_names(*sim);
    ASSERT_EQ(walk.size(), 24u);
    EXPECT_EQ(walk[16], "replay_config");
    EXPECT_EQ(walk[22], "tally_counts");
    EXPECT_EQ(walk[23], "tally_counts.slot_to_world");
}
```

  Two more tests:
  - `ModuleState.LegacyModulesRegisterBeforeReplayConfigInSetOrder`: swap `imu` and `rotor` in the set. Expect `rotors` at 10, `imu_sensors` at 12, `imu_ring` at 14 and `replay_config` at 16.
  - `ModuleState.ASetWithoutABuiltinArrayOrWithAnotherRowSizeIsRefused`:
    - erase `gnss` from the set: `invalid_argument`, with `gnss_sensors` in the message;
    - give `rotor` a 16-byte `rotors`: refused, naming `rotors`.
- [ ] **Step 2:** Build. Expected: `TheStandardWalk…` passes, because it pins master's walk. The other three fail: `ADevelopersArray…` at 22 entries, `LegacyModules…` because registration ignores set order, and `ASetWithout…` because the set is accepted. Commit the tests: `test(core): create() must register the walk from the declarations, and refuse a set without the built-in arrays`.
- [ ] **Step 3: Implement** as listed under Files. `ReplayConfig.OccupiesItsPinnedWalkPosition` stays untouched and green.
- [ ] **Step 4:** Build. Run the focused filter plus `ReplayConfig|RestoreConfigCheck|Determinism|StructuralQueue`. Commit: `feat(core): create() registers module arrays from the declarations; the walk is unchanged`.
- [ ] **Step 5: The task gate.** Expected: Task 1's total + 4, and every golden unchanged.

**Done when:** no array outside the core four is registered by hand, the standard walk and schema hash equal master's, and a developer array is appended at 22.

### Task 3: The GPU mirror is sized from the declarations

**Files:**
- Modify: `engine/compute/backend.hpp`. Add `StateArrayShape` and `StepShape::arrays`. This is an installed header, so it gets a `CHANGELOG.md` line in Task 8.
- Modify: `engine/compute/vulkan/state_mirror.{hpp,cpp}`.
  - Delete `ArrayShape` and `array_shapes()` (`:158-206`), with the hard-coded 32 for `replay_config`.
  - `create()` makes one entry per `shape.arrays` element. It refuses an empty list (`invalid_argument`).
  - `binding_for` (`:223-253`) stays. Bindings belong to the generated registry, so they stay by name (spec §5).
  - The header's growth rule (`state_mirror.hpp:28-34`) drops `array_shapes()`.
- Modify: `engine/sim/simulation.{hpp,cpp}`. Add `state_array_shapes`. `create()` sets `shape.arrays` before `VulkanBackend::create` (`:626-646`).
- Modify: `tests/test_gpu_state_mirror.cpp` (Test/Docs reviews).
  - `shape_of()` (`:221-231`) and the three bare `StepShape`s (`:542`, `:651`, `:684`) fill `arrays` from the standard schedule.
  - `kWalkEntryCount` stays 22. Its remedy text drops `array_shapes()`.
- Test: `tests/test_module_schedule.cpp`, `tests/test_gpu_state_mirror.cpp`

**Interfaces:**
```cpp
// compute/backend.hpp
struct StateArrayShape { std::string name{}; uint32_t elem_size = 0; uint32_t capacity_per_world = 0; };
// StepShape gains:  std::vector<StateArrayShape> arrays{};  // one per walk entry, maps included, in walk order
// sim/simulation.hpp, namespace spade
[[nodiscard]] std::vector<compute::StateArrayShape> state_array_shapes(const modules::CompiledSchedule& schedule,
                                                                       const compute::StepShape& shape);
```

- [ ] **Step 1: Write the failing tests.**
  - `ModuleState.TheMirrorsShapesAreTheRegistryWalkEntryForEntry` (CPU). For the standard set and for standard + `tally`, `state_array_shapes(sim.schedule(), shape)` equals the registry's `(name, elem_size, capacity_per_world)` list. Here `shape` takes the sim's layout capacities.
  - `GpuStateMirrorTest.ADeclaredArrayWithNoBindingRoundTripsThroughTheMirror` (device). Standard + `tally`, which has no passes and so needs no recipe. Run round trip 1's sequence (`:239-293`): upload, corrupt, read back. Expect all 24 entries byte-identical.
- [ ] **Step 2:** Build. Expected: it fails to compile, because `StepShape::arrays` and `state_array_shapes` are not declared. Commit the tests: `test(core): the GPU mirror must take its arrays from the declarations`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build every target. Run `GpuStateMirror*|GpuParity*|GpuInvariance*|GpuModuleSchedule*`. Expected: every band unchanged, `RecordedChainHasABarrier…` at 11 + S, and `DescriptorSetBindsEveryRegistryBinding` green. Commit: `feat(core): the GPU mirror sizes its buffers from the module declarations; no hand list`.
- [ ] **Step 5: The task gate.** Expected: Task 2's total + 2, with the gpu count + 1.

**Done when:** `compute/` holds no list of array names or sizes, and the standard set's device buffers and bands are unchanged.

### Task 4: Attached rows through the structural queue; the sensor pairs collapse

**Files:**
- Modify: `engine/sim/module.hpp`.
  - `ArrayDecl` gains `spawn_size`, `init` and `validate`.
  - Add `kMaxSpawnBytes`, `RowInit`, `RowInitFn`, `RowValidateFn`, `row_as` and `spawn_as`. `CompiledArray` gains the same three fields.
  - New compile rules: an attached array (`per_body`, `per_element` or `per_sensor`) needs `init`, and `spawn_size` ≤ `kMaxSpawnBytes`.
- Create: `engine/sim/builtin_state.{hpp,cpp}`. Add it to `spade_sim` after `sim/standard_modules.cpp`.
  - The four init functions are moved verbatim from `apply_op` (`simulation.cpp:1110-1302`), reading the spawn record where they read `op.`.
  - Three validate functions are moved from `add_drag_element` (`:1614-1623`), `add_gnss_sensor` (`:1686-1704`) and `add_imu_sensor` (`:1744-1762`). The `"<call>: "` prefix is now added by the caller. The fourth, `refuse_direct_rotor`, is new: rotors arrive only with a vehicle.
- Modify: `engine/sim/standard_modules.cpp`. Wire `init` and `validate` into the declarations.
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - `OpKind` becomes `{init_body, free_body, init_row}`. `StructuralOp` carries `array`, `spawn_size` and `spawn` in place of `drag`, `imu`, `gnss`, `rotor` and `rotor_omega`.
  - `apply_op`'s `free_body` calls `free_rows_of`. Delete the six member pairs (`:1308-1444`) and the never-called `free_sensors_of` and `clear_sensor_ring` templates (`:164-217`).
  - `add_drag_element`, `add_imu_sensor` and `add_gnss_sensor` become fronts over `attach_row_impl`.
  - `spawn(world, model, where)` queues `init_row` with `RotorSpawn`, `DragElementSpawn` and `ImuSensorSpawn` records, without calling `validate`; `register_model` validated the model.
  - `live_force_elements` sums every `per_element` array.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
// sim/module.hpp
inline constexpr uint32_t kMaxSpawnBytes = 128;
struct RowInit {
    std::span<std::byte> row;            // the row; an attached row's body_slot is already written
    std::span<const std::byte> spawn;    // spawn_size bytes
    const WorldParams* params = nullptr; // the world's registered row (its seed)
    uint32_t local_slot = 0;             // the row's world-local slot
    uint32_t local_body = 0;             // the world-local body slot it belongs to
    float h = 0.0f;                      // the substep, s
};
using RowInitFn = void (*)(const RowInit&) noexcept;
using RowValidateFn = Result<void> (*)(std::span<const std::byte> spawn);  // the message carries no call prefix
// ArrayDecl gains, after depth:  uint32_t spawn_size = 0;  RowInitFn init = nullptr;  RowValidateFn validate = nullptr;
template <class Row> [[nodiscard]] Row& row_as(std::span<std::byte> row) noexcept;
template <class Spawn> [[nodiscard]] Spawn spawn_as(std::span<const std::byte> spawn) noexcept;  // memcpy out
// sim/builtin_state.hpp, namespace spade::modules::builtin
struct RotorSpawn { vehicles::RotorDesc desc{}; float omega = 0.0f; };
void init_drag_row(const RowInit&) noexcept;   Result<void> validate_drag_spawn(std::span<const std::byte>);
void init_imu_row(const RowInit&) noexcept;    Result<void> validate_imu_spawn(std::span<const std::byte>);
void init_gnss_row(const RowInit&) noexcept;   Result<void> validate_gnss_spawn(std::span<const std::byte>);
void init_rotor_row(const RowInit&) noexcept;  Result<void> refuse_direct_rotor(std::span<const std::byte>);
// sim/simulation.hpp
struct RowRef { uint32_t world_index = 0; uint32_t slot = 0; };
[[nodiscard]] Result<RowRef> attach_row(BodyRef body, std::string_view array, std::span<const std::byte> spawn);
template <class Spawn> [[nodiscard]] Result<RowRef> attach_row(BodyRef body, std::string_view array, const Spawn& spawn);
template <class T> [[nodiscard]] Result<std::span<const T>> module_rows(std::string_view array, uint32_t world_index) const;
// private:  Result<RowRef> attach_row_impl(BodyRef, uint32_t array, std::span<const std::byte>, std::string_view what);
//           void free_rows_of(uint32_t world_index, uint32_t body_slot);  void clear_children(uint32_t array, uint32_t slot);
```

`attach_row_impl` runs in today's order:
1. `validate_ref`, then `validate`;
2. the capacity check: a `per_sensor` array counts alone, and a `per_element` array counts against the shared budget;
3. `alloc_slot`, then the `body_slot` write at reservation;
4. queue `init_row`.

A `per_body` row needs no reservation. It is queued at the body's own slot. `free_rows_of` walks the table in walk order:
- a slot-allocated row whose map names the world and whose `body_slot` is the body's is freed, in ascending slot order;
- a `per_body` row at the body's slot is zeroed;
- each freed or zeroed row's `per_row` children are cleared with `memset`.

`init_row` keeps the defence in depth generically: it skips a slot-allocated row whose map no longer names the world (`:1126-1128`).

- [ ] **Step 1: Write the failing tests.**

```cpp
namespace {
struct TagRow { uint32_t body_slot; uint32_t live; float value; float _p; };
struct TagSpawn { float value; };
void init_tag(const spade::modules::RowInit& in) noexcept {
    TagRow& row = spade::modules::row_as<TagRow>(in.row);
    row.value = spade::modules::spawn_as<TagSpawn>(in.spawn).value;
    row.live = 1u;  // last, like every built-in's liveness flag
}
spade::Result<void> validate_tag(std::span<const std::byte> spawn) {
    if (!(spade::modules::spawn_as<TagSpawn>(spawn).value >= 0.0f))
        return std::unexpected(spade::Error{spade::Code::invalid_argument, "value must be >= 0"});
    return {};
}
constexpr spade::modules::ArrayDecl kTagArrays[] = {
    {.name = "tag_rows", .elem_size = spade::modules::attached_row_size<TagRow>(), .extent = Extent::per_sensor,
     .spawn_size = sizeof(TagSpawn), .init = &init_tag, .validate = &validate_tag},
    {.name = "tag_ring", .elem_size = spade::modules::row_size<TagRow>(), .extent = Extent::per_row,
     .owner = "tag_rows", .depth = 4}};
}  // namespace

TEST(ModuleRows, AnAttachedRowIsInitializedAtTheBoundaryAndFreedWithItsBody) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "tags", .state = kTagArrays});
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto body = sim->spawn(0, spade::BodySpawn{});
    ASSERT_TRUE(body.has_value());
    const auto row = sim->attach_row(*body, "tag_rows", TagSpawn{2.5f});
    ASSERT_TRUE(row.has_value()) << row.error().context;
    EXPECT_EQ((*sim->module_rows<TagRow>("tag_rows", 0))[0].live, 0u) << "inert until the boundary";
    ASSERT_TRUE(sim->step(0).has_value());
    EXPECT_EQ((*sim->module_rows<TagRow>("tag_rows", 0))[0].value, 2.5f);
    ASSERT_TRUE(sim->despawn(*body).has_value() && sim->step(0).has_value());
    const TagRow freed = (*sim->module_rows<TagRow>("tag_rows", 0))[0];
    EXPECT_EQ(freed.live, 0u);
    EXPECT_EQ(freed.value, 0.0f);
}
```

  Five more tests:
  - `ModuleRows.AChildRowIsClearedWithItsOwner`: write `0xAB` into `tag_ring` through the registry (as `test_gpu_state_mirror.cpp:278-279` does), despawn, flush, and expect zeroes.
  - `ModuleRows.APerBodyRowIsAttachedAtTheBodysSlotAndClearedWithIt`.
  - `ModuleRows.AttachRowRefusesWhatItCannotAttach`:
    - an unknown array: `not_found`;
    - `dryden`, a wrong spawn size, a negative value and `rotors`: `invalid_argument`;
    - a second tag past `sensors = 1`: `capacity_exceeded`;
    - `imu_sensors` with `rate_divider` 0: the text `add_imu_sensor` returns for it, after the prefix.
  - `ModuleState.AnAttachedArrayNeedsAnInitAndASpawnThatFits` (compile).
  - `ModuleRows.DespawnLeavesEveryAttachedArrayAsAFreshSimulationHasIt`. It is green on master as well, and pins the cascade's end state. Spawn a quadrotor, add a drag element and a GNSS receiver, step 3, despawn and flush. Then every byte of `drag_bodies`, `imu_sensors`, `imu_ring`, `rotors`, `gnss_sensors` and `gnss_ring` is zero, and every map entry reads free.
- [ ] **Step 2:** Build. Expected: it fails to compile, because `attach_row`, `RowInit` and `ArrayDecl::init` are not declared. Commit the tests: `test(core): attached rows must go through one queued init and one declared cascade`.
- [ ] **Step 3: Implement** as listed under Files. The `apply_op` comments move with their code into `builtin_state.cpp`.
- [ ] **Step 4:** Build. Run the focused filter plus `Imu|Gnss*|SensorKind|StructuralQueue|VehicleRefAt|QuadHover|Spawn|M1B`. Expected: every IMU and GNSS test passes unchanged, through `init_row` and `free_rows_of`. Commit: `feat(core): attached rows are declared, queued and freed generically; the IMU/GNSS pairs collapse (CORE-4)`.
- [ ] **Step 5: The task gate.** Expected: Task 3's total + 6, every golden unchanged, and `VulkanDirtyTracking.*` green. `attach_row` joins the census's "covered transitively" row in Task 8.

**Done when:**
- `simulation.cpp` names no sensor, rotor or drag array in `apply_op`, the cascade or the `add_*` bodies;
- the public calls keep their signatures and messages;
- the IMU and GNSS suites pass unchanged.

### Task 5: Seeded streams are declared; `reseed()` walks them

**Files:**
- Modify: `engine/sim/module.hpp`. Add `RowReseedFn`, `StreamDecl`, `ModuleDesc::streams`, `CompiledStream` and `CompiledSchedule::streams`.
- Modify: `engine/sim/module_schedule.cpp`. A stream's tag must be non-empty and unique in the set, and not `kWorldSeedDomainTag`. Its array must be the declaring module's, and `per_world`, `per_element` or `per_sensor`. `reseed` is non-null.
- Modify: `engine/sim/builtin_state.{hpp,cpp}`. Add the three reseed functions: `dryden_init`, and the two `noise` re-derivations (`simulation.cpp:2518-2532`).
- Modify: `engine/sim/standard_modules.cpp`. Declare `dryden` on `dryden`, `sensor.imu` on `imu_sensors` and `sensor.gnss` on `gnss_sensors`.
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - `create()`'s per-world loop (`:527-555`) runs each `per_world` stream's `reseed` after the `WorldParams` write, as it calls `dryden_init` now.
  - `reseed()` (`:2477-2553`) writes the seed, then walks `schedule_.streams` over live rows.
  - The header's inventory (`simulation.hpp:1171-1209`) becomes a pointer to the declarations.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
using RowReseedFn = void (*)(std::span<std::byte> row, const WorldParams& params, uint32_t local_slot) noexcept;
struct StreamDecl {
    std::string_view tag;          // the rng domain tag: unique in the set, never "world"
    std::string_view array;        // this module's per_world, per_element or per_sensor array
    RowReseedFn reseed = nullptr;  // re-derives one live row's stream; history untouched
};
// ModuleDesc gains, after legacy_walk:  std::span<const StreamDecl> streams{};
struct CompiledStream { std::string module; std::string tag; uint32_t array = kNoArray; RowReseedFn reseed = nullptr; };
// CompiledSchedule gains:  std::vector<CompiledStream> streams{};  // set order, then declaration order
// sim/builtin_state.hpp
void reseed_dryden_row(std::span<std::byte>, const WorldParams&, uint32_t) noexcept;
void reseed_imu_row(std::span<std::byte>, const WorldParams&, uint32_t) noexcept;
void reseed_gnss_row(std::span<std::byte>, const WorldParams&, uint32_t) noexcept;
```

- [ ] **Step 1: Write the failing tests.** Four tests:
  - `ModuleStreams.ATagIsDeclaredOnceAndIsNotTheWorldTag`: `sensor.imu` declared again, and `world`.
  - `ModuleStreams.AStreamLivesInItsModulesSlotAllocatedOrPerWorldArray`: another module's array, a `per_row` array, and a null `reseed`.
  - `StandardModules.DeclareTheDrydenImuAndGnssStreams`: the tags, in order: `dryden`, `sensor.imu`, `sensor.gnss`.
  - `ModuleStreams.ReseedRederivesADevelopersStreamAndLeavesFreeRowsZero`:

```cpp
// NoisyRow: { uint32_t body_slot; uint32_t live; uint32_t _p[2]; spade::rng::Stream noise; }, per_sensor;
// its init and its reseed both set noise = rng::make_stream(params.seed, "test.noisy", local_slot).
TEST(ModuleStreams, ReseedRederivesADevelopersStreamAndLeavesFreeRowsZero) {
    auto sim = make_noisy_sim();  // standard + "noisy", one_body_world() with sensors = 2; one body, one row attached, flushed
    ASSERT_TRUE(sim.reseed(0xBEEF).has_value());
    const auto rows = sim.module_rows<NoisyRow>("noisy_rows", 0);
    const auto params = sim.world_params(0);
    ASSERT_TRUE(rows.has_value() && params.has_value());
    const spade::rng::Stream want = spade::rng::make_stream((*params)->seed, "test.noisy", 0);
    EXPECT_EQ(std::memcmp(&(*rows)[0].noise, &want, sizeof(want)), 0) << "the live row follows the new seed";
    const std::array<std::byte, sizeof(NoisyRow)> zero{};
    EXPECT_EQ(std::memcmp(&(*rows)[1], zero.data(), zero.size()), 0) << "a free row stays zero";
}
```

- [ ] **Step 2:** Build. Expected: it fails to compile, because `StreamDecl` is not declared. Commit the tests: `test(core): streams must be declared, and reseed() must reach every one`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter plus `Imu.Reseed*|GnssReseed*|M1B.Reseed*|RestoreConfigCheck.Reseed*|Dryden*|SimFields*`. Expected: all pass unchanged. Commit: `feat(core): modules declare their seeded streams; create() and reseed() walk them`.
- [ ] **Step 5: The task gate.** Expected: Task 4's total + 4.

**Done when:** `reseed()` names no system, and a developer's stream is reseeded with no edit to `Simulation`.

### Task 6: Passes see the state they declare

**Files:**
- Create: `engine/state/state_view.hpp`. Add `StateView` and `world_rows<T>`.
- Modify: `engine/physics/schedule.hpp` (Physics reviews). `SubstepContext` (`:167-227`) gains `state`. Drop the unused `<string_view>` (`:6`, stage-1 minor).
- Modify: `engine/sim/module.hpp` and `module_schedule.cpp`.
  - Add `QuantityAccess::optional`, `PassDecl::before`, `BindingKind`, `CompiledBinding` and `CompiledPass::state`.
  - A `before` edge is checked like `after` (`:226-240`), mirrored: it must name a declared pass in the same or a later phase.
  - `optional` is legal only on `read` of `<module>.<name>`. If the module is absent, there is no hazard and the binding is `absent`. If it is present, the name must be an array.
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - Add `pass_views_` and `pass_view_begin_`, sized at `create()`. `rebuild_views()` refills them in place from `ArenaSet::bytes`.
  - `step()`'s loop (`:939-943`) sets `ctx.state` before each pass.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
// state/state_view.hpp, namespace spade
struct StateView {
    std::byte* data = nullptr;        // null when absent
    uint32_t elem_size = 0;
    uint32_t world_count = 0;
    uint32_t capacity_per_world = 0;
    [[nodiscard]] bool present() const noexcept { return data != nullptr; }
};
template <class T> [[nodiscard]] std::span<T> world_rows(const StateView& v, uint32_t world) noexcept;  // empty if absent or sizeof(T) != elem_size
// physics::SubstepContext gains:  std::span<const StateView> state;  // state[i] is the running pass's i-th declared access
// sim/module.hpp
// QuantityAccess gains:  bool optional = false;
// PassDecl gains, after `after`:  std::span<const std::string_view> before{};  // "<module>.<pass>"
enum class BindingKind : uint8_t { absent = 0, array = 1 };
struct CompiledBinding { BindingKind kind = BindingKind::absent; uint32_t index = kNoArray; };
// CompiledPass gains:  std::vector<CompiledBinding> state{};  // one per declared access, in declaration order
```

Every access gets a slot, so a developer indexes `ctx.state` by its own access list. Core quantities, fields and plain tokens hold absent views. The built-in passes still read their typed `WorldSubstepView` members.

- [ ] **Step 1: Write the failing tests.**

```cpp
namespace {
void tally_pass(const spade::physics::SubstepContext& ctx) noexcept {
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<TallyRow> rows = spade::world_rows<TallyRow>(ctx.state[0], w);
        if (!rows.empty()) ++rows[0].count;
    }
}
constexpr QuantityAccess kTallyAccess[] = {{"tally.tally_counts", Access::write}};
constexpr PassDecl kTallyPasses[] = {{.name = "count", .phase = Phase::forces, .access = kTallyAccess, .cpu = &tally_pass}};
}  // namespace

TEST(ModuleViews, APassSeesItsOwnArrayPerWorld) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "tally", .passes = kTallyPasses, .state = kTallyArrays});
    auto sim = spade::Simulation::create(two_world_set(), 2'000'000, 2, {}, set);  // file-local: two one_body_world() instances
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(3).has_value());
    for (uint32_t w = 0; w < 2; ++w) EXPECT_EQ((*sim->module_rows<TallyRow>("tally_counts", w))[0].count, 6u) << w;
}
```

  Six more tests:
  - `ModuleViews.ADevelopersStateSnapshotsAndRestoresBitwise`: snapshot at 3 steps, restore into a twin, step both 2 more. The `state_digest`s are equal and the count is 10.
  - `ModuleViews.AReaderOfAnotherModulesArrayRunsAfterItsWriter`: a `copier` module, placed before `tally` in the set, reads `tally.tally_counts` into its own array, and sees this substep's count.
  - `ModuleViews.AnOptionalReadOfAnAbsentModuleBindsNothing`: the copier's read is marked optional, and `tally` is absent. It compiles, the pass sees `!ctx.state[0].present()`, and the schedule equals the copier-free order plus `copier.copy`.
  - `ModuleSchedule.AnOptionalAccessMayOnlyReadAnArrayThatExists`: an optional write, and an optional read of `tally.nope` with `tally` present.
  - `ModuleSchedule.ABeforeEdgeOrdersTwoWriters`: two writers of `body.pose`. The second in set order declares `before` the first, so it compiles first. Without the edge they are refused.
  - `ModuleSchedule.ABeforeEdgeMustNameAPassInThisOrALaterPhase`: a dangling target, and an earlier phase.
- [ ] **Step 2:** Build. Expected: it fails to compile, because `SubstepContext::state`, `world_rows`, `QuantityAccess::optional` and `PassDecl::before` are not declared. Commit the tests: `test(core): passes must see the state they declare, optional reads must bind nothing when absent, and before edges must order`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter plus `Determinism|ScenarioCorpus|GoldenCorpus|KinematicMover*`. Expected: all pass, and `CompileToTodaysOrder` and the identity pin are unchanged. Commit: `feat(core): passes see their declared arrays through SubstepContext::state; optional reads and before edges`.
- [ ] **Step 5: The task gate.** Expected: Task 5's total + 7.

**Done when:** a developer module owns, steps, snapshots and restores its state on the CPU through the public API alone, and Physics' input 5 can be declared exactly as written above.

### Task 7: Model-driven state for optional modules

**Files:**
- Modify: `engine/sim/module.hpp`.
  - Add `ConfigBuildFn`, `ConfigTableDecl`, `VehicleRows`, `RowInitRequest`, `VehicleRowsFn` and `init_request`.
  - `ModuleDesc` gains `tables` and `vehicle_rows`. Add `CompiledTable` and `CompiledSchedule::tables`. `BindingKind` gains `table`.
  - It includes `vehicles/model_type.hpp`. A span of an incomplete type breaks MSVC (`physics/schedule.hpp:20-28`).
- Modify: `engine/sim/module_schedule.cpp`.
  - Table names share the array namespace.
  - A `<module>.<table>` access must be `read`.
  - `build` is non-null.
  - A stateful module's names may be arrays or tables.
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - `config_tables_` is built at `create()` from no models, and rebuilt by `register_model()` (`:1810-1828`) from all of them, in registration order.
  - `spawn(world, model, where)` (`:1853-2109`) calls each module's `vehicle_rows` in set order, after its reservations and before the generation bump, so a bad request unwinds (`:1962-1968`). Each request is checked:
    - the array is the hook's module's;
    - a `per_body` row is the vehicle's body slot;
    - a `per_row` row's owner is `rotors`, and its slot lies in one of the vehicle's rotor windows;
    - `spawn_size` matches.
  - The checked requests are queued as `init_row` after the built-ins' ops.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
using ConfigBuildFn = void (*)(std::span<const vehicles::ModelType> models, std::vector<float>& out);
struct ConfigTableDecl { std::string_view name; ConfigBuildFn build = nullptr; };
struct VehicleRows {
    const vehicles::ModelType* model = nullptr;
    uint32_t world_index = 0;
    uint32_t body_slot = 0;                  // global
    std::span<const uint32_t> rotor_slots;   // global, in the model's rotor order
};
struct RowInitRequest {
    std::string_view array;                  // this module's per_body array, or a per_row array owned by rotors
    uint32_t slot = 0;                       // global slot of the row
    std::array<std::byte, kMaxSpawnBytes> spawn{};
    uint32_t spawn_size = 0;
};
using VehicleRowsFn = void (*)(const VehicleRows&, std::vector<RowInitRequest>& out);
template <class Spawn> [[nodiscard]] RowInitRequest init_request(std::string_view array, uint32_t slot, const Spawn& spawn) noexcept;
// ModuleDesc gains, after streams:  std::span<const ConfigTableDecl> tables{};  VehicleRowsFn vehicle_rows = nullptr;
struct CompiledTable { std::string module; std::string name; ConfigBuildFn build = nullptr; };
// CompiledSchedule gains:  std::vector<CompiledTable> tables{};   BindingKind gains:  table = 2
// sim/simulation.hpp
[[nodiscard]] Result<std::span<const float>> config_table(std::string_view name) const;
```

A table's view has `world_count` 1 and `elem_size` 4. A pass reads it with `world_rows<float>(v, 0)`.

- [ ] **Step 1: Write the failing tests.**

```cpp
// "ballast": a per_body array ballast_rows (BallastRow{float kg; uint32_t _p[3];}), a per_row array rotor_marks
// owned by "rotors" (MarkRow{uint32_t index; uint32_t _p[3];}), and this hook:
void ballast_vehicle_rows(const spade::modules::VehicleRows& v, std::vector<spade::modules::RowInitRequest>& out) {
    out.push_back(spade::modules::init_request("ballast_rows", v.body_slot, 2.0f * v.model->body.mass));
    for (uint32_t i = 0; i < v.rotor_slots.size(); ++i)
        out.push_back(spade::modules::init_request("rotor_marks", v.rotor_slots[i], i + 1u));
}

TEST(ModuleVehicleRows, AVehicleSpawnInitializesItsBodyAndRotorRowsAndDespawnClearsThem) {
    auto sim = make_ballast_sim();                            // standard + "ballast", one world, room for a quadrotor
    const auto model = sim.register_model(spade::vehicles::make_quadrotor(spade::vehicles::QuadrotorParams{}));
    ASSERT_TRUE(model.has_value());
    const auto quad = sim.spawn(0, *model, spade::VehicleSpawn{.pos = {0.0f, 10.0f, 0.0f}});
    ASSERT_TRUE(quad.has_value() && sim.step(0).has_value());
    const auto ballast = sim.module_rows<BallastRow>("ballast_rows", 0);
    const auto marks = sim.module_rows<MarkRow>("rotor_marks", 0);
    const uint32_t local_body = quad->body.slot;              // world 0: global == local
    EXPECT_EQ((*ballast)[local_body].kg, 2.0f * (*sim.model(*model))->body.mass);
    for (uint32_t i = 0; i < quad->rotor_count; ++i) EXPECT_EQ((*marks)[quad->rotor_slots[i]].index, i + 1u);
    ASSERT_TRUE(sim.despawn(quad->body).has_value() && sim.step(0).has_value());
    EXPECT_EQ((*ballast)[local_body].kg, 0.0f);
    for (uint32_t i = 0; i < quad->rotor_count; ++i) EXPECT_EQ((*marks)[quad->rotor_slots[i]].index, 0u);
}
```

  Three more tests:
  - `ModuleTables.ATableIsTheRegisteredModelsInRegistrationOrder`: a `masses` table holds each model's `body.mass`. It is empty at `create()`, `{m1}` after one model and `{m1, m2}` after two. A pass that copies `table[0]` into a `per_world` row sees it.
  - `ModuleTables.ATableMayOnlyBeRead` (compile).
  - `ModuleVehicleRows.ARequestOutsideTheVehicleIsRefusedAndUnwinds`: a hook naming another body's slot makes `spawn` return `invalid_argument`. The live body, rotor and sensor counts are as before, and the queue is empty.
- [ ] **Step 2:** Build. Expected: it fails to compile, because `ConfigTableDecl`, `vehicle_rows` and `config_table` are not declared. Commit the tests: `test(core): optional modules must get tables from the model registry and rows from a vehicle spawn`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter plus `ModelRegistry*|QuadHover|VehicleRefAt|Spawn|DesignFrame*`. Commit: `feat(core): configuration tables and vehicle-spawn rows for optional modules (propulsion-rows §4)`.
- [ ] **Step 5: The task gate.** Expected: Task 6's total + 4.

**Done when:** Physics' step 2 can declare `propulsion` (motors, batteries, two tables, `drive` with `before: rotor.forces`, the hook) with no engine change except `set_motor_duty` and its reads.

### Task 8: Prose follows the code

- **Code comments**, built once as a compile check:
  - `simulation.cpp`'s registration block (`:316-356`): legacy marker, not call order;
  - the sensor-family note (`:73-108`): its "not templated" paragraph now describes the modules' `validate` functions;
  - the census (`simulation.hpp:1564-1610`): `attach_row` is covered transitively;
  - `despawn()`'s contract;
  - the stale counts of "nine" arrays: `state_mirror.cpp:159,172`, `simulation.cpp:2434` and `test_determinism.cpp:1983`. There are eleven.
- **Docs, on master after the merge:**
  - `core/07-status.md`:
    - "Modules declare…" becomes **state, fields and passes**;
    - the snapshot row says **format v3**, which the drone builder's Task C made it;
    - "Code that disagrees with itself" records the dead templates this stage deleted.
  - `core/02-state-and-snapshot.md`: format v3; the legacy marker; streams declared per module.
  - `core/01-modules-and-scheduler.md`: arrays, attached rows, streams, `before`, optional reads.
  - `CHANGELOG.md`:
    - `StepShape::arrays`;
    - `ArenaSet::register_bytes`, `typed` and `bytes`;
    - the `ModuleDesc`, `PassDecl` and `QuantityAccess` additions;
    - `SubstepContext::state`;
    - `attach_row`, `module_rows`, `module_array` and `config_table`.
- **Tell the lead,** for `../../consumers.md`: new public calls; snapshot format unchanged (v3); `StepShape` gains a member.
- [ ] Commit: `docs(core): comments follow modules owning their state`.

## Open questions for the lead

1. **May a set omit a stateful built-in (drag, dryden, imu, rotor, gnss)?** *Recommendation:* refuse it at `create()` in this stage (Task 2). Allowing it means every typed host call needs an "absent" refusal. Stage 5's component availability is the natural place to open it. Today's tests and the sandbox build their sets from `standard_modules()`. Task 2's red run names any existing test the refusal changes, before the refusal lands. The alternative is stage 6's pattern: the module's typed calls refuse with `unavailable` when it is absent.
2. **Module `free` functions (spec §3).** *Recommendation:* none until a module needs more than a zero-fill. The core's cascade is the free, and one place stays easier to keep right.
3. **Does Task 7 belong in stage 4,** or with Physics' propulsion step 2? *Recommendation:* stage 4. It is Core mechanism, with no Physics model in it, and without it Physics' step 2 cannot start without a Core engine change.
4. **Should the configuration identity spell state declarations and stream tags?** *Recommendation:* no.
   - The schema hash already refuses a blob whose arrays differ.
   - A changed tag or `init` rides the module version, as a changed pass function does since stage 1.
   - Spelling them would change the standard identity and refuse every stored v3 blob, for no change in physics.
5. **When do the built-in passes read through `ctx.state`?** *Recommendation:* with the GPU module ABI, not now. Today they keep their typed members, which leaves every Physics pass body and every golden untouched in this stage.

## Notes for later stages

- **Transient scratch (Physics' IMU contact plan, `../../physics/plans/2026-10-05-imu-contact-specific-force-plan.md`, option A, agreed 2026-10-05).** Beside walked arrays, a module may declare a per-body **scratch**: not walked, not digested, not snapshotted, zero-filled at `create()`, and zero at every substep boundary by the module's own invariant. `contact_dv` is the first. If the IMU change lands first, its scratch is hand-registered like the grid's, and Task 1 adds the `scratch` kind and moves it over. If stage 4 lands first, the IMU change declares it. Either way, whichever lands second adopts the other. The parity plan (`2026-10-05-banded-parity-plan.md`) also changes this plan's "every GPU parity band holds unchanged" to "at its cut": whichever merges second re-runs `ctest -L gpu`.

- **Stage 6, the lock,** is a `per_body` array attached through `attach_row`, with `lock_translation` as its front. Its fixed GPU binding, which binds an empty buffer when the module is absent, is the pattern Physics' inputs 4 (the GPU mirror) and 6 then reuse.
- **Physics' propulsion step 2:** `set_motor_duty` and the motor and battery reads (input 7). Fold the new `ModelType` propulsion fields into `model_identity` in a way that leaves a propulsion-free model's identity, and so Kat's airframe hashes, unchanged.
- **SPH** gets per-particle rows as `per_body` arrays. The contact-participation bit (spec §11) is a later `init`'s job.
- **Stage-1 minor still open:** the cycle message's wording.
