# Module API Stage 5 Implementation Plan — grades, roles and component availability

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Each module declares a grade per backend, and `create()` refuses a set that falls below a requested minimum or holds a module absent on its backend. Roles make `dynamic_contact` a slot that exactly one module fills. The component-type table says which types can be built, with `fluid` reserved. First, two defects in stage 4's pass views are closed: a configuration table reads as empty from every world but 0, and a declared read hands out writable bytes. Every golden, the standard identity and the schema hash stay where they are.

**Architecture:**
- **Views first.** A table view is shared: every world reads the same rows, through `world_rows` or the new `table_rows`. A view keeps a const pointer for reading and a writable one only when its access writes or accumulates, so a declared read cannot hand out mutable rows.
- **Grades** are a four-value enum in `core/grade.hpp`. A module declares one per backend in `ModuleDesc::grade`. `compile_schedule` checks the declaration's shape (no Vulkan `reference`; Vulkan `banded` needs a CPU `reference`). `create()` checks it against the chosen backend and an optional minimum, before any device is touched.
- **Roles** come from one core table, `kRoles`. A module fills at most one. `compile_schedule` refuses a role filled twice, and `create()` refuses a required role left empty.
- **Component availability** sits beside the type table in `objects/component.hpp`. The loader and `ObjectGraph::attach` refuse a reserved type, and every enumerator lists it with its reason.

**Tech Stack:** C++23, GoogleTest through ctest, MSVC + Ninja presets. No kernel, binding or mirror change.

**Spec:** `2026-10-02-module-api-design.md` §3 (component types), §7 (roles), §8 (grades), §11, §13 ("Grades and roles", "Restore"), and the §15 amendment (approved as principles 2026-10-05). Outline: `2026-10-02-module-api-plan.md`, "Stage 5". Airflow's needs: `../../physics/plans/2026-10-05-airflow-design.md` §1.1, §1.7, §8 Q9–Q11, with Core's answers (`../2026-10-05-airflow-spec-core-answers.md`). Grades under banded parity: `TD-14`, `CORE-3` and `CORE-5`. Base: master after `core/module-state` (stage 4 Tasks 7b and 8) merges. Code read at master `7d030b9` (stage 4 Tasks 2–7 merged at `a698ea4`) and `core/module-state` at `335a899` plus Task 7b's working tree. Anchors marked **ms** are from that tree. Draft for the lead, 2026-10-05.

## Global Constraints

- **No golden moves, none is regenerated and none is added** (`TD-1`, `TD-12` are not triggered). Every GPU parity band holds unchanged (`TD-2`), and the standard chain stays 11 + S dispatches per substep.
- **The identity and the walk stay put.** The standard identity stays `0x3303cfc86821f502` and the one-body schema hash `0x1f60a6fef22e254a` (both pinned, `test_module_schedule.cpp:698-701` ms). The walk stays 22 entries with `replay_config` at 16, snapshot format v3, and no module version moves. Grades, roles and availability are declarations, never spelt into the identity (Decisions).
- No allocation in a step. Views are refilled in place. The grade, role and availability checks run in `compile_schedule`, `create()` and the loader, never in a step.
- **Determinism:** set order, then declaration order. No unordered container. No fp32 expression changes.
- Refuse, never skip (`L6`). Every refusal names the module, role, pass or component type. A refusal at `create()` comes before `validate_world_set` and before any backend is made, so it reads no device and runs in the Docker leg.
- **Layering:** `core/grade.hpp` includes nothing from `sim/`, `objects/` or `compute/`. `objects/` includes no `sim/` header. `state/state_view.hpp` stays dependency-free.
- **These signatures do not change:** `spawn`, `despawn`, `add_drag_element`, `add_imu_sensor`, `add_gnss_sensor`, `poll_imu`, `poll_gnss`, `set_rotor_commands`, `reseed`, `set_behaviors`, `attach_row`, `module_rows`, `config_table`, and Physics' model functions. `Simulation::create` gains one trailing defaulted parameter (Task 5), so every call site compiles unchanged.
- `-Wall -Wextra -Wpedantic` clean under gcc, and warning-free under MSVC.
- **Where the code lives:** `../spade-wt/core`, on branch `core/module-grades`, cut from master once `core/module-state` merges and the lead approves this plan. Re-check every file:line anchor at the cut. Commit only named paths. Never push. Never stash; park work in a WIP commit.
- **The task gate**, run at the end of every task before "ready for review":
  1. `scripts\build.ps1 -Preset debug -ParallelLevel 8` and `-Preset release -ParallelLevel 8`;
  2. `scripts\test.ps1 -Preset debug` and `-Preset release`: 0 failed, skips named, and counts reported with tree and commit (`TD-7`, `TD-8`). The full suite runs every `gpu` test on the admitted RTX 3060 Ti, so the debug run takes about 6–9 minutes;
  3. `scripts\docker-leg.ps1 -Memory 8g -Jobs 8` at the task's head commit: PASS. One fresh leg at a time on this machine, whose VM is capped at 10 GB. Do not redirect the leg driver's output (`2>&1`).

  All in the foreground.
- **Red first.** Every task commits its failing tests before the change. The red commit compiles on its own: any new name it needs is a minimal stub, so the tests fail on behaviour, never on a missing declaration. A test that is green at red is a pin, and the step says so.

## Decisions this plan makes

**What a grade means, now that CPU↔GPU is banded.** A grade is a module's claim about its implementation on one backend. `create()` checks the claim; it measures nothing.

| Grade | What it claims | Where it may be declared |
|---|---|---|
| `reference` | The CPU implementation is the source of truth. Its goldens are CPU digests, bit-identical across MSVC and gcc (`TD-1`, `TD-12`) | The CPU. A Vulkan `reference` is refused at registration (`L4`: the GPU is never a golden source) |
| `banded` | Every quantity the module writes stays inside its pinned `TD-14` bands of the CPU reference: `\|gpu − cpu\| ≤ abs + rel·\|cpu\|`, and a NaN fails. The bands are measured on each device of record, with `L4`'s record, up to each scenario's horizon; past it, invariants and statistics hold. Results are deterministic per device and driver (`CORE-5`), across runs, batching and workgroup sizes. They are **not** bit-identical to the CPU: a run resumed on the other backend continues within the band, not on the same bits | Vulkan, and only with a CPU `reference` to band against (Task 3) |
| `best-effort` | It runs, with no golden and no band. Under `PHY-3`, stepping reads it only in a world that accepts that grade | Either backend |
| `absent` | There is no implementation on that backend. It is a declaration, never a world's grade: `create()` always refuses it there | Either backend |

- **Order:** `reference` > `banded` > `best-effort` > `absent`. A set's grade on its backend is its lowest module's (`L3`), named by the first such module in set order.
- **Any admitted device.** `CORE-5` admits any Vulkan 1.1 device. On a device that is not of record, `banded` is claimed but unmeasured there, and `vulkan_device_report()` names the device and driver. That report is the announcement `L6` asks for. The NVIDIA report's option 4 (banded per driver, refused on an unmeasured one) is superseded by `CORE-5`. No grade depends on the device.
- **One grade per module and backend,** not per configuration. Airflow's Q9 (A) keeps one Vulkan grade at 32³ and 128³.
- **So no Vulkan set can meet a `reference` minimum.** That closes the gap the banded-parity plan records (`2026-10-05-banded-parity-plan.md:434`): a consumer can now ask for reference grade and be refused on Vulkan.

**How the identity stays put.** Nothing in this plan changes the standard identity or the schema hash, so no stored blob is refused.
- **Grades are not spelt.** They are claims, not computations. A CPU blob restores into a Vulkan simulation today, and still will.
- **The role choice is already spelt.** A role's filler is a module, and the identity spells every module's name and version and the compiled pass order (`module.hpp:605-614` ms). A restore under another role choice is therefore refused (`L2`) with no new bytes. Spec §7's "the role choice goes into the configuration hash" holds as written. Task 6 pins it.
- **Availability is not spelt.** The object graph is in no blob (`SL3`).
- **The cost if this is wrong:** spelling roles or grades explicitly would change the standard identity, and every stored v3 blob would be refused once, as after stage 3 (`consumers.md:18`), for no change in physics. If the lead wants that, it is its own decision, not this plan's.

**Other decisions.**
- **A table is shared, not world 0's** (Task 1). A table view carries the simulation's world count and a `shared` flag. `world_rows` returns the whole table for every world in the set. `table_rows<T>(view)` reads it with no world. The lead's alternative, refusing or asserting in `world_rows`, is open question 1.
- **A declared read gets const rows by type** (Task 2). `StateView::data` becomes `const std::byte*`. A second pointer, `writable`, is set only for a write or an accumulation. `world_rows<const T>` reads `data`; `world_rows<T>` reads `writable`, so a read view yields no mutable rows. There is no new branch: the accessor already tests its pointer. A table is never writable. Writing through a read view now takes a `const_cast` in the pass's own source, where review sees it.
- **A module that declares no grade is `best-effort` on both backends.** That is the weakest grade that runs, so an undeclared module never claims more than it has shown. It is refused only when a caller asks for more. Every built-in declares its grade explicitly (Task 4), and no test-local module needs editing (open question 3).
- **Refusal order at `create()`:**
  1. `compile_schedule`, including grade shapes (Task 3) and roles filled twice (Task 6);
  2. `check_builtin_arrays`, then `check_required_roles` (Task 6);
  3. on Vulkan, stage 2's per-pass recipe refusal and Task 7b's scratch-binding refusal, unchanged and first, so their tests keep their messages;
  4. a module `absent` on the chosen backend: `unavailable`, naming it;
  5. a set below the minimum: `unavailable`, naming the first module below it and its grade.
- **Attached behaviors are `best-effort`.** A behavior is developer code with no declared grade. A step with a non-empty registry attached, in a simulation whose minimum is above `best-effort`, returns `unavailable` and steps nothing. That is `CORE-1`'s pattern and site (`simulation.cpp:1109-1113` ms). `Simulation::grade()` reports `best-effort`, naming `behaviors`, while one is attached. No caller passes a minimum today, so nothing changes for the drone box (open question 4).
- **Roles live in one core table,** `kRoles`, so a misspelt role is refused, as a misspelt quantity is (Plan Ruling 3). `dynamic_contact` is required. A required role is checked at `create()`, not in `compile_schedule`, because many compile-only tests build sets without contact.
- **Availability is static,** a column of the type table. No module defines a component type yet, and the graph is not in stepping (spec §11). A reserved type cannot be built: `ObjectGraph::attach<T>` fails to compile for it, and the loader refuses it with `unavailable`.
- **This plan departs from the spec and the outline in five places:**
  1. **Tasks 1 and 2 are not in the outline.** The lead requires them.
  2. **Roles are not spelt into the identity.** They are already covered (above).
  3. **No scenario names its role choice yet** (spec §7). Only one filler of `dynamic_contact` exists, so the key would have no use until Jacobi (`PHY-5`). See Notes.
  4. **Availability is static, not per module set** (spec §3: "a component type no module defines is reserved"). Per-set availability comes with objects in stepping (§11).
  5. **Beyond the spec:** Vulkan `banded` requires a CPU `reference` (§15's rule for device-resident modules, made general), and attached behaviors count as `best-effort`.

## Physics' and Rendering's inputs

The built-ins' grades are Physics' and Rendering's to declare (spec §8). Task 4 is joint: Core writes the declarations into `sim/standard_modules.cpp`, and Physics reviews the commit and its pin test.

**From Physics, before Task 4's red commit:**
1. **Confirm `PHY-2`'s table** (`../../physics/04-verification.md`, "Grades") row by row against the standard modules. `integrate`, `static_contact`, `dynamic_contact`, `drag`, `rotor`, `dryden`, `imu` and `gnss` are each CPU `reference` and Vulkan `banded`. "Banded via `CORE-3`" declares plain `banded`.
2. **`environment`'s grade.** It is not in `PHY-2`'s table. *Core's suggestion:* CPU `reference` (every golden runs it) and Vulkan `banded` (its kernel copies constants).
3. **Confirm that `dynamic_contact` is a required role** and that the Gauss-Seidel module fills it in the standard set. Jacobi stays unwired (`PHY-5`).
4. **The grades Physics' optional modules will declare,** so Task 3's rules fit them:
   - propulsion: CPU `best-effort` until its golden exists, then `reference` (`DBP-60`). Vulkan `absent` until its kernels exist, which turns its "absent on Vulkan" posture into a declaration;
   - airflow, both tiers: CPU `reference`, Vulkan `banded` (airflow spec §1.1);
   - the airspeed sensor: the same (§3.5);
   - `PHY-7`'s `contact_dv`: no grade change; it is `integrate`'s scratch.
5. **After the merge,** Physics replaces `04-verification.md`'s last line ("Until the grade check exists (Core), this table is the declaration") with a pointer to the declarations.

**From Rendering:**
1. **Confirm that no Rendering module is in a module set,** so stage 5 declares no Rendering grade. `RND-3`'s renderer grades stay documented until a renderer or a camera-sensor module joins a set (`engine A1`).
2. **Confirm that `core/grade.hpp`'s four values serve `RND-3`** when the renderers declare grades in code. *Recommendation:* yes, with no Rendering code in this stage.

**Behaviors are Core's** (`SL6`): CPU `reference`, Vulkan `banded`. An empty slot computes nothing, and `CORE-1` refuses an attached registry on Vulkan.

## Airflow's needs from stage 5

Cameron's order is stages 4 → 5 → 6 → 7, airflow before SPH, with the bar "F1 level, nasa level aerodynamics". The airflow spec assigns stage 5 its grades (§1.7: "Grades per backend and the grade check (CPU reference, Vulkan banded) — Stage 5"; §9 M5). Core's answers add `absent` on the CPU for a GPU-only module (Q10).

| Need | Covered by |
|---|---|
| Airflow (both tiers) and the airspeed sensor declare CPU `reference` and Vulkan `banded` | Task 3 (`ModuleDesc::grade`). They are optional built-ins outside the standard set; Physics declares them when the modules exist (M3, M4) |
| §15: a device-resident module has a CPU reference, and its Vulkan results are banded against it | Task 3's rule: Vulkan `banded` needs a CPU `reference`, for every module. Residency itself is stage 7; no grade depends on it |
| Q10: a GPU-only airflow is refused unless it declares `absent` on the CPU | Tasks 3 and 5. CPU `absent` is declarable and always refused on the CPU, and Task 3's rule then caps its Vulkan grade at `best-effort` |
| Q9 (A): banded at 32³ in the gate, and a recorded 128³ statistics measurement per device of record | One grade per module and backend, not per grid size (Decisions). The 128³ record is Test/Docs' and Physics' |
| The F1/NASA bar: a consumer can demand it, and see the grade it gets | Task 5: `create(…, minimum)` refuses a set holding any `best-effort` module, such as the analytic wake if stepping ever read it (`PHY-3`). `Simulation::grade()` names the weakest module, and `vulkan_device_report()` names the device |
| Roles | **Not needed.** Coupling is airflow's presence, not a swapped tier (airflow spec §1.7, §3.4). The two tiers coexist per region; choosing between them is region binding, stage 7 |
| A component type | **Not needed.** The airflow region is world-file v3 data (§7), not a component. `fluid` stays reserved for SPH ("SPH, `PHY-4`"), unaffected by airflow coming first |
| Device-limit refusals and the VRAM budget check | **Stage 7.** Cameron chose Q11 (A), not (C), so they are not pulled into stage 5 |

## Review Focus

1. **Nothing moves.** Every golden, the standard identity and the schema hash are unchanged at every task's head. Grades and roles change no byte the identity spells. Pinned by `StateDeclarationsLeaveTheOrderAndTheIdentityAlone`, by Tasks 3 and 6, and by `Determinism.DigestsMatchTheCommittedGoldenCorpus`.
2. **A table read from any world is the table.** Nothing returns silently empty for world 3. Pinned in Task 1.
3. **A declared read cannot write.** The read path holds only a const pointer, and the cost per step is nil. Pinned in Task 2.
4. **Every grade refusal names the module, and none reads a device.** Pinned in Task 5 by tests that run in the Docker leg.
5. **`banded` means what `TD-14` says,** and a Vulkan `reference` or a Vulkan `banded` without a CPU `reference` cannot be declared. Pinned in Task 3.
6. **`fluid` cannot be built by any path:** attach, the loader, or a menu that reads the table. Pinned in Task 7.

---

### Task 1: A configuration table reads the same from every world (`L6`)

**The defect.** A table view has `world_count` 1 (`simulation.cpp:1046-1053` ms), so `world_rows<float>(view, w)` returns an empty span for every `w > 0` (`state_view.hpp:43` ms). A pass reading its table inside its world loop gets the table in world 0 and nothing, silently, in every other world. The documented convention, "read with `world_rows<float>(view, 0)`" (`module.hpp:289-291`, `state_view.hpp:8-9` ms), hides it. `ModuleTables.APassBindsItsOwnTableAndAnotherModulesOptionally` pins `world_count` 1 (`test_module_schedule.cpp:2626`, `:2632` ms). No standard module reads a table yet; Physics' propulsion will.

**Files:**
- Modify: `engine/state/state_view.hpp`. Add `StateView::shared` and `table_rows<T>`. `world_rows` broadcasts a shared view. The header comment drops the `(view, 0)` convention.
- Modify: `engine/sim/simulation.cpp`. `rebuild_views()`'s table branch sets `world_count = layout_.world_count` and `shared = true`. The comment above it follows (`:1036-1041` ms).
- Modify: `engine/sim/module.hpp`. `ConfigTableDecl`'s comment (`:289-291` ms) says a table is shared and read with `table_rows`.
- Test: `tests/test_module_schedule.cpp`. The two pins at `:2626` and `:2632` become `2u`, the world count, with a `shared` check. They pinned the defect.

**Interfaces:**
```cpp
// state/state_view.hpp
// StateView gains:  bool shared = false;  // a configuration table: one set of rows every world reads
// world_rows<T>(v, w): for a shared view, the whole table for every w < world_count; otherwise unchanged
template <class T> [[nodiscard]] std::span<const T> table_rows(const StateView& v) noexcept;
//   the rows of a shared view, with no world; empty if absent, not shared, or sizeof(T) != elem_size
```

- [ ] **Step 1: Write the failing tests.**

```cpp
namespace {
// "wmassed": massed's table and rows, read from INSIDE the world loop, as a pass
// that also reads per-world rows naturally does.
void copy_mass_each_world_pass(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 2) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        const std::span<const float> masses = spade::world_rows<const float>(ctx.state[0], w);
        const std::span<MassCopyRow> out = spade::world_rows<MassCopyRow>(ctx.state[1], w);
        if (out.empty()) continue;
        out[0].first = masses.empty() ? -1.0f : masses[0];
        out[0].count = static_cast<uint32_t>(masses.size());
    }
}
constexpr QuantityAccess kEachWorldAccess[] = {{"wmassed.masses", Access::read},
                                               {"wmassed.mass_copies", Access::write}};
constexpr PassDecl kEachWorldPasses[] = {
    {.name = "copy", .phase = Phase::forces, .access = kEachWorldAccess, .cpu = &copy_mass_each_world_pass}};
}  // namespace

TEST(ModuleTables, APassRunningAnyWorldReadsTheWholeTable) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "wmassed", .passes = kEachWorldPasses, .state = kMassCopyArrays, .tables = kMassTables});
    auto sim = spade::Simulation::create(two_world_set(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->register_model(quad_of_mass(1.5f)).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    for (uint32_t w = 0; w < 2; ++w) {
        const MassCopyRow row = (*sim->module_rows<MassCopyRow>("mass_copies", w))[0];
        EXPECT_EQ(row.count, 1u) << "world " << w << " reads the one table every world shares";
        EXPECT_EQ(row.first, 1.5f) << "world " << w;
    }
}
```

  One more test:
  - `ModuleTables.TableRowsReadsATableWithNoWorld`. It runs its own two-world simulation with `massed`, registers one model and steps once, so it depends on no other test's captures. On the captured `g_masses_view`:
    - `table_rows<float>` is `{1.5f}`;
    - it is empty for `mass_copies`' own view (an array), an absent view and a wrong row type;
    - `world_rows<const float>(g_masses_view, 1)` is the table, and `(…, 2)` is empty, outside the set.
- [ ] **Step 2:** Build `spade_tests`. `table_rows` is a stub that returns `{}`, and `shared` is declared. Expected: both new tests fail on behaviour (world 1 reads `count` 0 and `first` -1), and so do the two edited pins (`world_count` is 1). Commit the tests and the stub: `test(core): a configuration table must read the same from every world, not only world 0 (L6)`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run `-Filter "ModuleTables|ModuleViews|ModuleScratch|ModuleVehicleRows"`. Commit: `fix(core): a configuration table's view is shared by every world; table_rows reads it with no world (L6)`.
- [ ] **Step 5: The task gate.** Expected: the cut's total + 2, 0 failed. Record the cut's total at the start of this task (`TD-8`).

**Done when:** a pass running any world reads its table's rows, and no comment or test still says a table is read at world 0.

### Task 2: A declared read cannot write through its view

**The hazard.** `StateView::data` is `std::byte*` for every binding (`state_view.hpp:30` ms), so `world_rows<T>` with a non-const `T` hands a declared read writable rows. A pass that declared a read can change state that the hazard graph never ordered. Its result then depends on set order, and no output test catches that. Tables are just as writable today, though they are documented read-only (`state_view.hpp:8`).

**Files:**
- Modify: `engine/state/state_view.hpp`. `data` becomes `const std::byte*`. Add `writable`. `world_rows<T>` takes `data` for a const `T` and `writable` otherwise.
- Modify: `engine/sim/module.hpp` and `module_schedule.cpp`. `CompiledBinding` gains `writable`, set by `binding_of` (`:152-177` ms) for a write or an accumulation of an array or a scratch, and never for a table.
- Modify: `engine/sim/simulation.cpp`. `rebuild_views()` sets `writable` from the binding. `g_no_table_rows` becomes `const`.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
// state/state_view.hpp
struct StateView {
    const std::byte* data = nullptr;  // null when absent
    std::byte* writable = nullptr;    // data, for a declared write or accumulation; null for a read and every table
    uint32_t elem_size = 0;
    uint32_t world_count = 0;
    uint32_t capacity_per_world = 0;
    bool shared = false;              // Task 1
    [[nodiscard]] bool present() const noexcept { return data != nullptr; }
};
// world_rows<const T>: any present view.  world_rows<T>, T not const: a writable view only; a read view yields no rows.
// sim/module.hpp -- CompiledBinding gains:  bool writable = false;
```

- [ ] **Step 1: Write the failing tests.**

```cpp
namespace {
// "peeker" declares a READ of tally's counts, and tries to add 100 through it.
void peek_and_poke_pass(const spade::physics::SubstepContext& ctx) noexcept {
    if (ctx.state.size() != 1) return;
    for (uint32_t w = 0; w < ctx.worlds.size(); ++w) {
        for (TallyRow& row : spade::world_rows<TallyRow>(ctx.state[0], w)) row.count += 100u;
    }
}
constexpr QuantityAccess kPeekAccess[] = {{"tally.tally_counts", Access::read}};
constexpr PassDecl kPeekPasses[] = {
    {.name = "peek", .phase = Phase::forces, .access = kPeekAccess, .cpu = &peek_and_poke_pass}};
}  // namespace

TEST(ModuleViews, APassThatDeclaredAReadCannotWriteThroughItsView) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "tally", .passes = kTallyPasses, .state = kTallyArrays});
    set.push_back({.name = "peeker", .passes = kPeekPasses});
    auto sim = spade::Simulation::create(two_world_set(), 2'000'000, 2, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(3).has_value());
    for (uint32_t w = 0; w < 2; ++w) {
        EXPECT_EQ((*sim->module_rows<TallyRow>("tally_counts", w))[0].count, 6u) << "world " << w << ": tally's own count only";
    }
}
```

  Two more tests:
  - `ModuleViews.AReadViewYieldsOnlyConstRows`, on views captured from running passes:
    - for an array read, a scratch read and a table, `world_rows<const T>` is non-empty and `world_rows<T>` is empty;
    - for a write view, both are non-empty and alias the same rows.
  - `ModuleSchedule.ABindingRecordsWhetherItsAccessWrites` (compile): `writable` is true for the tally's write and the stamped scratch's write, and false for the copier's read, a table read and an optional read of an absent module.
- [ ] **Step 2:** Build. `CompiledBinding::writable` is declared and never set, and `StateView::writable` is declared and never read. Expected: all three fail on behaviour (the count reads 606). The red run also names any existing test whose pass writes through a read; the survey found none (`test_module_schedule.cpp:1864-1893`, `:2354-2356`, `:2769-2817` ms). Commit: `test(core): a pass must not write through a view it declared as a read`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter plus `ModuleSchedule|ModuleState|ModuleRows|ModuleStreams`. Commit: `fix(core): a declared read's view holds only a const pointer; only a write or an accumulation can yield mutable rows`.
- [ ] **Step 5: The task gate.** Expected: Task 1's total + 3.

**Done when:** a read view holds no mutable pointer, the step's cost is unchanged, and every existing views test passes unedited.

### Task 3: Modules declare a grade per backend

**Files:**
- Create: `engine/core/grade.hpp`. Add `Grade` and `grade_name`, header-only.
- Modify: `engine/sim/module.hpp`. Add `GradeDecl`, `ModuleDesc::grade`, `CompiledModule`, `CompiledSchedule::modules`, `SetGrade` and `set_grade`. The header comment (`:7-10` ms) drops "Grades and roles join the descriptor in later stages".
- Modify: `engine/sim/module_schedule.cpp`. Three new refusals, and `out.modules` in set order. `compile_schedule`'s doc lists the refusals, and its identity paragraph says grades are not spelt.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
// core/grade.hpp, namespace spade
enum class Grade : uint8_t { absent = 0, best_effort = 1, banded = 2, reference = 3 };  // ordered: higher is stronger
[[nodiscard]] constexpr std::string_view grade_name(Grade g) noexcept;  // "absent", "best-effort", "banded", "reference"; "" if unknown
// sim/module.hpp, namespace spade::modules
struct GradeDecl {
    Grade cpu = Grade::best_effort;     // undeclared: the weakest grade that runs
    Grade vulkan = Grade::best_effort;
};
// ModuleDesc gains, after scratch:  GradeDecl grade{};
struct CompiledModule { std::string name; uint32_t version = 1; GradeDecl grade{}; };
// CompiledSchedule gains:  std::vector<CompiledModule> modules{};  // set order
struct SetGrade { Grade grade = Grade::reference; std::string module{}; };  // the lowest grade, and the first module at it
[[nodiscard]] SetGrade set_grade(const CompiledSchedule& schedule, compute::BackendKind backend);
```

The new refusals, each `invalid_argument` naming the module:
1. a grade outside the enum, on either backend;
2. a Vulkan `reference` (`L4`);
3. a Vulkan `banded` without a CPU `reference` (`L4`: a band is measured against the reference; §15).

A CPU `absent` is legal; Task 5 refuses it on the CPU. Grades are not spelt into the identity.

- [ ] **Step 1: Write the failing tests.**

```cpp
TEST(ModuleGrades, AVulkanReferenceIsRefusedAtRegistration) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "golden_gpu", .grade = {.cpu = Grade::reference, .vulkan = Grade::reference}});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::invalid_argument);
    EXPECT_NE(s.error().context.find("golden_gpu"), std::string::npos);
}

TEST(ModuleGrades, ASetsGradeIsItsWeakestModuleInSetOrder) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "rough", .grade = {.cpu = Grade::best_effort, .vulkan = Grade::best_effort}});
    set.push_back({.name = "rougher", .grade = {.cpu = Grade::best_effort, .vulkan = Grade::absent}});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const spade::modules::SetGrade cpu = spade::modules::set_grade(*s, spade::compute::BackendKind::cpu);
    EXPECT_EQ(cpu.grade, Grade::best_effort);
    EXPECT_EQ(cpu.module, "drag") << "undeclared until Task 4: the first module at the lowest grade";
    const spade::modules::SetGrade gpu = spade::modules::set_grade(*s, spade::compute::BackendKind::vulkan);
    EXPECT_EQ(gpu.grade, Grade::absent);
    EXPECT_EQ(gpu.module, "rougher");
}
```

  Four more tests:
  - `ModuleGrades.VulkanBandedNeedsACpuReference`:
    - refused: Vulkan `banded` with CPU `banded`, `best-effort` or `absent`;
    - accepted: CPU `reference` with Vulkan `banded`, and CPU `absent` with Vulkan `best-effort`, a GPU-only module (airflow's Q10).
  - `ModuleGrades.AnUnknownGradeIsRefused`: `static_cast<Grade>(7)` on either backend.
  - `ModuleGrades.AnUndeclaredModuleIsBestEffortOnBothBackends`, through `CompiledSchedule::modules`, which also carries name and version in set order.
  - `ModuleGrades.GradesLeaveTheIdentityAlone`: the standard set with every module's grade changed compiles to `kStandardIdentity`. `grade_name` spells all four values.

  `ASetsGradeIsItsWeakestModule…`'s first expectation becomes `reference` and `drag`'s place moves to `rough` in Task 4; that edit is Task 4's.
- [ ] **Step 2:** Build. `core/grade.hpp`, `GradeDecl`, `ModuleDesc::grade` and `CompiledModule` are declared, and `set_grade` is a stub that returns `{}`. Expected: all six fail on behaviour, except `GradesLeaveTheIdentityAlone`, which is green at red (a pin). Commit: `test(core): modules must declare a grade per backend; a Vulkan reference, or a Vulkan band with no CPU reference, is refused`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run `-Filter "ModuleGrades|ModuleSchedule|StandardModules|ModuleSimulation|ModuleSnapshot"`. Commit: `feat(core): modules declare a grade per backend; compile_schedule checks its shape (L3, L4)`.
- [ ] **Step 5: The task gate.** Expected: Task 2's total + 6.

**Done when:** any module can declare a grade per backend, the two `L4` shapes cannot be declared, and the identity pin is unchanged.

### Task 4: The built-ins declare their grades (joint with Physics)

**Owner:** Core writes, Physics reviews (inputs 1–3 above). Rendering confirms that it declares nothing here.

**Files:**
- Modify: `engine/sim/standard_modules.cpp`. Each of the ten modules declares `.grade`, with a comment citing `PHY-2` (physics modules) or `SL6` (`behaviors`). `dynamic_contact`'s comment names the Gauss-Seidel row.
- Test: `tests/test_module_schedule.cpp`. `ASetsGradeIsItsWeakestModule…`'s CPU expectation now reads `best-effort` and `rough`.

- [ ] **Step 1: Write the failing tests.**

```cpp
// Transcribed from docs/design/physics/04-verification.md, "Grades" (PHY-2), not
// read off the declarations (TD-4). environment: Physics' answer to input 2.
// behaviors: Core's (SL6).
TEST(StandardModules, DeclareTheGradesPhysicsSigned) {
    const std::map<std::string, std::pair<Grade, Grade>> expected = {
        {"drag", {Grade::reference, Grade::banded}},        {"dryden", {Grade::reference, Grade::banded}},
        {"imu", {Grade::reference, Grade::banded}},         {"rotor", {Grade::reference, Grade::banded}},
        {"gnss", {Grade::reference, Grade::banded}},        {"behaviors", {Grade::reference, Grade::banded}},
        {"static_contact", {Grade::reference, Grade::banded}}, {"dynamic_contact", {Grade::reference, Grade::banded}},
        {"integrate", {Grade::reference, Grade::banded}},   {"environment", {Grade::reference, Grade::banded}}};
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_EQ(s->modules.size(), expected.size());
    for (const spade::modules::CompiledModule& m : s->modules) {
        const auto it = expected.find(m.name);
        ASSERT_NE(it, expected.end()) << m.name;
        EXPECT_EQ(m.grade.cpu, it->second.first) << m.name;
        EXPECT_EQ(m.grade.vulkan, it->second.second) << m.name;
    }
}
```

  One more test:
  - `StandardModules.AreReferenceOnTheCpuAndBandedOnVulkan`: through `set_grade`, the standard set is `reference` on the CPU and `banded` on Vulkan, and `drag`, the first module in set order, is named for both.
- [ ] **Step 2:** Build. Expected: both fail on behaviour; every module is still `best-effort`. Commit: `test(core): the built-ins must declare PHY-2's grades`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter. `StateDeclarationsLeaveTheOrderAndTheIdentityAlone` stays green at `kStandardIdentity`. Commit: `feat(core): the built-ins declare their grades (PHY-2; behaviors SL6)`. Physics reviews this commit.
- [ ] **Step 5: The task gate.** Expected: Task 3's total + 2, and every golden unchanged.

**Done when:** the standard set grades `reference` on the CPU and `banded` on Vulkan, Physics has signed off the table, and the identity is master's.

### Task 5: `create()` checks grades against the backend and a minimum

**Files:**
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - `create()` gains `minimum_grade` and runs the checks in the order Decisions gives, after the existing Vulkan refusals (`simulation.cpp:387-419` ms) and before `validate_world_set`.
  - Add `grade()`, which accounts for an attached behavior registry.
  - `step()` refuses an attached non-empty registry under a minimum above `best-effort`, beside `CORE-1`'s refusal (`:1109-1113` ms), on both backends. `step(0)` runs no behaviors and is unaffected.
  - The `create()` doc comment (`simulation.hpp:578-611` ms) lists the new codes.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
// sim/simulation.hpp, class Simulation
[[nodiscard]] static Result<Simulation> create(const WorldSetDesc& desc, uint64_t dt_ns, uint32_t substeps,
                                               const compute::BackendDesc& backend = {},
                                               const modules::ModuleSet& module_set = modules::standard_modules(),
                                               Grade minimum_grade = Grade::best_effort);
//   invalid_argument: minimum_grade below best_effort (absent is always refused, so it asks for nothing)
//   unavailable: a module absent on this backend, or a set below minimum_grade -- naming the module and its grade
[[nodiscard]] modules::SetGrade grade() const;  // the set's grade on this backend; best-effort, "behaviors", while a non-empty registry is attached
// private:  Grade minimum_grade_ = Grade::best_effort;
```

- [ ] **Step 1: Write the failing tests.**

```cpp
TEST(ModuleGrades, ASetBelowTheMinimumIsRefusedNamingTheModule) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "rough", .grade = {.cpu = Grade::best_effort, .vulkan = Grade::best_effort}});
    const auto refused = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set, Grade::reference);
    ASSERT_FALSE(refused.has_value());
    EXPECT_EQ(refused.error().code, spade::Code::unavailable);
    EXPECT_NE(refused.error().context.find("'rough'"), std::string::npos);
    EXPECT_NE(refused.error().context.find("best-effort"), std::string::npos);
    EXPECT_TRUE(spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set, Grade::best_effort).has_value());
    EXPECT_TRUE(spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, set).has_value()) << "no minimum";
}

TEST(ModuleGrades, NoVulkanSetMeetsAReferenceMinimum) {
    // Refused before any device is touched, so this runs in the Docker leg too.
    const auto s = spade::Simulation::create(one_body_world(), 2'000'000, 2,
                                             {.kind = spade::compute::BackendKind::vulkan},
                                             spade::modules::standard_modules(), Grade::reference);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::unavailable);
    EXPECT_NE(s.error().context.find("'drag'"), std::string::npos) << "the first module, in set order, below reference";
}
```

  Five more tests:
  - `ModuleGrades.AnAbsentModuleIsRefusedOnItsBackendByName`:
    - `ghost`, with no passes and Vulkan `absent`, is refused on Vulkan with `unavailable`, naming it and the backend, with no minimum given. It is accepted on the CPU;
    - `cpu_ghost`, with CPU `absent` and Vulkan `best-effort`, is refused on the CPU.
  - `ModuleGrades.WithNoMinimumEveryGradeButAbsentIsAccepted`: on the CPU, standard + `rough`, and standard + a CPU-`banded` module. Accepting a set on Vulkan needs a device; `GpuModuleSchedule*` already creates standard Vulkan sets with no minimum, and Step 4 runs them, so no new `gpu` test is needed.
  - `ModuleGrades.AMinimumBelowBestEffortIsInvalid`: `Grade::absent` gives `invalid_argument`.
  - `ModuleGrades.ASimulationReportsItsGradeAndWeakestModule`: the standard set on the CPU is `reference`; standard + `rough` is `best-effort`, naming `rough`.
  - `ModuleGrades.AnAttachedBehaviorRegistryIsBestEffort`: a CPU simulation with minimum `banded` and one kinematic mover attached:
    - `step(1)` returns `unavailable`, naming `behaviors`, and the tick does not move;
    - `grade()` reads `best-effort`, naming `behaviors`, while the registry is attached, and `reference` once it is detached;
    - with no minimum, the same simulation steps.
- [ ] **Step 2:** Build. The parameter exists and is ignored, `grade()` returns `{}`, and nothing else changes. Expected: all seven fail on behaviour. `NoVulkanSetMeets…` either creates on a device or fails in the backend stub, without the module's name. Commit: `test(core): create() must refuse an absent module and a set below the requested grade, naming the module`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter plus `GpuModuleSchedule*|Behavior*|KinematicMover*|VulkanDirtyTracking*`. Expected: every existing Vulkan refusal keeps its message. Commit: `feat(core): create() checks the set's grades against its backend and a minimum (L3, L6)`.
- [ ] **Step 5: The task gate.** Expected: Task 4's total + 7, with the `gpu` count unchanged.

**Done when:** spec §13's grade refusals pass ("a set below the minimum is refused, naming the module"; "an `absent` module on Vulkan is refused"), every existing caller behaves as before, and a consumer can ask for reference grade and be refused on Vulkan.

### Task 6: Roles, starting with `dynamic_contact`

**Files:**
- Modify: `engine/sim/module.hpp`. Add `RoleDecl`, `kRoles`, `ModuleDesc::role`, `CompiledModule::role`, `CompiledRole`, `CompiledSchedule::roles` and `check_required_roles`.
- Modify: `engine/sim/module_schedule.cpp`. Two refusals: a role not in `kRoles`, and a role filled twice (both `invalid_argument`, the second naming the role and both modules). They run with the module-level checks, **before the pass graph is built**. Two fillers of `dynamic_contact` are also two writers of `body.pose` with no edge, so a later check would refuse them as a hazard, not as a role. Add `check_required_roles`.
- Modify: `engine/sim/standard_modules.cpp`. `dynamic_contact` declares `.role = "dynamic_contact"`.
- Modify: `engine/sim/simulation.cpp`. `create()` calls `check_required_roles` after `check_builtin_arrays` (`:387` ms).
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
```cpp
struct RoleDecl { std::string_view name; bool required = false; };  // required: exactly one filler; otherwise at most one
inline constexpr RoleDecl kRoles[] = {{.name = "dynamic_contact", .required = true}};
// ModuleDesc gains, after grade:  std::string_view role{};  // a kRoles name; empty fills none
// CompiledModule gains:  std::string role{};
struct CompiledRole { std::string role; std::string module; };
// CompiledSchedule gains:  std::vector<CompiledRole> roles{};  // one per filled role, in kRoles order
[[nodiscard]] Result<void> check_required_roles(const CompiledSchedule& schedule);  // invalid_argument, naming the role
```

The role is not spelt into the identity. The filler's name already is (Decisions).

- [ ] **Step 1: Write the failing tests.**

```cpp
namespace {
// "contact_alt": a second filler of dynamic_contact. It runs Gauss-Seidel's CPU
// function with no recipe, so it is CPU-only, which is all a role test needs.
constexpr QuantityAccess kAltAccess[] = {{"body.pose", Access::write}};
constexpr std::string_view kAltAfter[] = {"static_contact.resolve"};
constexpr PassDecl kAltPasses[] = {{.name = "resolve", .phase = Phase::constraints, .access = kAltAccess,
                                    .after = kAltAfter, .cpu = &spade::physics::pass_collision_dynamic}};
[[nodiscard]] ModuleDesc contact_alt() { return {.name = "contact_alt", .passes = kAltPasses, .role = "dynamic_contact"}; }
[[nodiscard]] spade::modules::ModuleSet standard_with_contact_alt() {  // replaced in place, so set order holds
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    for (ModuleDesc& m : set) {
        if (m.name == "dynamic_contact") m = contact_alt();
    }
    return set;
}
}  // namespace

TEST(ModuleRoles, ARoleFilledTwiceOrNotAtAllIsRefused) {
    spade::modules::ModuleSet twice = spade::modules::standard_modules();
    twice.push_back(contact_alt());
    const auto both = compile_schedule(twice);
    ASSERT_FALSE(both.has_value());
    EXPECT_EQ(both.error().code, spade::Code::invalid_argument);
    for (const char* name : {"dynamic_contact", "contact_alt"}) EXPECT_NE(both.error().context.find(name), std::string::npos) << name;

    spade::modules::ModuleSet none = spade::modules::standard_modules();
    std::erase_if(none, [](const ModuleDesc& m) { return m.name == "dynamic_contact"; });
    ASSERT_TRUE(compile_schedule(none).has_value()) << "a schedule may be compiled without its roles";
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, {}, none);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::invalid_argument);
    EXPECT_NE(sim.error().context.find("role 'dynamic_contact'"), std::string::npos);
}
```

  Five more tests:
  - `ModuleRoles.ARoleIsOneOfTheKnownRoles`: `dynamic_contct` is refused, naming the module and the role.
  - `ModuleRoles.AnAlternativeFillsTheRoleAndSteps`: `standard_with_contact_alt()` creates on the CPU and steps 3. `schedule().roles` is `{dynamic_contact: contact_alt}`.
  - `ModuleRoles.AnotherRoleChoiceChangesTheIdentity`: its identity is not `kStandardIdentity`. **Green at red**: a pin of the Decisions argument.
  - `ModuleSnapshot.RestoreUnderAnotherRoleChoiceIsRefused` (spec §13, `L2`): a blob from the standard set does not restore into `standard_with_contact_alt()`. The refusal is restore's identity message (`simulation.cpp:2515` ms), as `RestoreIntoAnotherModuleSetIsRefused` expects. **Green at red**, a pin.
  - `StandardModules.FillTheDynamicContactRole`: `roles` is `{dynamic_contact: dynamic_contact}`, and the identity is still `kStandardIdentity`.
- [ ] **Step 2:** Build. `ModuleDesc::role` and `CompiledSchedule::roles` are declared, and `check_required_roles` is a stub that returns `{}`. Expected: four fail on behaviour, and the two pins pass. Commit: `test(core): a role must be filled exactly once, and another role choice must be refused on restore`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run the focused filter plus `ModuleRoles|Determinism|GoldenCorpus`. Commit: `feat(core): roles; dynamic_contact is filled exactly once (spec section 7)`.
- [ ] **Step 5: The task gate.** Expected: Task 5's total + 6.

**Done when:** spec §13's role refusals pass ("a role filled twice or not at all"), `L2` under another role choice is pinned, and the standard identity is master's.

### Task 7: Component availability; `fluid` reserved

**Files:**
- Modify: `engine/objects/component.hpp`. Add `ComponentAvailability`, `ComponentTypeInfo`, `kComponentTypes` (one row per type id, in id order, with its name) and `component_available<T>()`. `FluidComponent`'s comment (`:90-97`) says it is reserved.
- Modify: `engine/objects/graph.{hpp,cpp}`.
  - `kNames` (`graph.cpp:30-35`) goes; `component_type_name` and `component_type_id_from_name` read `kComponentTypes`. One table, one site (`TD-9`).
  - Add `component_types()`.
  - `attach<T>` (`graph.hpp:114-126`) gets a `static_assert(component_available<T>())`.
- Modify: `engine/objects/serialize.cpp`. The loader refuses a reserved type with `unavailable`, naming the object, the type and its reason (`:213-223`). The writer's `fluid` branch (`:135-137`) goes; nothing can attach one.
- Modify, in the red commit, three tests that attach a `FluidComponent`. None is a golden.
  - `test_objects_component.cpp:132`: `MaskBitIsTheTypeId` uses `behavior`, bit 8.
  - `test_objects_serialize.cpp:70-71`, `:97-98`: `RoundTripsEveryComponentType` becomes `RoundTripsEveryConstructibleType`.
  - `test_objects_determinism.cpp:156`: `attach_a_full_complement` attaches every constructible type, and its comment says so.
- Test: `tests/test_objects_component.cpp`, `tests/test_objects_serialize.cpp`

**Interfaces:**
```cpp
// objects/component.hpp, namespace spade::objects
enum class ComponentAvailability : uint8_t { available = 0, reserved = 1 };
struct ComponentTypeInfo {
    ComponentTypeId id;
    std::string_view name;                // the serialization key
    ComponentAvailability availability;
    std::string_view reason;              // reserved only: why, and what will provide it
};
inline constexpr ComponentTypeInfo kComponentTypes[kComponentTypeCount] = {
    /* transform … behavior: available, reason "" */
    {ComponentTypeId::fluid, "fluid", ComponentAvailability::reserved, "SPH, PHY-4"}};
template <class T> [[nodiscard]] consteval bool component_available() noexcept;
[[nodiscard]] std::span<const ComponentTypeInfo> component_types() noexcept;  // every type, in id order: what every enumerator reads
```

- [ ] **Step 1: Write the failing tests.**

```cpp
// The names are spelt here, not read from the table (TD-4): SL5's frozen order.
constexpr std::string_view kFrozenNames[] = {"transform", "body",          "mesh",   "material", "collider",
                                             "sensor",    "force_element", "camera", "behavior", "fluid"};

TEST(ComponentAvailability, EveryTypeIsListedInIdOrderWithItsAvailability) {
    const std::span<const ComponentTypeInfo> types = spade::objects::component_types();
    ASSERT_EQ(types.size(), std::size(kFrozenNames));
    for (uint32_t i = 0; i < types.size(); ++i) {
        EXPECT_EQ(static_cast<uint32_t>(types[i].id), i);
        EXPECT_EQ(types[i].name, kFrozenNames[i]);
        EXPECT_EQ(spade::objects::component_type_name(types[i].id), kFrozenNames[i]) << "the name lookup reads the same table";
        const bool fluid = types[i].id == ComponentTypeId::fluid;
        EXPECT_EQ(types[i].availability,
                  fluid ? ComponentAvailability::reserved : ComponentAvailability::available) << types[i].name;
        EXPECT_EQ(types[i].reason, fluid ? "SPH, PHY-4" : "") << types[i].name;
    }
}
```

  Two more tests:
  - `ComponentAvailability.TheLoaderRefusesAReservedTypeByName`. Feed `from_json` the object JSON that `RoundTripsEveryComponentType` wrote for `fluid` before this task. Expected: `unavailable`, with `fluid` and `SPH, PHY-4` in the message. Today it loads.
  - `ComponentAvailability.OnlyTheReservedTypeIsUnconstructible`: `component_available<T>()` is false for `FluidComponent` and true for the other nine, checked at run time over the traits.
- [ ] **Step 2:** Build. `component_types()` is a stub that returns `{}`, and `component_available` returns `true`. Expected: the three new tests fail on behaviour, and the three edited tests pass. Commit: `test(core): the component-type table must carry availability, and a reserved type must not load`.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build. Run `-Filter "Component*|ObjectGraph*|ObjectsSerialize*|ObjectGraphDeterminism*"`. Expected: `ObjectGraphDeterminism.*` stays green against the committed corpus. Commit: `feat(core): component availability beside the type table; fluid is reserved (SPH, PHY-4)`.
- [ ] **Step 5: The task gate.** Expected: Task 6's total + 3.

**Done when:** `fluid` cannot be attached or loaded, `component_types()` lists all ten types with availability and reason, and the ids are unchanged (`SL5`).

### Task 8: Prose follows the code

- **Code comments**, built once as a compile check:
  - `sim/module.hpp`'s header (`:7-10` ms) and `compile_schedule`'s doc: grades, roles, their refusals, and the identity paragraph;
  - `state/state_view.hpp`: shared tables, and const rows for reads;
  - `simulation.hpp`'s `create()` doc and the census (`:1564-1610` at stage 4): `grade()` is a read.
- **Docs, on master after the merge:**
  - `core/07-status.md`:
    - the modules row adds grades and roles, and "nine built-ins" becomes ten (`environment` joined in stage 3);
    - "Grades, grade check at `create()`" becomes **Yes**.
  - `core/01-modules-and-scheduler.md`: the grade line gains `absent` and this plan's table of meanings under `TD-14`; roles; shared tables and read-only views.
  - `core/02-state-and-snapshot.md`: the role choice is covered by the identity's module names; no format change.
  - `CHANGELOG.md`:
    - `core/grade.hpp`;
    - `GradeDecl`, `ModuleDesc::grade` and `role`, `kRoles`, `set_grade`, `check_required_roles`, `CompiledSchedule::modules` and `roles`;
    - `Simulation::create`'s `minimum_grade` and `Simulation::grade()`;
    - `StateView`'s `data` (now const), `writable` and `shared`, and `table_rows`;
    - `component_types()`, and the loader's refusal of `fluid`.
- **Owners' follow-ups:** Physics, `04-verification.md`'s last line (input 5). Rendering, `RND-3`'s status ("no grade API exists in code yet" now holds for renderers only). Interface, the editor's component menu reads `component_types()` (`interface/plans/2026-10-05-editor-design.md:29`, `:114`).
- **Tell the lead,** for `../../consumers.md`:
  - `create()` gains a trailing defaulted parameter;
  - `grade()`;
  - a saved object graph holding a `fluid` component no longer loads (none is in the repo);
  - a developer pass that wrote through a declared read now writes nothing;
  - snapshot format unchanged (v3), and the identity unchanged.
- [ ] Commit: `docs(core): comments follow grades, roles and component availability`.

## Open questions for the lead

1. **The table fix: share or refuse?** You suggested `table_rows` plus a refusal or an assert in `world_rows` on a table view. *Recommendation:* share (Task 1), with `table_rows` as asked.
   - A refusal inside a `noexcept` pass can only be an empty span, which is the defect itself in Release, or an `assert`, which runs in Debug only. The tree has no death tests, so an assert cannot be pinned red then green.
   - Sharing removes the failure mode at no cost, and both presets test it.
   - If you prefer the refusal, Task 1 keeps `world_count` 1 and adds a Debug `assert`, and the two edited pins stay as they are.
2. **Read-only views: const by type, or also an assert?** *Recommendation:* const by type alone (Task 2). The read path then holds no mutable pointer, and the cost is nil. A pass that asks a read view for mutable rows gets none, which its own output test shows. A silent unordered write is what nothing would catch.
3. **What grade does an undeclared module get?** *Recommendation:* `best-effort` on both backends. The alternatives:
   - require a declaration: every test-local module in `test_module_schedule.cpp` (dozens) is edited;
   - derive Vulkan `absent` from missing recipes: a second site for stage 2's per-pass refusal (`TD-9`).
4. **Attached behaviors under a minimum grade.** *Recommendation:* refuse at `step()`, in `CORE-1`'s place (Task 5), because `set_behaviors` returns `void`. No caller passes a minimum today. The alternative is to document behaviors as ungraded until they become modules. That lets a reference-minimum world run ungraded code silently (`L3`).
5. **Where does the minimum go?** *Recommendation:* a trailing defaulted parameter of `create()`. `BackendDesc` is the result-neutral knob block (`engine A7`), and each of its knobs owes an invariance test; a minimum is a requirement, not a knob. A new options struct would change every call site.
6. **May a set omit a stateful built-in?** This is stage 4's open question 1, deferred to here. *Recommendation:* keep refusing it (`check_builtin_arrays`). Stage 6 builds the "typed call refuses `unavailable` when the module is absent" pattern; open it when a consumer asks.
7. **A scenario's role key** (spec §7). *Recommendation:* defer to Jacobi (`PHY-5`), the first second filler. Until then every scenario's choice is the standard set's.
8. **Two silent empties remain in `world_rows`:** a wrong row type and a world outside the set. Stage 4 pins both (`test_module_schedule.cpp:1954-1955` ms). *Recommendation:* leave them; both are a pass's own programming error, and its tests show it. Say if you want them in Task 2.

## Notes for later stages

- **Stage 6, the lock,** declares CPU `reference` with its new golden, and Vulkan `banded` with `pos` and `vel` bit-exact (spec §9). Its fixed binding gives the tables their device mirror, and gives propulsion and airflow theirs.
- **Stage 7, airflow's groundwork** (§15), grades nothing new. A device-resident module declares CPU `reference` and Vulkan `banded` through `GradeDecl`, and Task 3's rule already holds it to its CPU reference. Stage 7 also carries:
  - residency's sync-or-refuse;
  - the hashed module configuration (the standard identity stays, since the standard set has no configured modules);
  - the device-limit refusals and the budget check (Q11 (A), not (C));
  - the cost query (Q10).
- **Jacobi (`PHY-5`)** becomes the second filler of `dynamic_contact`, with its own golden and bands. Scenarios then name their choice (open question 7).
- **An aero-tier role** (BEMT, T1/T2) joins `kRoles` as optional: at most one filler.
- **SPH (`PHY-4`)** declares Vulkan `absent` ("no GPU path") and defines `fluid`, so `fluid`'s row turns available. With objects in stepping (§11), availability becomes per set: a type is available when a module in the set defines it, and `ModuleDesc` gains the types it defines.
- **Interface:** the editor's component menu, and later the sandbox's choice of a minimum grade.
- **Behaviors** get a declared grade when they become modules (spec §3, "Behaviors").
