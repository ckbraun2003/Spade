# Module API Stage 2 Implementation Plan — the GPU chain derived from the schedule

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Vulkan step records its dispatches from the compiled module schedule. No hand-kept pass table remains, and a pass the GPU cannot run is refused by name.

**Architecture:** A pass may name a **GPU recipe**, which is one of the built-in kernels the recorder knows how to dispatch.
- `compile_schedule` refuses a recipe paired with any CPU function other than that recipe's own.
- On Vulkan, `create()` refuses a pass with no recipe. Otherwise it derives the ordered list of GPU passes from the schedule and hands that list to the Vulkan backend.
- The recorder walks the list every substep. Per-pass timings are keyed by pass name.
- The recipe-to-kernel knowledge stays beside the kernels, in `compute/`. That layer sits below `sim/` and cannot read the module types.

**Tech Stack:** C++23, Vulkan through volk, Slang kernels (unchanged), GoogleTest through ctest.

**Spec:** `2026-10-02-module-api-design.md` §5 (approved 2026-10-02). Stage 1 plan: `2026-10-02-module-api-plan.md`, merged as `062e1fa`, with its docs in `b03585a`.

## Global Constraints

- Every golden digest and every GPU parity band stays unchanged. For the standard set, the GPU dispatches and barriers are exactly today's: 9 + S dispatches per substep, in the same order (`CORE-2`).
- No allocation in a step. The GPU pass list is built in `create()`.
- Refuse, never skip (`L6`). A pass the GPU cannot run makes `create()` return `unavailable`, naming the pass.
- `compute/` must not include `sim/` headers. The layering is `sim` → `compute`.
- Builds and tests run only through `scripts\build.ps1` and `scripts\test.ps1`, in the foreground, in a slot the lead hands out.
- Code lives in `../spade-wt/core`, on branch `core/modules-stage2`.
- Commit only named paths. Never push.

## Decisions this plan makes

- **A recipe is a built-in kernel, named by an enum in `compute/backend.hpp`.** Developer kernels come later, with the GPU module ABI. The enum:
  `GpuRecipe { none, behaviors_kinematic, medium_update, rotors, drag, behaviors_force, collision_static, collision_dynamic, integrate, sensor_imu, sensor_gnss }`
- **A recipe is honest only with its CPU twin.**
  - `builtin_cpu_for(recipe)` in `standard_modules.cpp` names the one CPU function each recipe stands for.
  - `compile_schedule` refuses any other pairing, as `invalid_argument` and on every backend, because such a declaration is false wherever it runs.
  - This generalises stage 1's fix (`068e6ff`). A set that keeps a built-in's name with another function either declares no recipe, which Vulkan refuses, or declares the recipe, which the compiler refuses.
- **The behavior slots get two recipes that dispatch nothing.** With no registry attached they do nothing on either backend, and with a registry attached the Vulkan step still refuses (`CORE-1`). Because pairing covers them, a developer pass cannot claim to be inert. There is no separate "inert" flag.
- **A pass with no recipe is refused on Vulkan.** The message contains `module set` and the pass's `<module>.<pass>` name.
- **The identity is unchanged.** It spells names, versions and order, not functions or kernels, and the recipe is bound to the function. No identity constant moves.
- **Timings are per pass.** `PassDurationsNs` becomes a list of `{pass name, ns}` in schedule order.
  - The bench keeps its six counters by summing named passes: `gpu_force_elements_ns` = rotors + drag, and `gpu_sensor_synthesis_ns` = IMU + GNSS.
  - `tests/bench/bench_sim.cpp` belongs to Test/Docs, so Test/Docs reviews that edit.
- **`PassTimestamps` loses its hand-written moves.** It is only ever held by `unique_ptr`, the same lesson as defect 5.
- **This plan departs from the stage-1 plan's outline of stage 2 in two places:**
  1. **The outline:** `PassDecl::gpu` would be a function that appends dispatches. **Here:** it is a recipe enum. A function would need the recorder's types in `sim/`, and `compute/` cannot call back into `sim/`. A developer-supplied recording waits for the GPU module ABI.
  2. **The outline:** a module would "declare a pass inert" on Vulkan. **Here:** only the built-in behavior functions can, through their recipes. An inert flag a developer could set would let a pass change state on the CPU and nothing on the GPU, which `L6` forbids.

## Review Focus

1. **A developer module with no recipe on Vulkan** is refused and named. It is never silently left out of the GPU chain. Pinned in Task 1.
2. **A built-in recipe with another CPU function** is refused at compile, on every backend. Pinned in Task 1. Stage 1's `AStandardNamedSetWithAnotherFunctionIsRefusedOnVulkan` stays green, now through the no-recipe refusal.
3. **The standard set's GPU chain is the same, dispatch for dispatch,** as today's: 9 + S dispatches per substep, same order. Pinned by `RecordedChainHasABarrier...` (unchanged) and Task 2's recorded-names test, and gated by the GPU parity suite.
4. **A set whose passes all have recipes but run in another order** runs in that order on the GPU, not today's order. Pinned in Task 2, which swaps two commuting sensor modules.
5. **Timings after the set changes** come back from `vulkan_pass_durations_ns()` as one entry per GPU pass, named and in order. No pass is reported in another pass's slot. Pinned in Task 2.

---

### Task 1: Passes name their GPU recipe; dishonest or kernel-less passes are refused

**Files:**
- Modify: `engine/compute/backend.hpp` (add `GpuRecipe`)
- Modify: `engine/sim/module.hpp`. Adds `PassDecl::gpu`, `CompiledPass::gpu`, `builtin_cpu_for` and `gpu_passes`.
- Modify: `engine/sim/module_schedule.cpp`. Adds the pairing check and carries the recipe into `CompiledPass`.
- Modify: `engine/sim/standard_modules.cpp`. Gives the built-ins their recipes and defines `builtin_cpu_for`.
- Modify: `engine/sim/simulation.cpp`. Adds the per-pass Vulkan refusal before stage 1's whole-set comparison, which stays until Task 2.
- Test: `tests/test_module_schedule.cpp`

**Interfaces:**
- Produces:
  - `enum class compute::GpuRecipe : uint8_t { none = 0, behaviors_kinematic, medium_update, rotors, drag, behaviors_force, collision_static, collision_dynamic, integrate, sensor_imu, sensor_gnss };`
  - `struct compute::GpuPass { std::string name; GpuRecipe recipe; };` in `compute/backend.hpp`, where `name` is `<module>.<pass>`;
  - `PassDecl::gpu` and `CompiledPass::gpu`, both `compute::GpuRecipe` and defaulting to `none`;
  - `modules::PassFn modules::builtin_cpu_for(compute::GpuRecipe) noexcept`, which returns `nullptr` for `none`;
  - `std::vector<compute::GpuPass> modules::gpu_passes(const CompiledSchedule&)`, which returns every pass in schedule order, refused or not. Task 2's recorder and tests consume it.

- [ ] **Step 1: Write the failing tests.** Append them to `tests/test_module_schedule.cpp`:

```cpp
TEST(StandardModules, EveryPassNamesARecipePairedWithItsOwnFunction) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    for (const auto& p : s->passes) {
        EXPECT_NE(p.gpu, spade::compute::GpuRecipe::none) << p.module << "." << p.pass;
        EXPECT_EQ(p.cpu, spade::modules::builtin_cpu_for(p.gpu)) << p.module << "." << p.pass;
    }
}

TEST(ModuleSchedule, ARecipeWithAnotherCpuFunctionIsRefused) {
    static constexpr PassDecl liar[] = {{.name = "lift", .phase = Phase::forces, .access = kHoverAccess,
                                         .cpu = &hover_pass, .gpu = spade::compute::GpuRecipe::rotors}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = liar});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_EQ(s.error().code, spade::Code::invalid_argument);
    EXPECT_NE(s.error().context.find("hover.lift"), std::string::npos) << s.error().context;
}

TEST(ModuleSimulation, ADeveloperPassWithNoRecipeIsRefusedOnVulkanByName) {
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "hover", .passes = kHoverPasses});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan},
                                               set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_EQ(sim.error().code, spade::Code::unavailable);
    EXPECT_NE(sim.error().context.find("hover.lift"), std::string::npos) << sim.error().context;
}

TEST(ModuleSchedule, GpuPassesFollowTheSchedule) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    const auto g = spade::modules::gpu_passes(*s);
    ASSERT_EQ(g.size(), s->passes.size());
    for (size_t i = 0; i < g.size(); ++i) {
        EXPECT_EQ(g[i].name, s->passes[i].module + "." + s->passes[i].pass);
        EXPECT_EQ(g[i].recipe, s->passes[i].gpu);
    }
}
```

`one_body_world()`, `hover_pass` and `kHoverAccess` already exist in that file's anonymous namespace, defined at about line 280. `ARecipeWithAnotherCpuFunctionIsRefused` uses `kHoverAccess`, so it goes after that namespace. The refusals happen in `create()` before any device is opened (`simulation.cpp:261-290`), so the tests above run on a machine without a GPU.

- [ ] **Step 2: Build the test object in a slot.** Expected: it fails to compile because `GpuRecipe`, `PassDecl::gpu`, `builtin_cpu_for` and `gpu_passes` are not declared.
- [ ] **Step 3: Implement.**
  - `compute/backend.hpp`:
    - add the enum with a comment: "a built-in kernel the recorder knows how to dispatch; the two behavior recipes dispatch nothing (`CORE-1`)";
    - add `GpuPass`.
  - `module.hpp`:
    - add the field to both structs, placed last in `PassDecl` so existing designated initialisers still compile;
    - include `compute/backend.hpp`;
    - declare `builtin_cpu_for` and `gpu_passes`.
  - `module_schedule.cpp`:
    - while validating each pass, refuse with `invalid_argument` if `p.gpu != none && p.cpu != builtin_cpu_for(p.gpu)`, using the message "pass '<module>.<pass>' names a built-in GPU kernel but carries another CPU function";
    - copy `gpu` into `CompiledPass`;
    - define `gpu_passes`.
  - `standard_modules.cpp`:
    - give each built-in its recipe:

      | Pass | Recipe |
      |---|---|
      | `behaviors.kinematic` | `behaviors_kinematic` |
      | `dryden.advance` | `medium_update` |
      | `rotor.forces` | `rotors` |
      | `drag.forces` | `drag` |
      | `behaviors.force` | `behaviors_force` |
      | `static_contact.resolve` | `collision_static` |
      | `dynamic_contact.resolve` | `collision_dynamic` |
      | `integrate.integrate` | `integrate` |
      | `imu.synthesize` | `sensor_imu` |
      | `gnss.synthesize` | `sensor_gnss` |

    - define `builtin_cpu_for` as a `switch` over the same pairs, returning the `physics::pass_*` functions.
  - `simulation.cpp`, on Vulkan, before stage 1's comparison:
    - for each compiled pass in order, if `gpu == none`, return `unavailable` with "module set: pass '<module>.<pass>' has no GPU kernel; it runs only on the CPU";
    - keep stage 1's comparison after this loop. Task 2 removes it.
- [ ] **Step 4: Build, then run** `scripts\test.ps1 -Filter "ModuleSchedule|ModuleSimulation|ModuleSnapshot|StandardModules"`. Expected: all pass. The stage-1 Vulkan tests stay green, because their messages still contain `module set`.
- [ ] **Step 5: Commit**, test first and then the implementation: `feat(core): passes name their GPU kernel; a false or missing one is refused`.

### Task 2: The recorder walks the schedule's GPU passes

**Files:**
- Modify `engine/compute/backend.hpp`:
  - `PassDurationsNs` becomes `{ bool supported; std::vector<PassDuration> passes; uint32_t implausible_samples; double ns_of(std::string_view pass) const noexcept; }`, with `struct PassDuration { std::string pass; double ns; }`. `ns_of` returns 0 when the pass is absent.
  - `RecordedChain` gains `std::vector<std::string> passes`, listing each pass once in recorded order.
- Modify `engine/compute/vulkan/backend.{hpp,cpp}` and `backend_stub.cpp`: the signature becomes `VulkanBackend::create(const BackendDesc&, const StepShape&, std::span<const GpuPass> passes)`.
- Modify `engine/compute/vulkan/step_recorder.{hpp,cpp}`:
  - `create` takes the list and keeps a copy.
  - `record()` walks the list every substep, with a `switch` on the recipe. Each recipe emits exactly the dispatches today's slot emits:

    | Recipe | Dispatches |
    |---|---|
    | `behaviors_kinematic`, `behaviors_force` | none |
    | `medium_update` | `kPipelineMedium` on the world grid |
    | `rotors`, `drag` | `kPipelineRotors`, `kPipelineDrag` on the body grid |
    | `collision_static` | `kPipelineCollision` on the body grid |
    | `collision_dynamic` | `kPipelineGridBuild` on the grid-entry grid, then the bitonic stages, then `kPipelineCollisionDynamic` on the world grid |
    | `integrate` | `kPipelineIntegrate` on the body grid |
    | `sensor_imu`, `sensor_gnss` | their pipelines, on the sensor grid |

  - A `none` recipe is a bug at this point (`create()` refuses it), so `record()` returns `internal`.
  - One timestamp mark is written after each pass, including the two that dispatch nothing, as the `kNoDispatch` slots do today.
  - Delete `kPassPipeline`, `kPassGrid`, `PassGrid`, `kNoDispatch`, `kForceElementsSlot`, `kSensorSynthesisSlot`, `kCollisionDynamicSlot`, the 8-slot loop and the interim note.
- Modify `engine/compute/vulkan/timestamps.{hpp,cpp}`:
  - `create(ctx, substeps, std::vector<std::string> pass_names)`;
  - marks per substep = passes + 1;
  - `read_durations_ns` fills one `PassDuration` per name;
  - delete `kPassTimestampMarksPerSubstep` and the hand-written moves.
- Modify `engine/sim/simulation.{hpp,cpp}`:
  - pass `modules::gpu_passes(schedule_)` to `VulkanBackend::create`;
  - add `Result<compute::RecordedChain> vulkan_recorded_chain() const`, modelled on `vulkan_pass_durations_ns()` (`unavailable` on the CPU);
  - delete stage 1's whole-set comparison and its `kStandard`.
- Modify `tests/bench/bench_sim.cpp:234-239` (Test/Docs reviews):
  - `gpu_force_elements_ns` = `ns_of("rotor.forces") + ns_of("drag.forces")`;
  - `gpu_sensor_synthesis_ns` = `ns_of("imu.synthesize") + ns_of("gnss.synthesize")`;
  - each of the other four counters is one `ns_of`.
- Modify `tests/test_gpu_state_mirror.cpp`. Its five `VulkanBackend::create` calls (lines 243, 341, 542, 649 and 682) gain `kStandardGpuPasses`, a file-local `static const std::vector<GpuPass>` built from `gpu_passes(*compile_schedule(standard_modules()))`.
- Modify `tests/test_determinism.cpp`: delete `Schedule.TheCompiledStandardSetFollowsTheGpuRecordersOrder`. The recorder now follows the schedule by construction, and Task 2's recorded-names test pins it.
- Modify `tests/test_module_schedule.cpp`:
  - delete `ModuleSimulation.ANonStandardSetOnVulkanIsRefusedUntilStage2`, which Task 1's `ADeveloperPassWithNoRecipeIsRefusedOnVulkanByName` supersedes;
  - in the comment above `AStandardNamedSetWithAnotherFunctionIsRefusedOnVulkan`, replace "Stage 1's GPU recorder would still run the stock rotor kernel" with: "a pass without a recipe has no kernel, and the recipe can only be claimed with the built-in function".
- Create `tests/test_gpu_module_schedule.cpp`. Register it in `tests/CMakeLists.txt` inside the `if(SPADE_VULKAN)` block, beside the other device tests.

**Interfaces:**
- Consumes Task 1's `GpuRecipe`, `GpuPass`, `CompiledPass::gpu` and `gpu_passes`.
- Produces `compute::PassDuration`, `PassDurationsNs::ns_of`, `RecordedChain::passes`, the new `VulkanBackend::create`, and `Simulation::vulkan_recorded_chain()`.

- [ ] **Step 1: Write the failing tests** in the new `tests/test_gpu_module_schedule.cpp`.
  - `test_module_schedule.cpp` is compiled with `SPADE_VULKAN` off as well, so it must not include a Vulkan header.
  - The new file includes `compute/vulkan/context.hpp` for `vulkan_available()`. It carries its own `one_body_world()`, a copy of the helper in `test_module_schedule.cpp`.
  - The tests need a device, so their suite is `Gpu*`. That gives them the `gpu` label by the name-prefix rule in `AppendSpadeLabels.cmake`. Each one skips without a device.

```cpp
namespace {
const std::vector<std::string> kStandardGpuOrder = {
    "behaviors.kinematic", "dryden.advance",         "rotor.forces",        "drag.forces",
    "behaviors.force",     "static_contact.resolve", "dynamic_contact.resolve", "integrate.integrate",
    "imu.synthesize",      "gnss.synthesize"};

spade::compute::BackendDesc vulkan() { return {.kind = spade::compute::BackendKind::vulkan}; }
}  // namespace

TEST(GpuModuleSchedule, TheRecorderRecordsTheSchedulesPassesInOrder) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, vulkan());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto chain = sim->vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value()) << chain.error().context;
    EXPECT_EQ(chain->passes, kStandardGpuOrder);
}

// imu and gnss share no quantity, so swapping them in the set swaps them in
// the schedule; the GPU must follow the schedule, not a remembered order.
TEST(GpuModuleSchedule, ReorderedSensorsRecordInScheduleOrder) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    ASSERT_EQ(set[2].name, "imu");
    ASSERT_EQ(set[4].name, "gnss");
    std::swap(set[2], set[4]);
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, vulkan(), set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    const auto chain = sim->vulkan_recorded_chain();
    ASSERT_TRUE(chain.has_value()) << chain.error().context;
    std::vector<std::string> expected = kStandardGpuOrder;
    std::swap(expected[8], expected[9]);
    EXPECT_EQ(chain->passes, expected);
}

TEST(GpuModuleSchedule, DurationsAreOnePerPassByName) {
    if (!spade::compute::vulkan_available()) GTEST_SKIP();
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 2, vulkan());
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    const auto d = sim->vulkan_pass_durations_ns();
    ASSERT_TRUE(d.has_value()) << d.error().context;
    if (!d->supported) GTEST_SKIP() << "this device cannot time compute work";
    std::vector<std::string> names;
    for (const auto& p : d->passes) names.push_back(p.pass);
    EXPECT_EQ(names, kStandardGpuOrder);
}
```

- [ ] **Step 2: Build the test object in a slot.** Expected: it fails to compile because `vulkan_recorded_chain`, `RecordedChain::passes` and `PassDurationsNs::passes` do not exist.
- [ ] **Step 3: Implement as listed under Files.** The `switch` body is today's 8-slot loop body, regrouped by recipe. Keep every dispatch's pipeline, grid, push constants and barrier byte for byte.
- [ ] **Step 4: Build all targets, then run the focused tests, then the full suite.** Expected:
  - `RecordedChainHasABarrierBetweenEveryAdjacentDispatchPair` still passes (9 + S);
  - every `GpuParity*` and `GpuInvariance*` test passes with unchanged bands;
  - every golden is unchanged;
  - total = Task 1's total + 3 (the new `GpuModuleSchedule` tests) − 2 (the deleted `Schedule` test and the superseded stage-1 test), with the `gpu` count + 3.
  - The bench builds. Test/Docs runs it when it suits them; the tests do not wait on it.
- [ ] **Step 5: Commit**, test first and then the implementation: `feat(core): the Vulkan step records the schedule's GPU passes; no hand-kept table`.

### Task 3: Prose follows the code

**Files:**
- Comment-only: `engine/compute/backend.hpp` (the `PassDurationsNs` comment still cites `kNoDispatch` and Gravity/Publish), `engine/compute/vulkan/timestamps.{hpp,cpp}`, `engine/shaders/dryden.slang`, `engine/shaders/kernels/forces_drag.slang`, `rotors.slang` and `integrate.slang`, and `tests/test_gpu_parity.cpp`.
  - These are the stage-1 review's stale citations of `kSchedule`, `pass_force_elements`, `pass_gravity` and slot numbers.
  - Replace each with the pass's name (for example `drag.forces`) or its phase.
  - First, find them all with `git grep -n "kSchedule\|pass_force_elements\|pass_gravity\|pass_sensor_synthesis\|slot [0-9]"`.
- Docs, in the main tree after the merge:
  - `docs/design/core/07-status.md`: "GPU chain derived from the schedule" becomes **Yes**.
  - `docs/design/core/04-api-and-backend.md`: "Today it is a separate table" is replaced by the recipe model: one kernel per recipe, the pairing check, and refusal by name.

- [ ] **Step 1:** Edit the comments, then build in a slot as a compile check.
- [ ] **Step 2:** Run the same `git grep`. Expected: no hit names a removed symbol or a pass slot. Hits for an unrelated "slot", such as the params ring, stay.
- [ ] **Step 3: Commit:** `docs(core): GPU-side comments name passes and phases, not slots`.
- [ ] **Step 4 (after the merge):** commit the docs on master, path-scoped.

## Notes for later stages (from the stage-1 review and the drone-builder design)

- **Stage 3:** the field registry needs small fixed-array value types, for the acoustic and RF bands.
- **Stage 4:**
  - validate module-owned quantities against the module's declared arrays (stage-1 review, minor 1);
  - add a duty-command host call;
  - add a design-frame vehicle-state read (`DBE-013`).
- **Stage 6 (GPU module ABI):** developer kernels become recipes the developer supplies. The pairing rule carries over: such a kernel declares its CPU twin.
- **Stage-1 minors still open.** Fold each into whichever stage next touches its file:
  - pass names containing `.`;
  - the identity-0 restore message (use `hex64`);
  - the unused `<string_view>` in `physics/schedule.hpp`;
  - no unknown-phase test;
  - the wording of the cycle message.
