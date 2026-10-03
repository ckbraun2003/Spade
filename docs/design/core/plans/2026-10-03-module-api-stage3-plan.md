# Module API Stage 3 Implementation Plan — fields and sample buffers

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rotors, drag and Integrate read gravity, density and wind as **fields**: values a provider writes once per world per substep, in the Fields phase, into a sample buffer. They no longer compute the medium inline or read `WorldParams` directly. Every golden and every GPU band stays where it is.

**Architecture:**
- A field is declared by the module that provides it: a name, a value kind (scalar, vec3, or a fixed array of bands) and a unit.
- The provider's **sample pass** writes the quantity `field.<name>`. Readers declare a read of it, so the stage-1 scheduler orders them with no new machinery.
- The values live in a per-world sample row. It is **scratch**: not registered state, recomputed every substep before any reader, so snapshots, walks and digests do not change.
- The CPU row and the GPU row share one std430 prefix for the built-in fields.
- Rotors and drag read through a `Medium` adapter over the stored row, so their model functions do not change. Integrate takes gravity by value.

**Tech Stack:** C++23, Slang kernels, Vulkan through volk, GoogleTest through ctest.

**Spec:** `2026-10-02-module-api-design.md` §6 (approved 2026-10-02); §13's field tests. Stage 2 plan: `2026-10-03-module-api-stage2-plan.md`. This stage starts after stage 2 merges and builds on its recipes.

## Global Constraints

- Every golden digest and every GPU parity band stays unchanged. A stored sample is the same operations on the same inputs as today's inline value: bitwise on the CPU, and bitwise against the GPU's own inline value on the GPU.
- No allocation in a step. The sample rows are sized at `create()`.
- The sample buffer is scratch. It is not in the registry, the walk (`kWalkEntryCount` stays 22), `schema_hash` or any digest.
- Refuse, never skip (`L6`). A field read with no provider, two providers of one field, and a provider with no GPU kernel on Vulkan (stage 2's rule) are all refused at `create()`, by name.
- Physics' model functions (`apply_rotors`, `apply_drag`) keep their signatures, because they are the pure functions the builder calls (`DBE-009`). `integrate_bodies` takes gravity by value; Physics reviews that change.
- `-Wall -Wextra -Wpedantic` clean under gcc, and warning-free under MSVC.
- Builds and tests run only in a slot the lead hands out, in the foreground. Code lives in `../spade-wt/core`, on branch `core/modules-stage3`. Commit only named paths. Never push.

## Decisions this plan makes

- **The registry lives in the module set.** `ModuleDesc` gains `std::span<const FieldDecl> fields`, the fields that module provides.
  - The registry is the union of the set's declarations, in set order. The built-ins come first, at fixed places (below).
  - `compile_schedule` refuses a field declared twice (this stage has one region, the whole world, so one provider per field). It also refuses a read of `field.<name>` that no module declares, and a provider module with no pass that writes its field.
- **Value kinds:** `scalar` (1 float), `vec3` (3 floats), and `bands` (N floats, 1 ≤ N ≤ `kMaxFieldBands` = 32). The band kind is what the acoustic and RF fields need (drone-builder design §4). No provider of a band field ships in this stage. A developer module can declare one and run it on the CPU; on Vulkan it is refused, because its sample pass has no recipe.
- **The built-in providers are two new passes.**
  - `environment.sample`: a new stateless `environment` module that provides `gravity` (vec3, m/s²) and `density` (scalar, kg/m³) from `WorldParams`.
  - `dryden.sample`: a second pass of the `dryden` module that provides `wind` (vec3, m/s) as `WorldParams::wind` plus the gust.
  - Both are position-independent: one value per world, the same as `Medium::sample` today.
  - Both are Fields passes. Hazards place `dryden.sample` after `dryden.advance`, because it reads `dryden.state`. The placement-first kinematic behaviors run before both.
- **The sample row's layout is fixed for the built-ins:** `FieldSampleRow { vec3 gravity; float density; vec3 wind; float _pad; }`, 32 bytes, std430.
  - It is mirrored in `layouts.slang` with `@cpp` directives, so `layout_check` pins it.
  - Developer fields follow the built-in prefix on the CPU only, in registry order, from float 8, at a per-world stride fixed at `create()`.
  - The GPU buffer holds the built-in prefix only, because no developer field can run on Vulkan yet.
- **Sample points are declared but resolve per world.** The spec's per-body and per-element points matter only for a position-dependent provider (SPH, later, with Physics), and this stage refuses that kind of provider. Readers here read the world's row.
- **Rotors and drag read through `SampledMedium`**, a `final` `Medium` whose `sample(params, pos)` returns `{row.density, row.wind}`. `apply_rotors` and `apply_drag` are unchanged. When SPH arrives, readers move to point-indexed sampling with Physics.
- **Integrate reads `field.gravity`.**
  - `integrate_bodies(span<BodyState>, glm::vec3 gravity, float h)` replaces the `const WorldParams&` overload, which only ever read `params.gravity` (integrator.cpp:14).
  - The sandbox's `pin_force` keeps reading `WorldParams::gravity`, which is still true state. Its owner can switch it later.
- **Undeclared reads are fixed.** `rotor.forces` and `drag.forces` read the medium but declared no read of it. They now declare reads of `field.density` and `field.wind`.
- **Every commit stays green; the GPU follows in two steps.**
  - Task 2 adds the two sample passes with their recipes, `environment_sample` and `dryden_sample`, which **dispatch nothing yet**.
  - The GPU kernels still compute the medium inline at that commit, so the GPU runs the same experiment as before. The pairing rule holds: each recipe stands for its CPU function, and nothing on the GPU reads a sample until Task 3.
  - Task 3 gives the recipes their kernels and switches the GPU readers.
  - Without this step, the new passes would have no recipe at Task 2, Vulkan would refuse the standard set, and every gpu test would fail until Task 3.
- **GPU: two small kernels and two recipes (Task 3).**
  - The kernels are `field_environment.slang` and `field_dryden.slang`, one thread per world; the recipes are `environment_sample` and `dryden_sample`.
  - They write a new derived buffer, `field_samples` (binding 26, zero-filled at create, device-written, never read back on the step path).
  - `rotors.slang` and `forces_drag.slang` read `field_samples[world]` in place of `dryden_medium_sample(...)`, and `integrate.slang` reads it in place of `world_params[world].gravity`.
  - The standard chain grows from 9 + S to **11 + S** dispatches per substep. That is a deliberate tally change: `RecordedChainHasABarrier...` and the recorder's header move with it, and the bench gains two `gpu_pass.*` counters.
  - In exchange, rotors and drag stop recomputing the gust once per body.
- **Host reads do not change.** `Simulation::sample_medium` still calls the provider's CPU function on current state (spec §6). A new diagnostic, `Simulation::field_samples(world)`, returns the CPU row as of the last substep, for tests. On Vulkan it reads the device buffer, like `vulkan_grid_entries()`.

## Review Focus

1. **Bit-identity, CPU.** The stored `wind` equals `DrydenMedium(row, params).sample(...)` bitwise on a turbulent world. Rotor and drag forces, and the whole corpus, are unchanged. Pinned in Task 2, and gated by `Determinism.DigestsMatchTheCommittedGoldenCorpus`.
2. **Ordering.** A Fields provider that reads a pose written by a placement-first pass sees that substep's pose, not the previous one. Pinned in Task 2 with a developer module, because no built-in provider reads poses.
3. **A field read with no provider, or a field with two providers**, is refused at compile, naming the field. It is never silently read as zero. Pinned in Task 1.
4. **Restore, then step.** The sample row is scratch, so the first substep after `restore()` must recompute it before any reader. A run restored mid-flight must match an uninterrupted run bitwise. Pinned in Task 2.
5. **A band field on the CPU** round-trips N values from provider to reader, and is refused on Vulkan by name. Pinned in Tasks 1 and 2.

---

### Task 1: Fields in the module set

**Files:**
- Modify: `engine/sim/module.hpp`. Add `FieldKind`, `FieldDecl`, `kMaxFieldBands` and `ModuleDesc::fields`. Add `CompiledField` and `CompiledSchedule::fields` (name, kind, count and float offset; built-ins first), plus `field_stride`.
- Modify: `engine/sim/module_schedule.cpp`. Build the registry and validate it. `known_quantity` accepts `field.<name>` for a declared field.
- Test: `tests/test_module_schedule.cpp`. The standard set does not change in this task. Its fields arrive with their providers in Task 2, so every test here uses a developer module.

**Interfaces:**
- Produces:
  ```cpp
  enum class FieldKind : uint8_t { scalar = 0, vec3 = 1, bands = 2 };
  inline constexpr uint32_t kMaxFieldBands = 32;
  struct FieldDecl { std::string_view name; FieldKind kind = FieldKind::scalar; uint32_t bands = 0; std::string_view unit; };
  struct CompiledField { std::string name; FieldKind kind; uint32_t count; uint32_t offset; std::string unit; };
  // CompiledSchedule gains:  std::vector<CompiledField> fields;  uint32_t field_stride = 8;
  inline constexpr uint32_t kFieldGravityOffset = 0, kFieldDensityOffset = 3, kFieldWindOffset = 4, kFieldBuiltinFloats = 8;
  ```
- The first `kFieldBuiltinFloats` floats of every row are reserved for the built-ins, whether or not the set provides them, so a developer field's offset never depends on which built-ins are present.
- A module that declares `gravity`, `density` or `wind` must use exactly these kinds and units: `gravity` vec3 m/s², `density` scalar kg/m³, `wind` vec3 m/s. Anything else is refused, because the GPU row's layout depends on them.
- The identity is unchanged: fields are declarations, like access, and stage 1's identity covers names, versions and order only.

- [ ] **Step 1: Write the failing tests** (append to `tests/test_module_schedule.cpp`):

```cpp
TEST(ModuleFields, ABuiltinFieldDeclaredWithAnotherKindIsRefused) {
    static constexpr spade::modules::FieldDecl wind[] = {{.name = "wind", .kind = spade::modules::FieldKind::scalar, .unit = "m/s"}};
    static constexpr QuantityAccess writes[] = {{"field.wind", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "gusts", .passes = p, .fields = wind}};
    EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument);
}

TEST(ModuleFields, AReadOfAFieldNoModuleProvidesIsRefused) {
    static constexpr QuantityAccess reads[] = {{"field.salinity", Access::read}};
    static constexpr PassDecl p[] = {{.name = "probe", .phase = Phase::forces, .access = reads, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "probe", .passes = p});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("field.salinity"), std::string::npos) << s.error().context;
}

TEST(ModuleFields, TwoProvidersOfOneFieldAreRefused) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::scalar, .unit = "dB"}};
    static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    const ModuleDesc set[] = {{.name = "a", .passes = p, .fields = hum}, {.name = "b", .passes = p, .fields = hum}};
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("hum"), std::string::npos) << s.error().context;
}

TEST(ModuleFields, ABandFieldTakesItsCountAfterTheBuiltins) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = 8, .unit = "dB"}};
    static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
    static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = p, .fields = hum});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    EXPECT_EQ(s->fields.back().name, "hum");
    EXPECT_EQ(s->fields.back().offset, spade::modules::kFieldBuiltinFloats);
    EXPECT_EQ(s->fields.back().count, 8u);
    EXPECT_EQ(s->field_stride, spade::modules::kFieldBuiltinFloats + 8u);
}

TEST(ModuleFields, ABandCountOutsideOneToTheMaximumIsRefused) {
    for (const uint32_t n : {0u, spade::modules::kMaxFieldBands + 1u}) {
        const spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = n, .unit = "dB"}};
        static constexpr QuantityAccess writes[] = {{"field.hum", Access::write}};
        static constexpr PassDecl p[] = {{.name = "sample", .phase = Phase::fields, .access = writes, .cpu = &noop}};
        spade::modules::ModuleSet set = spade::modules::standard_modules();
        set.push_back({.name = "acoustic", .passes = p, .fields = hum});
        EXPECT_EQ(code_of(compile_schedule(set)), spade::Code::invalid_argument) << "bands = " << n;
    }
}

TEST(ModuleFields, AProviderModuleWithNoPassWritingItsFieldIsRefused) {
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::scalar, .unit = "dB"}};
    static constexpr PassDecl p[] = {{.name = "idle", .phase = Phase::fields, .cpu = &noop}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = p, .fields = hum});
    const auto s = compile_schedule(set);
    ASSERT_FALSE(s.has_value());
    EXPECT_NE(s.error().context.find("field.hum"), std::string::npos) << s.error().context;
}
```

- [ ] **Step 2:** In a slot, build the test object. Expected: it fails to compile (`FieldDecl`, `ModuleDesc::fields` and `CompiledSchedule::fields` are not declared).
- [ ] **Step 3: Implement.**
  - The registry walk runs in set order: the built-ins first, by name, then the rest in set order.
  - A field name may not be empty and may not contain `.`. A `field.<name>` quantity resolves against the registry.
  - The provider check: for each declared field, some pass of the declaring module has `write` access to `field.<name>`.
- [ ] **Step 4:** Build. Run `-Filter "ModuleFields|ModuleSchedule|StandardModules|ModuleSimulation|ModuleSnapshot"`. Expected: all pass. The standard set's schedule and identity are unchanged.
- [ ] **Step 5: Commit**, tests first, then the implementation: `feat(core): modules declare the fields they provide; reads resolve against them`.

### Task 2: Sample rows on the CPU; readers switch to them

**Files:**
- Modify: `engine/physics/schedule.{hpp,cpp}` (Physics reviews).
  - `WorldSubstepView` gains `std::span<float> fields`, this world's row of `field_stride` floats.
  - New passes: `pass_environment_sample` (gravity and density from `*w.params`) and `pass_dryden_sample` (`params.wind + dryden_turbulence(*w.dryden, *w.dryden_params)`, the exact expression `DrydenMedium::sample` returns, medium.cpp:296-301).
  - `pass_rotor_forces` and `pass_drag` construct `SampledMedium(w.fields)` in place of `DrydenMedium(...)`.
  - `pass_integrate` passes the row's gravity.
- Create: `engine/physics/sampled_medium.hpp`: `SampledMedium final : Medium`, a header-only view over a row.
- Modify: `engine/physics/integrator.{hpp,cpp}` (Physics reviews). `integrate_bodies(std::span<BodyState>, glm::vec3 gravity, float h)`. Its one caller moves.
- Modify: `engine/sim/standard_modules.cpp`.
  - Add the stateless `environment` module and the `dryden.sample` pass. They declare the three built-in fields and write them, and they are paired with their CPU functions and recipes.
  - The `environment` module is appended to the set, so no existing module's set index moves.
  - Add the field reads to rotor, drag and integrate. Integrate keeps its `world.params` read for anything else it touches.
- Modify: `engine/compute/backend.hpp` and `engine/compute/vulkan/step_recorder.cpp`. Add the recipes `environment_sample` and `dryden_sample`, recorded, for now, like the behavior recipes: a timestamp bracket and no dispatch. The GPU kernels still compute the medium inline, so nothing on the GPU reads a sample yet.
- Modify these tests:
  - `tests/test_gpu_module_schedule.cpp`: `kStandardGpuOrder` gains `dryden.sample` and `environment.sample` after `dryden.advance`.
  - `StandardModules.CompileToTodaysOrder`: the same two passes, in the same place.
- Modify: `engine/sim/simulation.{hpp,cpp}`.
  - Add a `std::vector<float> field_rows_` member, sized `world_count * field_stride` at `create()` and zero-filled. It is not registered.
  - `rebuild_views()` points each view at its row.
  - Add `Result<std::vector<float>> field_samples(uint32_t world) const`. It is a diagnostic: it copies the CPU row, and on Vulkan (Task 3) it reads the device row back.
- Test: `tests/test_module_schedule.cpp`, `tests/test_sim_medium.cpp`

**Interfaces:**
- Consumes Task 1's `CompiledField`, `field_stride` and the `kField*Offset` constants.
- Produces:
  - `physics::pass_environment_sample` and `physics::pass_dryden_sample`;
  - `physics::SampledMedium`;
  - the new `integrate_bodies` signature;
  - `Simulation::field_samples(world)`;
  - the recipes `GpuRecipe::environment_sample` and `GpuRecipe::dryden_sample`.

- [ ] **Step 1: Write the failing tests.**

```cpp
// test_sim_medium.cpp: the stored wind is today's inline value, bitwise.
TEST(SimFields, StoredWindIsTheInlineDrydenSampleBitwise) {
    auto sim = make_turbulent_one_world(spade::TurbulenceLevel::moderate);   // file-local helper, seed 0x5EED
    ASSERT_TRUE(sim.step(3).has_value());
    const auto row = sim.field_samples(0);
    ASSERT_TRUE(row.has_value()) << row.error().context;
    const auto inline_sample = sim.sample_medium(0, glm::vec3(0.0f));       // provider's CPU function, current state
    ASSERT_TRUE(inline_sample.has_value());
    const glm::vec3 stored_wind{(*row)[spade::modules::kFieldWindOffset + 0], (*row)[spade::modules::kFieldWindOffset + 1],
                                (*row)[spade::modules::kFieldWindOffset + 2]};
    EXPECT_EQ(std::bit_cast<std::array<uint32_t, 3>>(stored_wind), std::bit_cast<std::array<uint32_t, 3>>(inline_sample->wind));
    EXPECT_EQ(std::bit_cast<uint32_t>((*row)[spade::modules::kFieldDensityOffset]), std::bit_cast<uint32_t>(inline_sample->density));
}

TEST(SimFields, SampleMediumBeforeTheFirstStepIsMeanWindPlusTheInitialGust) {
    auto sim = make_turbulent_one_world(spade::TurbulenceLevel::moderate);
    const auto s = sim.sample_medium(0, glm::vec3(0.0f));
    ASSERT_TRUE(s.has_value());
    EXPECT_NE(s->wind, sim_mean_wind()) << "dryden_init places the filter on its stationary distribution at create()";
}

TEST(SimFields, RestoreThenStepMatchesAnUninterruptedRun) {
    auto a = make_turbulent_one_world(spade::TurbulenceLevel::severe);
    auto b = make_turbulent_one_world(spade::TurbulenceLevel::severe);
    spawn_one_quad(a); spawn_one_quad(b);                                    // rotors read the samples
    ASSERT_TRUE(a.step(5).has_value());
    const auto blob = a.snapshot();
    ASSERT_TRUE(blob.has_value() && b.restore(*blob).has_value());
    ASSERT_TRUE(a.step(5).has_value() && b.step(5).has_value());
    EXPECT_EQ(spade::testing::state_digest(a), spade::testing::state_digest(b));
}
```

```cpp
// test_module_schedule.cpp, after the hover helpers. Pass functions reach their field through a
// file-static offset, set from the compiled schedule before create().
namespace {
uint32_t g_offset = 0;
float g_seen_height = -1.0f;
std::array<float, 8> g_seen_bands{};

[[nodiscard]] uint32_t offset_of(const spade::modules::CompiledSchedule& s, std::string_view name) {
    for (const auto& f : s.fields) if (f.name == name) return f.offset;
    ADD_FAILURE() << "no field " << name;
    return 0;
}
void lift_to_42(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds) w.bodies[0].pos.y = 42.0f;
}
void sample_height(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds) w.fields[g_offset] = w.bodies[0].pos.y;
}
void read_height(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds) g_seen_height = w.fields[g_offset];
}
void sample_bands(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds)
        for (uint32_t i = 0; i < 8; ++i) w.fields[g_offset + i] = 0.5f * static_cast<float>(i);
}
void read_bands(const spade::physics::SubstepContext& ctx) noexcept {
    for (const auto& w : ctx.worlds)
        for (uint32_t i = 0; i < 8; ++i) g_seen_bands[i] = w.fields[g_offset + i];
}
}  // namespace

// The mover (an ordered Fields writer of body.pose) and the provider (a Fields reader of body.pose)
// are ordered by their hazard, so with ONE substep the provider must see 42, not the spawn's 0.
TEST(ModuleFields, AProviderSeesThePoseWrittenEarlierInFieldsTheSameSubstep) {
    static constexpr QuantityAccess move[] = {{"body.pose", Access::write}};
    static constexpr QuantityAccess sample[] = {{"body.pose", Access::read}, {"field.height", Access::write}};
    static constexpr QuantityAccess read[] = {{"field.height", Access::read}};
    static constexpr spade::modules::FieldDecl height[] = {{.name = "height", .kind = spade::modules::FieldKind::scalar, .unit = "m"}};
    static constexpr PassDecl mover[] = {{.name = "lift", .phase = Phase::fields, .access = move, .cpu = &lift_to_42}};
    static constexpr PassDecl probe[] = {
        {.name = "sample", .phase = Phase::fields, .access = sample, .cpu = &sample_height},
        {.name = "read", .phase = Phase::forces, .access = read, .cpu = &read_height}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "probe", .passes = probe, .fields = height});   // declared BEFORE the mover in set order
    set.push_back({.name = "mover", .passes = mover});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    g_offset = offset_of(*s, "height");
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 1, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->spawn(0, spade::BodySpawn{}).has_value());
    ASSERT_TRUE(sim->step(1).has_value());
    EXPECT_EQ(g_seen_height, 42.0f);
}

TEST(ModuleFields, ABandFieldRoundTripsOnTheCpu) {
    static constexpr QuantityAccess sample[] = {{"field.hum", Access::write}};
    static constexpr QuantityAccess read[] = {{"field.hum", Access::read}};
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = 8, .unit = "dB"}};
    static constexpr PassDecl acoustic[] = {
        {.name = "sample", .phase = Phase::fields, .access = sample, .cpu = &sample_bands},
        {.name = "read", .phase = Phase::forces, .access = read, .cpu = &read_bands}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = acoustic, .fields = hum});
    const auto s = compile_schedule(set);
    ASSERT_TRUE(s.has_value()) << s.error().context;
    g_offset = offset_of(*s, "hum");
    auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 1, {}, set);
    ASSERT_TRUE(sim.has_value()) << sim.error().context;
    ASSERT_TRUE(sim->step(1).has_value());
    for (uint32_t i = 0; i < 8; ++i) EXPECT_EQ(g_seen_bands[i], 0.5f * static_cast<float>(i)) << "band " << i;
}

TEST(ModuleFields, ABandFieldIsRefusedOnVulkanByName) {
    static constexpr QuantityAccess sample[] = {{"field.hum", Access::write}};
    static constexpr spade::modules::FieldDecl hum[] = {{.name = "hum", .kind = spade::modules::FieldKind::bands, .bands = 8, .unit = "dB"}};
    static constexpr PassDecl acoustic[] = {{.name = "sample", .phase = Phase::fields, .access = sample, .cpu = &sample_bands}};
    spade::modules::ModuleSet set = spade::modules::standard_modules();
    set.push_back({.name = "acoustic", .passes = acoustic, .fields = hum});
    const auto sim = spade::Simulation::create(one_body_world(), 2'000'000, 1,
                                               spade::compute::BackendDesc{.kind = spade::compute::BackendKind::vulkan}, set);
    ASSERT_FALSE(sim.has_value());
    EXPECT_NE(sim.error().context.find("acoustic.sample"), std::string::npos) << sim.error().context;
}

TEST(StandardModules, ProvideGravityDensityAndWindAtFixedOffsets) {
    const auto s = compile_schedule(spade::modules::standard_modules());
    ASSERT_TRUE(s.has_value()) << s.error().context;
    ASSERT_EQ(s->fields.size(), 3u);
    EXPECT_EQ(s->fields[0].name, "gravity");
    EXPECT_EQ(s->fields[0].offset, spade::modules::kFieldGravityOffset);
    EXPECT_EQ(s->fields[1].name, "density");
    EXPECT_EQ(s->fields[1].offset, spade::modules::kFieldDensityOffset);
    EXPECT_EQ(s->fields[2].name, "wind");
    EXPECT_EQ(s->fields[2].offset, spade::modules::kFieldWindOffset);
    EXPECT_EQ(s->field_stride, spade::modules::kFieldBuiltinFloats);
}
```

The `test_sim_medium.cpp` helpers are file-local:
- `make_turbulent_one_world(level)` builds one world with `Environment::wind = {3, 0, 1}`, the given Dryden level and seed `0x5EED`, at 2 ms and 2 substeps.
- `spawn_one_quad(sim)` registers `vehicles::make_quadrotor(QuadrotorParams{})` and spawns it at (0, 10, 0), with `rotor_omega = hover_command(...)`.
- `sim_mean_wind()` is that `{3, 0, 1}`.
- Reuse the file's existing world helper if one fits.

- [ ] **Step 2:** Build in a slot. Expected: it fails to compile (`field_samples`, `WorldSubstepView::fields`).
- [ ] **Step 3: Implement** as listed under Files.
  - The sample passes write their floats one at a time in the row's layout.
  - `SampledMedium::sample` returns `{fields[kFieldDensityOffset], vec3(fields[4], fields[5], fields[6])}`. It is a copy, not arithmetic.
- [ ] **Step 4:** Build. Run the focused tests, then the full suite. Expected:
  - the new tests pass;
  - every golden passes unchanged (the corpus, `quad_hover`, the viewer trajectories);
  - every gpu test is still green. The two sample recipes record a timestamp bracket and no dispatch, so the barrier test still counts 9 + S, and every GPU band is unchanged, because the kernels have not changed.
- [ ] **Step 5: Commit**, tests first, then the implementation: `feat(core): fields are sampled once per world in Fields, and the medium's readers read them`.

### Task 3: Sample rows on the GPU

**Files:**
- Modify: `engine/shaders/shared/layouts.slang`. Add `FieldSampleRow` with `@cpp` directives, so `layout_check.gen.hpp` asserts the C++ mirror.
- Modify: `engine/shaders/shared/bindings.slang`. Add `field_samples` at binding 26: an `RWStructuredBuffer<FieldSampleRow>`, derived, zero-filled at create.
- Create: `engine/shaders/kernels/field_environment.slang` and `field_dryden.slang`. One thread per world. The Dryden kernel computes `world_params[w].wind + dryden_turbulence(dryden[w], dryden_params[w])`, the same expression `dryden_medium_sample` uses (dryden.slang:340-345).
- Modify: `rotors.slang` and `forces_drag.slang`. Read `field_samples[world].density` and `.wind` in place of `dryden_medium_sample(...)` (rotors.slang:379-380; forces_drag.slang:155-158).
- Modify: `integrate.slang`. Read `field_samples[world].gravity` (integrate.slang:158).
- Modify: `engine/CMakeLists.txt`. Register the two kernels with `spade_slang_kernel_variants`.
- Modify: `engine/compute/vulkan/step_recorder.{hpp,cpp}`. Two pipelines. The two sample recipes now dispatch their kernels on the world grid, in place of Task 2's empty brackets, and the recipe table in the header follows.
- Modify: `engine/compute/vulkan/state_mirror.{hpp,cpp}`. Add `field_samples_` as a derived entry (`derived_entries()` grows to 11) and `read_field_samples()`, a readback in the shape of `read_grid_entries()`. `VulkanBackend` gains the forwarding call, and `backend_stub.cpp` refuses it.
- Modify: `engine/sim/simulation.cpp`. `field_samples(world)` on Vulkan reads the device row back, as a diagnostic only, like `vulkan_grid_entries()`.
- Modify these tests:
  - `tests/test_slang_layouts.cpp`: one more binding in `kExpectedRegistry`, `kScheduleKernels` 12 → 14, and two SPIR-V profiles. Test/Docs reviews.
  - `tests/test_gpu_state_mirror.cpp`: the tally 9 + S becomes 11 + S, and the comment moves with it.
  - `tests/test_gpu_module_schedule.cpp`: `kStandardGpuOrder` gains the two passes.
- Test: `tests/test_gpu_parity.cpp`, with one new case.

- [ ] **Step 1: Write the failing tests.** Update the three expectations above, then add:

```cpp
// test_gpu_parity.cpp: the device's stored samples, after one step. Gravity and density are copies,
// so they match the CPU bitwise. Wind carries Dryden's GPU band, which the existing rotor and drag
// parity cases already gate through every force; here it only has to be finite and present.
TEST_F(GpuParityTest, StoredFieldSamplesMatchTheCpuCopiesBitwise) {
    if (!vulkan_available()) GTEST_SKIP();
    auto cpu = make_turbulent_world_set(BackendKind::cpu);   // the file's existing parity world-set builder, two worlds
    auto gpu = make_turbulent_world_set(BackendKind::vulkan);
    ASSERT_TRUE(cpu.step(1).has_value() && gpu.step(1).has_value());
    for (uint32_t w = 0; w < 2; ++w) {
        const auto c = cpu.field_samples(w);
        const auto g = gpu.field_samples(w);
        ASSERT_TRUE(c.has_value() && g.has_value());
        for (uint32_t i = 0; i < 4; ++i)
            EXPECT_EQ(std::bit_cast<uint32_t>((*c)[i]), std::bit_cast<uint32_t>((*g)[i])) << "world " << w << " float " << i;
        for (uint32_t i = 4; i < 7; ++i) EXPECT_TRUE(std::isfinite((*g)[i])) << "world " << w << " wind " << i;
    }
}
```

- [ ] **Step 2:** Build in a slot. Expected: it fails, first at compile (`read_field_samples`). With that stubbed, the tally and order tests fail at 9 + S, until the kernels dispatch.
- [ ] **Step 3: Implement** as listed under Files.
- [ ] **Step 4:** Build every target. Run the focused tests, then the full suite. Expected:
  - every `GpuParity*` and `GpuInvariance*` band is unchanged;
  - every golden is unchanged;
  - the barrier test passes at 11 + S;
  - the gpu count is master's + 1, for the one new parity case;
  - `test_slang_layouts` passes with 14 kernels and binding 26.
- [ ] **Step 5: Commit**, tests first, then the implementation: `feat(core): the GPU samples fields once per world, and its readers read them`. The commit body carries the TD-8 note: two new brackets, and rotors and drag lose their per-body gust work.

### Task 4: Prose follows the code

- Code comments: `medium.hpp`'s note that consumers call `sample()` inline; schedule.hpp's pass descriptions; rotors.slang's and forces_drag.slang's medium notes. Keep the line counts in the `.slang` files.
- Docs, on master after the merge:
  - `core/07-status.md`: "Field registry and sampling" becomes **Built for gravity, density and wind**, with the band kind and position-dependent providers noted as later work.
  - `core/02-state-and-snapshot.md`: the sample buffer is scratch, and is named as such.
  - `CHANGELOG.md`: `field_samples()`, the field declarations, `integrate_bodies`' signature, and the 11 + S chain.
- [ ] Commit: `docs(core): field comments follow the sample buffer`.

## Notes for later stages

- **SPH** (with Physics): position-dependent providers, per-body and per-element sample points, readers that sample by point index, and a provider that reads the grid. It needs a second build of the grid in Fields (Physics' requirement §1).
- **Acoustic and RF:** their providers are emitters' source terms, with free-field propagation; cameras draw them as field channels. They are best-effort and never read by stepping (`PHY-3`). The band kind is ready.
- **Regions** (spec §11) replace "one provider per field" with "one per field per region".
