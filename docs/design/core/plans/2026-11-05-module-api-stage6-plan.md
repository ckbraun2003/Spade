# Module API Stage 6 Implementation Plan — the translation lock and the fixed-binding pattern

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A set that holds the built-in `lock` module can hold a body's translation on chosen world axes at an anchor, bitwise, on both backends, while its rotation stays free and its IMU reads `−Rᵀg` on the held axes. A set without it computes today's bits on both backends. The mechanism that makes the second half true on the GPU, a fixed binding that holds an empty buffer when its module is absent plus a recipe switch, is the pattern stage 7 reuses.

**Architecture:**
- **Option A, Integrate honours the lock** (spec §9; Physics' recommendation). The lock module declares one `per_body` array, `lock_rows` (anchor + axis mask). Integrate reads it as an optional read, computes today's arithmetic, then selects per locked axis. No new phase, no second writer of `pos`, `vel` or `specific_force`.
- **The typed front is `attach_row`.** `lock_translation(body, axes)` and `unlock(body)` are fronts over it, like `add_imu_sensor`: queued, applied at the next step boundary by the array's `init`, and refused with `unavailable` when the set has no lock module. No new queue op.
- **The fixed binding.** `lock_rows` is a hand-numbered binding in set 0 (29). A set without the module has no array to bind, so the state mirror binds a zero buffer of one row there, and the recorder switches the lock select off through a bit in `PassParams::switches`. The pattern is one small table (`compute/optional_bindings.hpp`), so a second user adds a row, not a mechanism.
- **Grades.** The lock is CPU `reference` with a new golden, Vulkan `banded` with `pos` and `vel` bit-exact (they are written, not integrated).

**Tech Stack:** C++23, Slang kernels (`integrate.slang`), GoogleTest through ctest, MSVC + Ninja presets, the gcc-13 Docker leg.

**Spec:** `2026-10-02-module-api-design.md` §9 (the lock, with its GPU paragraph), §3 (attached rows), §8 (grades), §13. Physics' requirements: `../../physics/plans/2026-10-02-module-api-requirements.md` §2. Outline: `2026-10-02-module-api-plan.md`, "Stage 6". What stage 7 takes from it: `2026-10-12-module-api-stage7-design.md` (T8, T11, T12) and `../2026-10-05-airflow-spec-core-answers.md` (items 3 and 25).

**Checked against:** master `58fbec4f` (stage 5 merged, `37ca30bf`). Every `file:line` below is from that tree and must be re-checked at the cut.

## Global Constraints

- **No existing golden moves and none is regenerated.** The standard identity stays `0x3303cfc86821f502`, the one-body schema hash `0x1f60a6fef22e254a`, snapshot format v3, and the 22-entry standard walk. Every GPU parity band holds unchanged (`TD-2`). One golden is **added** (Task 3), with its header recording what it covers (`TD-1`, `TD-12`).
- **A set without `lock` computes today's bits on both backends.** The CPU select is guarded by a non-zero mask, and the GPU select by the switch. The unlocked arithmetic is not restructured in `integrator.cpp` or `integrate.slang` (spec §9).
- **Wire contracts that do move, and only these:** `PassParams` 44 → 48 bytes (`switches` appended; `layout_check` is generated), set 0 gains binding 29 (`kBindingCount_state` 29 → 30), `layouts.slang` gains `LockRow`. Every SPIR-V variant is rebuilt; no fp32 expression in any kernel changes for an unlocked body.
- Refuse, never skip (`L6`): every refusal names the call, the module and the body. A refusal comes before any write.
- **Determinism:** set order, then declaration order; no unordered container; no allocation in a step; no libm, no wall-clock under `engine/` (`BitPortability*:M1B*`).
- **Layering:** `physics/lock.hpp` includes no `sim/` header; `compute/optional_bindings.hpp` is Vulkan-free.
- **These do not change:** `create()`'s signature, `spawn`, `despawn`, `attach_row`, `set_rotor_commands`, `module_rows`, the standard set's order and module versions (see decision 3).
- `-Wall -Wextra -Wpedantic` clean under gcc, warning-free under MSVC.
- **Where the code lives:** `../spade-wt/core-2`, on `core/module-api-stage6`, cut from master when the lead approves this plan. Commit only named paths. Never push, never stash.
- **The branch gate** (the tiered rule): per task, red first, then the focused sets; at the end of each group the full CPU `ctest -L spade -LE gpu` and `-L gpu`, each as one `jobrun` ticket under the lead's slot. Debug, the `-NoSeed` leg and smoke run at the batch gate. `gcc-check.sh --compile` on every changed `.cpp`, the plain check on changed headers, before READY.
- **Red first.** Every task commits its failing tests before the change; the red commit compiles (stubs, and for Task 4 the one binding declaration), so tests fail on behaviour. A test green at red is a pin and is named in its comment.

## Decisions this plan makes

1. **`lock_rows` is `per_body`, 16 bytes: `float anchor[3]; uint32_t axes`.** Bits 0, 1, 2 are world x, y, z. `axes == 0` is free, and a zero row is free, so a fresh or despawned body is unlocked with no init. The row type is `physics::LockRow` in `physics/lock.hpp`; a row is read as `world_rows<const LockRow>` by local body slot.
2. **The lock is a module with an array and no pass.** `modules::lock_module()` (in `standard_modules.cpp`, declared beside `standard_modules()`) is not in the standard set. Integrate gains one optional read, `"lock.lock_rows"`, appended as access index 5 (`kIntegratePassLockAccess`; the contact scratch stays at 3). An optional read of an absent module binds an absent view, and neither optional reads nor bindings are spelt into the identity (`module.hpp:684`).
3. **Integrate's module version does not move.** For a set without `lock` the pass is bit-identical, so the standard identity stays. A set with `lock` has its own identity, because the lock module's name and version are spelt in it (`L2`): a blob never restores across a lock choice, as for a role choice. *Alternative:* bump `integrate`'s version. It moves `0x3303cfc86821f502` and every saved blob, which is a frozen identity, so it would go to Cameron. Not recommended. (Open question 2.)
4. **The select, per locked axis `a`, after today's arithmetic.** Step 2: `felt[a] = −gravity[a]` (free axes keep `force_acc / m` plus the contact response), then `specific_force = conj(orient) * felt` as today. Step 3: `vel[a] = 0.0f` after the velocity update. Step 5: `pos[a] = anchor[a]` after the position update. Contact's position correction on a locked axis is therefore overwritten, so the anchor holds bitwise.
5. **The contact limit (spec §9) stays and is pinned.** The contact solve treats a locked body as free, with its finite mass, and the lock then discards that body's response. A test asserts today's behaviour for a body that hits a held one, so the limit is pinned, not discovered. Infinite mass in the solve is later work, with joints.
6. **`set_body_state` on a locked body is refused (`L6`), not silently overridden.** Integrate would snap the pose back on the next substep, so a re-pose along a held axis would be skipped. The call refuses with `invalid_argument` naming the body and its axes: unlock first. The census in `simulation.hpp` classifies `lock_rows` as KEPT. (Open question 5.)
7. **The optional binding is a table, not a special case.** `compute/optional_bindings.hpp`: `{name, switch bit, dummy bytes}`, one row today (`"lock_rows"`, `kSwitchLockRows = 1u`, 16 bytes). `Simulation::create()` sets `StepShape::switches` from it (a bit for every named array the schedule holds). The mirror finds the name's binding in its existing `binding_for` table, and binds a zeroed dummy where the set holds no such array. An unbound binding that is **not** in the table still fails the census (`DescriptorSetBindsEveryRegistryBinding`), so a forgotten standard binding is still caught.
8. **A grade for the lock.** CPU `reference`, Vulkan `banded`, with `pos` and `vel` bit-exact. It is Physics' table (`04-verification.md`, PHY-2), so Task 5 hands Physics the row to confirm (joint, as stage 5 Task 4 was).
9. **No kernel, switch or binding other than the lock's.** Stage 7's rows come with stage 7.

## What stage 6 unblocks, and the rework risk

Stage 6 is next in the agreed order (5, 6, 7); stage 5 merged `37ca30bf`.

**For stage 7** (`2026-10-12-module-api-stage7-design.md`), three tasks wait on this pattern:
- **T8, point samples:** a reader's kernel reads the per-point row behind a recipe switch, and switched off reads the per-world row and computes today's bits. That is `PassParams::switches` plus the empty binding.
- **T11, posed parts:** the part table's device mirror. The L10 mirror for a bound table exists (`static_contact.contact_sets`, binding 28), but a table whose module may be absent needs the empty binding.
- **T12, loads:** the per-rotor model row, read by `rotor.forces` on the GPU through the recipe switch. "Rotors and drag are unchanged without airflow, bit for bit" holds on the CPU only today, and on the GPU it needs exactly this (answers doc, item 25).

**What it does not need:** airflow's own grid arrays. B0 put them in a separate descriptor set 1 (`2026-10-21-airflow-gpu-b0-plan.md`, decision 1), so they never need set 0's empty bindings. The design's old premise, "stage 6's empty-buffer pattern in set 0", is narrower now: the three consumers above.

**The configuration tables' GPU mirror:** `07-status.md`'s line "waits for stage 6's fixed bindings" is half stale. Bound tables are mirrored since L10. What waits is a table in an optional module, which is T11. Task 6 corrects the line.

**The rework risk.** The 2026-10-12 inputs note priced the order "7 before 6" at about half a day of rework: stage 6 adapting to a pattern stage 7 had built (`2026-10-12-airflow-demo-core-inputs.md` §3). With the order held, the risk inverts: **stage 7 adapting to stage 6's pattern if it is too lock-shaped.** Sizing, as an estimate:
- If stage 6 hard-codes the lock (a `switches` bit and a dummy buffer inline in the mirror), stage 7 pays the same **about half a day** to generalise it before T8.
- With decision 7 (a one-row table keyed by array name, a 32-bit switch word), stage 7 adds a row per consumer. **About 1 to 2 hours** each; T8, T11 and T12 are three rows. The one thing the table does not cover today is an optional *table* or *scratch* binding (T11's part table): its row resolves by the declared binding number rather than an array name, which is a few lines in the mirror and belongs to T11. The cost of building it generic now is about 1 to 2 hours inside Task 4 and no new mechanism.
- Net: the order holding costs stage 7 about 0.5 day at most and about 0.25 day as planned, and nothing is reworked in stage 6.

**The consumer outside Core:** Interface's drone box drops its two behaviors for `lock_translation` and lifts its Vulkan refusal (`../../interface/07-status.md` row, after this merges). Their call sequence is in Task 6.

## Review Focus

Failure modes the spec implies that no single task's tests would otherwise exercise:
1. **A lock call on a body spawned this boundary** (not flushed): refused, as `set_body_state` refuses it. Task 1.
2. **Despawn then respawn into the same slot:** the new body is free. Task 1.
3. **A held body hit from below by a free one,** and a held body resting on the ground: the limit is pinned; the ground's contact on a locked axis is overwritten. Task 2.
4. **`−0.0` and `+0.0`:** a locked axis velocity is `+0.0f`, never `−0.0f` from a product. Task 2 compares bytes.
5. **A snapshot between the lock call and the next flush:** `snapshot()` already refuses a non-empty queue; restore of a locked state resumes holding. Task 1.
6. **The GPU with the lock module present but nothing locked** (all rows zero): bit-equal to the same set with the switch off. Task 5.
7. **A Vulkan set without the lock module:** the dummy buffer is bound and the debug leg's validation layer is silent. Task 4.

## Task 1: The lock module, its rows and the typed calls

**Files:**
- Create: `engine/physics/lock.hpp` (`LockRow`, `kLockAxisX/Y/Z`, `kLockAllAxes`, `LockSpawn`, `lock_rows_init`, `lock_rows_validate`).
- Modify: `engine/sim/standard_modules.cpp` and `.hpp` (`lock_module()`, with grade `{reference, banded}`; not added to `standard_modules()`).
- Modify: `engine/sim/simulation.hpp`, `simulation.cpp` (`lock_translation`, `unlock`, the `set_body_state` refusal, the census line).
- Test: new `tests/test_translation_lock.cpp` (registered in `tests/CMakeLists.txt`; named in READY).

**Interfaces:**
```cpp
// physics/lock.hpp, namespace spade::physics
inline constexpr uint32_t kLockAxisX = 1u, kLockAxisY = 2u, kLockAxisZ = 4u, kLockAllAxes = 7u;
struct LockRow { glm::vec3 anchor{0.0f}; uint32_t axes = 0; };  // 16 bytes; axes == 0 is free
static_assert(sizeof(LockRow) == 16 && std::is_trivially_copyable_v<LockRow>);
struct LockSpawn { float anchor[3]; uint32_t axes; };           // the attach_row record
// sim/standard_modules.hpp, namespace spade::modules
[[nodiscard]] ModuleDesc lock_module();    // array "lock_rows", per_body, init + validate, no pass
// Simulation (sim/simulation.hpp)
[[nodiscard]] Result<void> lock_translation(BodyRef body, uint32_t axes);  // axes in 1..7
[[nodiscard]] Result<void> unlock(BodyRef body);
```

Behaviour, in order, each a refusal before any write:
- `unavailable`: the set holds no `lock` module (`CompiledSchedule::modules`), naming the call and the module;
- `not_found`: a dead, stale or foreign ref, with `body()`'s messages, and a body spawned but not yet flushed (`set_body_state`'s rule);
- `invalid_argument`: `axes` outside 1..7 (`lock_translation`), or a poisoned Simulation, with `step()`'s text.

Then `attach_row(body, "lock_rows", LockSpawn{anchor = body's pos now, axes})`. Its `init` writes the row field-wise at the next boundary. `unlock` queues `axes = 0` with a zero anchor.

- [ ] **Step 1: Write the failing tests** (red stubs: `lock_module()` returns a module with the array and an `init` that writes nothing; `lock_translation` and `unlock` return `unavailable`).

```cpp
TEST(TranslationLock, ACallIsRefusedWhenTheSetHasNoLockModule) {            // green at red by stub: a pin
    auto sim = Simulation::create(one_body_world(), 2'000'000, 2);
    auto ref = spawn_flushed(*sim);
    const auto r = sim->lock_translation(ref, kLockAllAxes);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, spade::Code::unavailable);
    EXPECT_NE(r.error().context.find("lock"), std::string::npos) << r.error().context;
}
TEST(TranslationLock, ALockWritesItsRowAtTheNextBoundaryAndUnlockClearsIt) {
    auto sim = Simulation::create(one_body_world(), 2'000'000, 2, {}, standard_plus_lock());
    auto ref = spawn_flushed(*sim);
    ASSERT_TRUE(sim->lock_translation(ref, kLockAxisX | kLockAxisZ).has_value());
    ASSERT_TRUE(sim->flush_structural().has_value());
    const auto rows = sim->module_rows<LockRow>("lock_rows", 0);
    ASSERT_TRUE(rows.has_value());
    EXPECT_EQ((*rows)[0].axes, kLockAxisX | kLockAxisZ);
    EXPECT_EQ(bits((*rows)[0].anchor), bits(spawn_pos)) << "the anchor is the body's position at the call";
    ASSERT_TRUE(sim->unlock(ref).has_value());
    ASSERT_TRUE(sim->flush_structural().has_value());
    EXPECT_EQ((*sim->module_rows<LockRow>("lock_rows", 0))[0].axes, 0u);
}
```

  More tests, each named for its failure: `ARefusalNamesTheCallAndWritesNothing` (axes 0, axes 8, a stale ref, a spawned-not-flushed ref); `ADespawnFreesTheRowAndARespawnIsFree` (Review Focus 2); `TheStandardSetStillRefusesTheCallsAndKeepsItsIdentity` (identity `0x3303cfc86821f502`, walk 22); `ASetWithLockHasItsOwnIdentityAndWalk` (identity differs, `lock_rows` is in the walk); `ALockedStateSurvivesASnapshotAndRestore` (snapshot after a flush, restore into a fresh lock set, same digest after N more steps; Task 2 adds the hold); `RestoreIntoASetWithoutLockIsRefused` (identity message, as for a role choice); `SetBodyStateRefusesALockedBody` (decision 6) and `SetBodyStateOnAnUnlockedBodyInALockSetIsUnchanged`. `SetBodyState.CensusClassifiesEveryPerBodyArrayAndTheSetHonoursIt` covers the standard set only, so one census case for a lock set is added (`lock_rows` classified KEPT), beside the `simulation.hpp` census line.

- [ ] **Step 2: Run, confirm each fails for its stated reason** (`ctest -R TranslationLock|SetBodyState`). Keep the failing output.
- [ ] **Step 3: Implement** `lock.hpp`, `lock_module()`, the two calls and the `set_body_state` refusal.
- [ ] **Step 4: Focused tests green; gcc-check** `sim/simulation.cpp`, `sim/standard_modules.cpp`, `tests/test_translation_lock.cpp`.
- [ ] **Step 5: Commit** red, then green (two commits).

## Task 2: Integrate honours the lock on the CPU

**Files:**
- Modify: `engine/physics/integrator.hpp`, `integrator.cpp` (`integrate_bodies(..., std::span<const LockRow> locks = {})`).
- Modify: `engine/physics/schedule.cpp` (`pass_integrate` reads `ctx.state[kIntegratePassLockAccess]`), `engine/physics/contact_dv.hpp` or `lock.hpp` (the index constant), `engine/sim/standard_modules.cpp` (the sixth access, optional).
- Test: `tests/test_translation_lock.cpp`.

**Physics-owned files:** `integrator.cpp` is Physics'. The diff is guarded insertions after steps 2, 3 and 5; Physics reviews that the unlocked path is untouched, as in stage 5 Task 4 (open question 3).

```cpp
// integrate_bodies, inside the loop, after today's steps (sketch)
const uint32_t axes = locks.empty() ? 0u : (locks[i].axes & kLockAllAxes);
// step 2: after `felt` is computed, before specific_force is written
if (axes != 0u) for (int a = 0; a < 3; ++a) if (axes & (1u << a)) felt[a] = -gravity[a];
// step 3: after body.vel += (accel_ext + gravity) * h;
if (axes != 0u) for (int a = 0; a < 3; ++a) if (axes & (1u << a)) body.vel[a] = 0.0f;
// step 5: after body.pos += body.vel * h;
if (axes != 0u) for (int a = 0; a < 3; ++a) if (axes & (1u << a)) body.pos[a] = locks[i].anchor[a];
```

- [ ] **Step 1: Write the failing tests** (the red commit compiles: the parameter exists and is ignored).

```cpp
TEST(TranslationLock, AFullLockHoldsPositionAndVelocityBitwise) {
    // a mass that is NOT a power of two: the old behavior-pair hold cancels only for powers of two
    auto sim = lock_sim({.mass = 1.37f});
    auto ref = spawn_flushed(*sim, /*pos*/ {0.0f, 3.0f, 0.0f}, /*vel*/ {0.5f, -1.0f, 0.25f});
    ASSERT_TRUE(sim->lock_translation(ref, kLockAllAxes).has_value());
    for (int s = 0; s < 200; ++s) {
        ASSERT_TRUE(sim->step(1).has_value());
        const BodyState& b = **sim->body(ref);
        EXPECT_EQ(bits(b.pos), bits(anchor)) << "step " << s;
        EXPECT_EQ(bits(b.vel), bits(glm::vec3(0.0f))) << "step " << s << ": +0.0f on every axis";
    }
}
TEST(TranslationLock, AHeldBodysImuReadsMinusRTgFromAnyMass) {
    // an attitude that is not axis-aligned; the reference is the same glm expression, so exact
    ... EXPECT_EQ(bits(b.specific_force), bits(glm::conjugate(b.orient_before) * (-gravity)));
}
TEST(TranslationLock, TheYFreeRailClimbsLikeAFreeBodyAndHoldsXAndZ)   // axes x|z; y bit-equal to an unlocked twin in world 1
TEST(TranslationLock, RotationStaysFree)       // torque on both; omega_body and orient bit-equal to the unlocked twin
TEST(TranslationLock, ContactAgainstALockedBodyIsTodaysBehavior)       // decision 5: the free body's response at the contact tick
                                                                       // equals the same run with the other body unlocked
TEST(TranslationLock, AHeldBodyOnTheGroundIsOverwrittenOnItsLockedAxes) // contact's pos correction on a locked axis never survives a substep
TEST(TranslationLock, AnUnlockedBodyInALockSetIsBitIdenticalToTheStandardSet)  // same scenario, standard vs lock set: digests equal
TEST(TranslationLock, UnlockReleasesFromTheAnchorAtRest)               // free fall from the anchor with vel 0
```

- [ ] **Step 2: Run; confirm each fails on behaviour** (position drifts, vel is not 0). `AnUnlockedBody...` is green at red: a pin.
- [ ] **Step 3: Implement** the three guarded insertions and the sixth access.
- [ ] **Step 4: Focused green:** `TranslationLock|Integrator|Physics|ModuleSchedule|StandardModules|Determinism|GoldenCorpus` (the corpus is the proof that no existing digest moved); gcc-check `physics/integrator.cpp`, `physics/schedule.cpp`, `sim/standard_modules.cpp`.
- [ ] **Step 5: Commit** red, then green.

## Task 3: The held-quad golden (CPU reference)

**Files:** `tests/test_translation_lock.cpp`. No scenario-file or loader change (open question 1).

A held quadrotor (the existing quad model and its trim helper, as `quad_hover`'s) flying a collective-and-attitude script under a full lock: 450 steps of 2 ms at 2 substeps, rotor commands changed at fixed ticks and a body torque pulse so the attitude moves. Asserted, with the digest as the golden:
- `pos` is bitwise the anchor at every step and `vel` is `+0.0f`;
- `specific_force` is bit-equal to `conj(orient) * (−g)` at every step, whatever the rotors do (the IMU reads +1 g up);
- `state_digest()` at the end equals `kHeldQuadDigest`, a literal;
- a mid-run snapshot at tick 225 restored into a fresh lock set lands on the same digest (the `quad_hover` test's pattern).

- [ ] **Step 1: Write the test with `kHeldQuadDigest = 0` and the header comment** (what it covers, and that it is added, never regenerated). Red: the digest differs; the bitwise asserts already pass after Task 2, so this task's red is the digest alone, which is the nature of a new golden.
- [ ] **Step 2: Run, read the printed digest, check it twice** (two runs, both presets); pin it. Commit once the number is stable on both presets, with the run counts in the message.
- [ ] **Step 3: Commit.**

## Task 4: The fixed binding and the recipe switch (plumbing; no kernel semantic change)

**Files:**
- Create: `engine/compute/optional_bindings.hpp` (`OptionalBinding`, `kOptionalBindings`, `kSwitchLockRows`).
- Modify: `engine/shaders/shared/layouts.slang` (`LockRow`, `PassParams::switches`), `engine/shaders/shared/bindings.slang` (`StructuredBuffer<LockRow> lock_rows` at `(29, 0)`, a new section G with the pattern's rationale), `engine/compute/backend.hpp` (`StepShape::switches`), `engine/compute/vulkan/step_recorder.hpp/.cpp` (`PassParams::switches` from the shape), `engine/compute/vulkan/state_mirror.cpp` (`binding_for` entry; the dummy), `engine/sim/simulation.cpp` (the switch bits).
- Test: `tests/test_gpu_state_mirror.cpp`, `tests/test_bindings_airflow.cpp` (29 → 30, with the comment), `tests/test_translation_lock.cpp` (a CPU-side `switches` test).

The red commit holds the one declaration (`bindings.slang`'s binding and `LockRow`) so `gen::kBinding_lock_rows` exists; nothing reads it.

- [ ] **Step 1: Write the failing tests.**

```cpp
TEST(OptionalBindings, ASetWithoutTheLockModuleHasNoSwitch_AndASetWithItHasOne) {     // CPU-side, runs in the leg
    EXPECT_EQ(step_shape_of(standard_schedule()).switches, 0u);
    EXPECT_EQ(step_shape_of(standard_plus_lock_schedule()).switches, spade::compute::kSwitchLockRows);
}
TEST_F(GpuStateMirrorTest, ASetWithoutTheLockModuleBindsAnEmptyBufferAtItsBinding) {
    // shape.arrays = walk_shapes(standard_schedule(), shape): no lock_rows entry
    ... EXPECT_EQ(backend->bound_binding_count(), spade::compute::gen::kBindingCount_state);  // 30, not 29
}
TEST_F(GpuStateMirrorTest, ALockSetBindsItsRealArrayAtTheSameBindingAndRoundTripsIt)
TEST(OptionalBindings, EveryOptionalNameHasAMirrorBinding)  // the table and binding_for cannot drift
```

  Plus `GpuStateMirrorTest.DescriptorSetBindsEveryRegistryBinding` (existing) must still pass with 30 and still fail for a standard binding that is not in the table.
- [ ] **Step 2: Run; confirm failure on behaviour** (bound count 29 ≠ 30, `switches` always 0).
- [ ] **Step 3: Implement** the table, the shape field, the recorder's push constant, the mirror's dummy and `binding_for` entry.
- [ ] **Step 4: Focused green:** `OptionalBindings|GpuStateMirror|BindingsAirflow|GpuModuleSchedule|SlangLayouts|LayoutCheck`, then the whole `-L gpu` tier (every kernel is rebuilt against a 30-binding set). **Measure that the standard set's device state is unchanged:** `GpuInvariance*` and `GpuParity*` unchanged, and one same-device digest of the standard set's state after N Vulkan steps before and after this task (printed by a one-off, not committed), bit-equal.
- [ ] **Step 5: gcc-check** the changed `.cpp` and headers (the Vulkan files compile in the leg build; `bindings.gen.hpp` exists there). **Commit** red, then green.

## Task 5: Integrate honours the lock on Vulkan; the lock's grade

**Files:**
- Modify: `engine/shaders/kernels/integrate.slang` (a guarded select, mirroring `integrator.cpp`'s, behind `(pass_params.switches & kSwitchLockRows) != 0u`; the header cites the CPU lines).
- Modify: `engine/sim/standard_modules.cpp` (confirm the lock's grade).
- Test: `tests/test_translation_lock.cpp` (GPU cases, skipped with `vulkan_skip_reason()`), `tests/test_gpu_parity.cpp` only if a helper is shared.

- [ ] **Step 1: Write the failing tests.**

```cpp
TEST(TranslationLockVulkan, PositionAndVelocityAreBitExactAgainstTheCpuUnderAFullLock)   // pos and vel: bytes equal to the CPU run, every step
TEST(TranslationLockVulkan, TheYFreeRailHoldsXAndZBitExactAndYIsBandedAsIntegrateIs)
TEST(TranslationLockVulkan, ASetWithLockButNothingLockedIsBitEqualToTheSwitchOff)        // Review Focus 6
TEST(TranslationLockVulkan, TheHeldQuadsImuIsBandedAgainstTheCpu)                        // the golden's script on Vulkan; specific_force and orient in TD-14's band
TEST(TranslationLock, ALockSetsGradeIsReferenceOnTheCpuAndBandedOnVulkan)                // CPU-side, no device: set_grade() of the compiled lock set on both backends
```

- [ ] **Step 2: Run on the admitted device; confirm failure on behaviour** (the GPU body drifts under the lock). On a box with no device the GPU cases skip and the CPU-side one runs.
- [ ] **Step 3: Implement** the kernel select.
- [ ] **Step 4: Focused green, then the full `-L gpu` tier,** plus `TranslationLock*` on both presets. The debug leg's validation layer is the check that the dummy and the switch are valid (batch gate).
- [ ] **Step 5: gcc-check; commit** red, then green. Send Physics the lock's grade row for `04-verification.md`.

## Task 6: Prose follows the code

**Files:** `engine/sim/module.hpp` (header comment), `docs/design/core/01-modules-and-scheduler.md`, `02-state-and-snapshot.md`, `07-status.md` (the lock row, the stage 6 bullet, and the stale "tables' GPU mirror waits for stage 6" line, now "a table in an optional module waits for the pattern: stage 7's T11"), `docs/design/backlog.md` (the lock item), `CHANGELOG.md`, and `docs/design/consumers.md` (Lead's). The stage 7 design's "Before it" lines name stage 6 as merged.

The consumers line for Kat and Interface: `lock_module()` and `Simulation::lock_translation / unlock`; `physics::LockRow`; `StepShape::switches` and `PassParams` 48 bytes (internal); binding 29 and `kBindingCount_state` 30; a set without `lock` is unchanged on both backends; `set_body_state` refuses a locked body. **Interface's change:** add `lock_module()` to the drone box's set, call `lock_translation(ref, kLockAllAxes)` after the spawn's flush, drop the two CPU behaviors, lift the Vulkan refusal.

- [ ] **Step 1: Edit.** **Step 2:** `AnalysisValidation*` (records `.md` changed); no code. **Step 3: Commit.**

## Goldens and identities

| | Moves? |
|---|---|
| Every corpus golden (`ballistic`, `bounce`, `gnss_tumble`, `quad_hover`, `shower`, `two_world_isolation`) | No |
| Standard identity `0x3303cfc86821f502`, schema hash `0x1f60a6fef22e254a`, snapshot v3, the 22-entry walk | No |
| Every GPU parity band; the standard chain's dispatch count | No (the switch is off) |
| `PassParams` (44 → 48 bytes), set 0 bindings (29 → 30), `layouts.slang` | Yes: wire contracts, generated checks |
| The held-quad golden `kHeldQuadDigest` | **Added**, never regenerated |
| A lock set's identity and schema hash | New, per set; no standard blob is affected |

Nothing here changes a law or a frozen identity, so nothing goes to Cameron, **unless** open question 2 is answered "bump".

## gcc-check list

`.cpp` (`--compile`): `engine/physics/integrator.cpp`, `engine/physics/schedule.cpp`, `engine/sim/standard_modules.cpp`, `engine/sim/simulation.cpp`, `engine/compute/vulkan/step_recorder.cpp`, `engine/compute/vulkan/state_mirror.cpp`, `tests/test_translation_lock.cpp`, `tests/test_gpu_state_mirror.cpp`, `tests/test_bindings_airflow.cpp`.
Headers (plain): `engine/physics/lock.hpp`, `integrator.hpp`, `engine/compute/optional_bindings.hpp`, `engine/compute/backend.hpp`, `engine/compute/vulkan/step_recorder.hpp`, `engine/sim/simulation.hpp`, `engine/sim/standard_modules.hpp`.

## Open questions for the lead

1. **The held-quad golden: test-local or a corpus scenario?** *Recommendation:* test-local, a literal digest in `test_translation_lock.cpp` (Task 3). The corpus schema v1 has no module-set key and no lock input, so a scenario would extend the format (an additive key and a Test/Docs loader change, and the `expected` list at `test_determinism.cpp:985`). YAGNI until a second held scenario exists.
2. **Integrate's version.** *Recommendation:* no bump (decision 3). A bump moves the standard identity, a frozen identity, and goes to Cameron.
3. **Physics' files.** `integrator.cpp` and `integrate.slang` get guarded insertions, and the lock's grade row is Physics' table. *Recommendation:* Core edits, Physics reviews the diff, as stage 5 Task 4.
4. **`lock_translation` as a front over `attach_row`** (queued, applied at the next boundary), against a direct write like `set_body_state`. *Recommendation:* `attach_row`; it is the spec's "queued", needs no new op, and `snapshot()` already refuses a pending queue.
5. **`set_body_state` on a locked body: refuse** (decision 6) against keep-and-snap. *Recommendation:* refuse (`L6`); about fifteen lines and two tests.
6. **A set that holds `lock` but not `integrate`** can hold nothing, silently. No check is added (YAGNI): the standard set always has Integrate. Say if you want one.

## Notes for later stages

- **Stage 7** adds its rows to `kOptionalBindings` (T8's per-point row, T12's per-rotor model row) and resolves T11's part table by its declared binding number. Its `BodyState::flags` bit for two-way coupling is a separate mechanism (spec §11).
- **A locked body as infinite mass in the contact solve** is joints' work, not this stage's.
- **Estimate:** six tasks, about 9 working hours *(estimate)*, which matches the 2026-10-05 sizing of 1 to 1.5 days for this stage. Review and merge queues add calendar time of their own.
