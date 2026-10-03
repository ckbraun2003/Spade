# Drone builder, Core's part — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the scene composer and Kat's compiler what they need from Core:
- a public pose-to-transform function;
- model types that carry their version and design frame;
- a spawn and a read that speak the design frame;
- snapshots that refuse another model registry;
- the scene-file schema.

**Architecture:** Four tasks, each independently mergeable, in the order the composer needs them:
- **A** unblocks `compose()`.
- **B** is one contract: spawn and the vehicle-state read are inverses, so they land together.
- **C** pins the registry that B changes.
- **D** is the schema the composer reads.

Module-API stage 4 (modules own their state) follows this plan and gets its own plan. Nothing here waits on it.

**Tech Stack:** C++23, glm, yaml-cpp (already linked privately by `spade_world`), GoogleTest.

**Spec:**
- The joint drone-builder spec, approved by the user 2026-10-03: `../../plans/2026-10-03-drone-builder-engine-design.md` (`DBE-*`), `../../physics/plans/2026-10-03-drone-builder-physics.md` (`DBP-44..47`: the design frame);
- the scene-file schema (`../../interface/plans/2026-10-03-scene-file-draft.md`, `SCN-001..009`, Core's, approved 4294f9b);
- the composer plan (`../../interface/plans/2026-10-03-scene-composer-plan.md`, approved 0718594, corrected 2026-10-03: spawn, not the composer, converts the design frame).

## Global Constraints

- Every golden digest and every GPU band stays unchanged. Every existing model has the identity design rotation and a zero centre-of-mass offset, and spawn short-circuits that case, so its rows are bit-identical.
- `DBP-45`: the design-to-body rotation and the centre-of-mass offset appear only in the model type, spawn and the vehicle-state read.
- The world file's canonical text is unchanged byte for byte (`WorldFile.MaximalWorldRoundTripsBitExactly`, `EmissionIsCanonical`).
- gcc-13 `-Wall -Wextra -Wpedantic -Werror` clean: every new aggregate member gets a default member initializer, and the Docker leg runs on each task's head before "ready for review".
- Builds and tests run only in a slot the lead hands out, in the foreground, at `-j1`, chunked. Code lives in `../spade-wt/core`, on one branch per task (`core/transform-of`, `core/design-frame`, `core/snapshot-v3`, `core/scene-schema`). Commit only named paths. Never push.

## Decisions this plan makes

- **`transform_of` returns `SdfTransform{}` for the identity pose.**
  - The analytic path would store the identity's translation as `-0`.
  - The builder has always short-circuited it to transform 0, which is `SdfTransform{}` (+0). `config_hash` folds those bytes, and `maximal.world.yaml` spells them.
  - So the public function keeps the builder's `==` identity test, its check order (non-finite, then scale, then degenerate quaternion) and its arithmetic (divide by the length, `mat3_cast`, transpose, divide by the scale, negate `m·position`).
  - `WorldBuilder::add_transform` keeps returning index 0 for the identity and calls `transform_of` otherwise.
- **ModelType gains three fields:**
  - `version` (`uint32_t`, at least 1);
  - `design_to_principal` (`glm::quat`, the `q_bd` of `DBP-44`: it maps design-frame vectors to body-frame vectors);
  - `com_offset` (`glm::vec3`, the centre of mass from the design origin, in the design frame, metres).
  - `validate()` requires a finite, non-zero-length rotation and a finite offset.
  - `register_model` normalizes the rotation and canonicalizes it to w ≥ 0 before storing, so `q` and `-q` register the same model. That normalization is not on the step path.
  - The defaults are 1, the identity and zero, so every model built today is unchanged.
- **Spawn takes the design frame (`DBP-46`); `vehicle_state` returns it (`DBP-47`).**
  - The rigid-body identities are `DBP-44`'s. With `R_wd` the design-to-world rotation:
    - `q_wb = q_wd ⊗ conj(q_bd)`;
    - `p_com = p_d + R_wd·c`;
    - `ω_b = R_bd·ω_d`;
    - `v_com = v_d + ω_w × (R_wd·c)`, with `ω_w = R_wd·ω_d`.
  - The read applies the inverses.
  - Both sit in one pair of functions in `sim/design_frame.hpp` (`to_body_state`, `to_design_state`), so `spawn` and the read cannot disagree.
  - When `q_bd` is exactly the identity and `c` is exactly zero, both return their input untouched, which is the bit-identity the goldens need.
  - Bare bodies (`spawn(world, BodySpawn)`) have no design frame and are unchanged.
- **Snapshot format v3: a second identity, the model registry's.**
  - The header grows to 48 bytes: `model_registry_identity` (u64) at offset 40.
  - The identity is an FNV-1a 64 fold over every registered model in registration order, spelt byte by byte: name, version, `param_schema_id`, then every field that reaches state or the design frame, as little-endian bit patterns. `visual_ref` is excluded, because it is render data (`L5`).
  - `Simulation::restore` refuses a mismatch with `invalid_argument` naming "model registry", after the module-set check.
  - A v2 blob is refused with the version message, and `consumers.md` gets a line.
- **The world file's YAML helpers move to `world/yaml_text.{hpp,cpp}`.** The scene file is the third parser, which is the moment `scenario_file.hpp` itself names for the promotion. They are:
  - the writer side: `dec`, `format_float`, `float_list`, `quote_yaml`;
  - the reader side: `check_map`, `field`, `parse_float`, `parse_uint`, `parse_floats`.
  The world file and the scenario reader call them. Their output is byte-identical, and the world file's round-trip tests are the gate.
- **The scene file's world hash** is `fnv1a64(world_to_yaml(load_world_file(path)))`, written as a quoted `"0x"` plus 16 lowercase hex digits (`SCN-001`).

## Review Focus

1. **A posed primitive's transform bytes** are the same through `WorldBuilder` and through `transform_of`. The identity pose gives `SdfTransform{}` exactly, not `-0`. Pinned in Task A.
2. **Every existing model spawns bit-identically.** The identity rotation with a zero offset short-circuits. Pinned in Task B, and gated by `quad_hover` and the viewer goldens.
3. **Spawn, then read, returns the design-frame start** for a model with a real rotation and offset, within a stated fp32 tolerance. A model whose rotation is `-q` registers identically to one with `q`. Pinned in Task B.
4. **A snapshot restored into a Simulation with another model registry is refused,** naming "model registry". The registration order is part of the identity, `visual_ref` is not, and a v2 blob is refused with the version message. Pinned in Task C.
5. **A scene file round-trips byte for byte, and its world hash refuses a different world.** Unknown, missing and duplicate keys are refused at every level. Pinned in Task D.

---

### Task A: `transform_of` is public

**Files:**
- Modify: `engine/world/builder.hpp` (declare it), `engine/world/builder.cpp` (move the body of `add_transform`'s maths into it; `add_transform` calls it).
- Test: `tests/test_sdf.cpp`.

**Interfaces:**
- Produces `[[nodiscard]] Result<SdfTransform> transform_of(const SdfPose& pose);` in namespace `spade`. It returns `invalid_argument` with the builder's own three messages.

- [ ] **Step 1: Write the failing tests.**

```cpp
TEST(TransformOf, TheIdentityPoseIsTransformZeroExactly) {
    const auto t = spade::transform_of(spade::SdfPose{});
    ASSERT_TRUE(t.has_value()) << t.error().context;
    const spade::SdfTransform zero{};
    EXPECT_EQ(std::memcmp(&*t, &zero, sizeof(zero)), 0) << "the identity's translation must be +0, not -0";
}

TEST(TransformOf, MatchesTheBuildersBytesForAPosedPrimitive) {
    const spade::SdfPose pose{.position = {1.5f, -2.0f, 0.25f},
                              .rotation = glm::normalize(glm::quat(0.9f, 0.1f, -0.3f, 0.2f)),
                              .scale = 1.75f};
    const auto world =
        spade::WorldBuilder().name("t").capacities(spade::Capacities{1, 1, 1, 1}).sphere(0.5f, pose).build();
    ASSERT_TRUE(world.has_value()) << world.error().context;
    const auto t = spade::transform_of(pose);
    ASSERT_TRUE(t.has_value());
    const spade::SdfTransform& built = world->sdf.transforms.back();
    EXPECT_EQ(std::memcmp(&*t, &built, sizeof(built)), 0);
}

TEST(TransformOf, RefusesWhatTheBuilderRefuses) {
    EXPECT_EQ(spade::transform_of({.scale = 0.0f}).error().code, spade::Code::invalid_argument);
    EXPECT_EQ(spade::transform_of({.rotation = glm::quat(0.0f, 0.0f, 0.0f, 0.0f)}).error().code, spade::Code::invalid_argument);
    EXPECT_EQ(spade::transform_of({.position = {std::numeric_limits<float>::infinity(), 0.0f, 0.0f}}).error().code,
              spade::Code::invalid_argument);
}
```
- [ ] **Step 2:** In a slot, build. Expected: fails to compile, because `transform_of` is undeclared.
- [ ] **Step 3: Implement.** Move the checks and maths, verbatim and in the same order, into `transform_of`. `add_transform` keeps its identity pre-check (it returns 0 and pushes nothing), calls `transform_of` otherwise, and forwards its error through `fail()`.
- [ ] **Step 4:** Build. Run `-Filter "TransformOf|WorldBuilder|WorldDescProduct|WorldFile|SdfProgram"`, then the full suite. Expected: all pass, and every world-file round trip and golden is unchanged.
- [ ] **Step 5: Commit** (test, then implementation): `feat(world): transform_of(SdfPose) is public; WorldBuilder poses through it`. Send Interface the hash: `compose()` can start.

### Task B: model types carry their design frame; spawn and `vehicle_state` speak it

**Files:**
- Modify `engine/vehicles/model_type.{hpp,cpp}`: the three fields and their validation. Physics reviews this, since vehicles/ is their layer.
- Create `engine/sim/design_frame.hpp` and `design_frame.cpp`: `DesignState { glm::vec3 pos; glm::quat orient; glm::vec3 vel; glm::vec3 omega; }`, plus `to_body_state` and `to_design_state`.
- Modify `engine/sim/simulation.{hpp,cpp}`:
  - `register_model` canonicalizes the rotation;
  - `spawn(world, ModelTypeId, VehicleSpawn)` converts through `to_body_state`;
  - add `Result<DesignState> vehicle_state(const VehicleRef&) const`.
- Tests: `tests/test_model_type.cpp` and `tests/test_sim_vehicle.cpp` (or the file that holds the `spawn(world, model, ...)` cases today).

**Interfaces:**
- Produces:
  - the `ModelType` fields `version`, `design_to_principal` and `com_offset`;
  - `sim::DesignState`, `to_body_state(const DesignState&, glm::quat q_bd, glm::vec3 c)` and `to_design_state(...)`;
  - `Simulation::vehicle_state`.

- [ ] **Step 1: Write the failing tests.**
  - `ModelTypeValidation.TheDesignFrameMustBeFiniteAndOrientable`: a non-finite offset and a zero-length rotation are each refused.
  - `DesignFrame.TheIdentityFrameLeavesTheStateBitwise`: `to_body_state` with the identity and zero returns its input bit for bit, and so does `to_design_state`.
  - `DesignFrame.SpawnThenReadReturnsTheDesignStart`:
    - Use a quadrotor model with `design_to_principal = normalize(quat(0.96, 0.10, 0.20, -0.15))` and `com_offset = (0.01, -0.02, 0.005)`.
    - Spawn at a design pose with a velocity and rates, read `vehicle_state` at tick 0 (before any step), and compare.
    - Each component must agree to within `1e-6 · max(1, |x|)`.
  - `DesignFrame.QAndMinusQRegisterTheSameModel`: two models differing only in the sign of the rotation spawn bit-identical rows.
  - `DesignFrame.ADefaultFrameSpawnsTheStartBitwise`: a quadrotor with the default design frame, spawned with a unit `orient`, a velocity and rates, has a body row whose `pos`, `orient`, `vel` and `omega_body` hold exactly the start's bits. That is the short-circuit, observed at spawn. The `quad_hover` and viewer goldens gate everything else.
- [ ] **Step 2:** Build in a slot. Expected: fails to compile.
- [ ] **Step 3: Implement.**
  - The conversion uses only `glm` quaternion and vector operations, in the order written in the plan's decision above. It sits in `spade_sim`, which builds under `spade_fp_strict`.
  - `vehicle_state` reads the body row through the vehicle's `BodyRef` and the model through `VehicleRef::model`. It refuses a stale ref with the same code that `body()` uses.
- [ ] **Step 4:** Build. Run the focused tests, then the full suite. Expected: every golden and band is unchanged.
- [ ] **Step 5: Commit**: `feat(core): model types carry their design frame; spawn and vehicle_state speak it (DBP-45..47)`.

### Task C: snapshot format v3 — the model registry's identity

**Files:**
- Modify `engine/state/snapshot.{hpp,cpp}`:
  - `kSnapshotVersion = 3`;
  - the header is 48 bytes, with `model_registry_identity` at 40 and the static_asserts updated;
  - `save()` gains the parameter, defaulting to 0;
  - add the accessor.
- Modify `engine/sim/simulation.cpp`:
  - `snapshot()` folds the registry;
  - `restore()` refuses a mismatch after the module-set check.
- Add `engine/sim/model_identity.{hpp,cpp}`: `uint64_t model_registry_identity(std::span<const vehicles::ModelType>)`, with the byte spelling written in its header comment.
- Update the comments the exploration found:
  - simulation.hpp's "still an unchecked caller obligation" and its layers-of-check list;
  - `register_model`'s doc;
  - `model_type.hpp:37-44`;
  - `CMakeLists.txt:15-16`, which says "currently 1".
- Tests: `tests/test_snapshot.cpp` and `tests/test_module_schedule.cpp`'s `ModuleSnapshot` cases.

- [ ] **Step 1: Write the failing tests.**
  - `SnapshotFormat.AVersionTwoBlobIsRefusedWithTheVersionMessage`: like the existing v1 case.
  - `SnapshotFormat.TheModelRegistryIdentityRoundTripsThroughTheHeader`.
  - `ModelSnapshot.RestoreIntoAnotherModelRegistryIsRefused`: the same models registered in another order are refused, and so is a model with another version. The error names "model registry".
  - `ModelSnapshot.AVisualRefDoesNotEnterTheIdentity`.
  - `ModelSnapshot.RestoreIntoTheSameRegistrySucceeds`.
- [ ] **Step 2:** Build in a slot. Expected: fails to compile.
- [ ] **Step 3: Implement.** In the identity fold, every float goes in as its `std::bit_cast<uint32_t>` little-endian bytes. Lists are preceded by their count. Strings are followed by a 0 byte.
- [ ] **Step 4:** Build. Run `-Filter "Snapshot|ModuleSnapshot|ModelSnapshot"`, then the full suite. Expected: all pass, with no golden moved (the identity is in no digest).
- [ ] **Step 5: Commit**: `feat(core): snapshots carry the model registry's identity; restore refuses another (format v3)`. Tell the lead: `consumers.md` needs the v3 line, because KAT's session ring cannot restore a v2 blob.

### Task D: the scene-file schema (`spade::scene`)

**Files:**
- Create `engine/world/yaml_text.{hpp,cpp}`: the promoted helpers. `world_file.cpp` and `testing/scenario_file.hpp` switch to them, and their copies are deleted.
- Create `engine/scene/scene_file.{hpp,cpp}`, in Interface's `spade_scene` target:
  - `SceneDesc` (the draft's sections: world reference, materials, models, assets, vehicles, spare);
  - `scene_from_yaml`, `scene_to_yaml` (canonical), `load_scene_file` and `world_hash(const WorldDesc&)`.
- Test: `tests/test_scene_file.cpp`.

**Interfaces:**
- Produces `scene::SceneDesc`, `scene_from_yaml(std::string_view)`, `scene_to_yaml(const SceneDesc&)`, `load_scene_file(const std::filesystem::path&)` and `world_hash(const WorldDesc&)`.
- Models are full `ModelType`s, Task B's fields included. Assets carry an `SdfPose`, a collider as an `SdfProgram` in the world file's encoding with node materials named, and a visual. Vehicles carry a model name and a design-frame `VehicleSpawn`.

- [ ] **Step 1: Write the failing tests.**
  - `SceneFile.AScenesTextRoundTripsByteForByte`: over a committed `tests/golden/scenes/gate_run.scene.yaml` that exercises every section.
  - `SceneFile.UnknownMissingAndDuplicateKeysAreRefusedAtEveryLevel`.
  - `SceneFile.TheWorldHashRefusesAnotherWorld`.
  - `SceneFile.NamesAreUniqueAcrossAssetsAndVehicles`.
  - `SceneFile.AMaterialNamedInTheWorldsPaletteIsRefused`.
  - `SceneFile.SpareIsAlwaysWritten`.
- [ ] **Step 2:** Build in a slot. Expected: fails to compile.
- [ ] **Step 3: Implement.** First promote the helpers, and confirm the world-file and scenario tests still pass at that commit. Then write the scene file on top of them.
- [ ] **Step 4:** Build. Run the focused tests, then the full suite.
- [ ] **Step 5: Commit**, in two commits:
  - `refactor(world): the YAML text helpers are shared (the scene file is the third parser)`;
  - `feat(scene): the scene-file schema -- SceneDesc, reader, canonical writer, world hash`.
  Send Interface the hash: `compose_file()` and `instantiate()` can start.

## After this plan

- **Module-API stage 4** (modules own their state, `CORE-4`) gets its own plan. Its outline is in the stage-1 plan, and its design-frame item has moved here (Task B).
- **The duty-command host call** (drone-builder physics) stays a stage-4 item, because it needs module-owned rotor state.
