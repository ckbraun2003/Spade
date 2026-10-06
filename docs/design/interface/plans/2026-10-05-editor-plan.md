# The editor, first cut — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Grow `spade_sandbox` into Spade's editor as the approved design specifies: scene and world documents with validated, undoable edits; a library placed by copy; an inspector over every parameter; worlds created, edited and saved; runs rebuilt from the documents.

**Architecture:** Every decision lives in display-free headers under `sandbox/` (`SL15b`), asserted by `spade_tests` with no window: a generic undoable `Document<Desc>`, edit variants applied by pure functions, atomic file I/O, a run controller over `compose()` and `instantiate()`. The window (`EditorSession`, `GlTargetSink` panels) only turns input into edits and draws the documents. A one-line text form of every edit drives the same functions from the command line (`EDT-002`).

**Tech stack:** C++23, MSVC `/W4 /WX` and gcc-13 `-Werror`, GoogleTest, ImGui 1.91.5-docking on GLFW 3.4, the installed `spade::` targets only (`SL2b`): `core`, `world`, `render`, `render_gl`, `sim`, `vehicles`, `scene`.

**Spec:** `2026-10-05-editor-design.md` (approved by the user on 2026-10-05, via the lead). Each task names the `EDT-` rows it implements.

**Status:** draft for the lead's review. No code before the lead approves it.

## Global constraints

- `SL2b`: the sandbox includes only installed engine headers and links only `spade::` targets; a missing capability is a need for Core, never a widened include path.
- `SL15b`: every rule is in a display-free header and tested in `spade_tests`; the window never decides anything.
- No wall-clock time in stepped code; runs step in whole fixed steps (`kBuiltinStepNs` = 4 ms, 4 substeps).
- No golden moves. A task that would move one stops and goes to the lead (`TD-1`).
- Red, then green: each task's tests are committed failing first, and the commit message records how they failed.
- Before "ready for review": `scripts/gcc-check.sh` on every changed C++ file, and the full suite on both MSVC presets (`scripts\test.ps1 -Preset release` and `-Preset debug`), counts quoted from ctest's own summary.
- Commits name explicit paths. Never `git stash`, `checkout -- <path>`, `restore` or `clean`. Never push.
- Names, never indices (`SCN-004`, `SCN-009`): an edit addresses a model, asset, vehicle or material by its name.
- Floats in the edit text form use 9 significant digits (the scene file's spelling), so a command line round-trips bit for bit.
- Errors say what, why and what next.

## Review focus

The inputs the design implies but no single requirement names, most likely to bite first. Each has its test in the task named.

1. **A scene that shares a model, opened and saved unedited**, must come back byte for byte; copy-on-edit must happen on the first edit, never on open (Task 3).
2. **Names that collide when objects are added**: duplicate, place and paste must produce a name unique across assets and vehicles together, also when "gate_2" already exists (Task 2).
3. **A save that cannot complete** (a read-only target, a missing folder): the document stays dirty, the old file is untouched, no temporary file is left behind, and the error names the path (Task 5).
4. **Undo across a save**: undoing a saved edit marks the document dirty again, and redoing back to the saved state marks it clean (Task 1).
5. **A rebuild that renames or removes a vehicle, or changes its model**: a renamed vehicle starts at its start pose, a removed one is dropped, a re-modelled one carries its pose and rates (Task 7).

---

## File structure

All new code is in `sandbox/`, in the files' existing flat layout:

| File | Responsibility |
|---|---|
| `editor_document.hpp` | `Document<Desc>`: the description, its validator, undo and redo, the save point |
| `editor_scene_edits.hpp` | the scene's edits (`SceneEdit`) and `apply()`, unique names |
| `editor_model_params.hpp` | the inspector's model table: every `ModelType` parameter as a path and a kind |
| `editor_world_edits.hpp` | the world's edits (`WorldEdit`) and `apply()`, the new-world template |
| `editor_files.hpp`, `.cpp` | atomic writes, open, save, save-as, the hash follow |
| `editor_run.hpp`, `.cpp` | a run built from the documents, stepped and rebuilt with carry |
| `editor_edit_text.hpp` | the one-line text form of every edit, both directions |
| `editor_library.hpp`, `.cpp` | library items, placement by copy, (Task 13) library files |
| `editor_pick.hpp` | what the cursor hits, by marching the composed SDF |
| `editor_sdf_edit.hpp` | the terrain edit helper (Task 15, Core reviews) |
| `editor_session.hpp`, `.cpp` | the window: panels over the documents, like the other sessions |

Tests are `tests/test_editor_<unit>.cpp`, registered in the sandbox block of `tests/CMakeLists.txt`.

## Order and Core dependencies

Core's queue, as the lead gave it: T4, module-API stage 4, world file v3, model and asset YAML, the SDF helper review. Tasks 1–12 need nothing from Core and come first; Tasks 13–16 wait on Core's items in that order.

| Task | Needs from Core | EDT rows |
|---|---|---|
| 1 Document | — | 001, 003, 004 |
| 2 Scene edits | — | 003, 008 |
| 3 Model parameters | — | 013, 014, 019 |
| 4 World edits | — | 015 |
| 5 Files | — | 005, 006, 015 |
| 6 Hash follow | — | 016, 017 |
| 7 Run | — (single rotor speed carried until Task 16) | 001, 007, 009, 018 |
| 8 Edit text and CLI | — | 002 |
| 9 Library, built-ins | — | 012 |
| 10 Window | — | 008, 013, 021 |
| 11 Scene picker | — | (`SL13`) |
| 12 Live smoke | — | 011 |
| 13 Library files | model and asset YAML | 020 |
| 14 Physics records in worlds | world file v3 | 018 |
| 15 Terrain edits | the SDF helper review | 015 |
| 16 Per-rotor carry | per-rotor spawn speeds | 007 |
| every task | — | 010 (`SL2b`, a global constraint, checked at each review) |

Each task is its own `interface/editor-<n>-<name>` branch from master, gated by the lead's review before the next one starts.

---

### Task 1: The document

**Files:** create `sandbox/editor_document.hpp`; test `tests/test_editor_document.cpp`.

**Interfaces — produces:**

```cpp
namespace spade::sandbox::editor {
inline constexpr std::size_t kHistoryBound = 200;

template <class Desc>
class Document {
  public:
    using Validator = std::function<Result<void>(const Desc&)>;
    Document(Desc initial, Validator validate, std::size_t bound = kHistoryBound);
    [[nodiscard]] const Desc& desc() const noexcept;
    // Validates `next`; on success the current description goes onto the undo
    // stack, the redo stack is cleared, and `next` becomes current.
    [[nodiscard]] Result<void> commit(Desc next);
    bool undo();  // false when there is nothing to undo
    bool redo();
    [[nodiscard]] bool can_undo() const noexcept;
    [[nodiscard]] bool can_redo() const noexcept;
    void mark_saved() noexcept;           // the current state is what is on disk
    [[nodiscard]] bool dirty() const noexcept;
    std::filesystem::path path;           // empty until saved
};
using SceneDocument = Document<scene::SceneDesc>;
using WorldDocument = Document<WorldDesc>;
[[nodiscard]] SceneDocument make_scene_document(scene::SceneDesc desc, std::filesystem::path path);
[[nodiscard]] WorldDocument make_world_document(WorldDesc desc, std::filesystem::path path);
}
```

The save point is a serial number: every commit takes the next serial, undo and redo move between them, and `dirty()` compares the current serial with the saved one.

- [ ] **Step 1: write the failing tests** (`Document<int>` with a validator refusing negatives, then the two real factories):

```cpp
TEST(EditorDocument, ACommitIsUndoneAndRedoneExactly) {
    Document<int> d(1, [](const int& v) -> Result<void> { if (v < 0) return std::unexpected(Error{Code::invalid_argument, "negative"}); return {}; });
    ASSERT_TRUE(d.commit(2).has_value());
    ASSERT_TRUE(d.commit(3).has_value());
    EXPECT_TRUE(d.undo()); EXPECT_EQ(d.desc(), 2);
    EXPECT_TRUE(d.redo()); EXPECT_EQ(d.desc(), 3);
}
TEST(EditorDocument, ARefusedCommitChangesNothing) {
    Document<int> d(1, /* as above */);
    const auto r = d.commit(-5);
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().context, "negative");
    EXPECT_EQ(d.desc(), 1); EXPECT_FALSE(d.can_undo()); EXPECT_FALSE(d.dirty());
}
TEST(EditorDocument, ANewCommitClearsRedo) { /* commit 2, undo, commit 4: can_redo() false, desc 4 */ }
TEST(EditorDocument, TheHistoryKeepsTheNewestBoundEntries) { /* bound 3: commit 2..6; undo 3 times reaches 3, a 4th undo is false */ }
// Review focus 4.
TEST(EditorDocument, UndoingASavedEditMakesItDirtyAndRedoingMakesItClean) {
    Document<int> d(1, /* as above */);
    ASSERT_TRUE(d.commit(2).has_value()); d.mark_saved(); EXPECT_FALSE(d.dirty());
    d.undo(); EXPECT_TRUE(d.dirty());
    d.redo(); EXPECT_FALSE(d.dirty());
}
TEST(EditorDocument, TheSceneDocumentValidatesWithValidateScene) {
    auto d = make_scene_document(sample_scene(), {});
    auto bad = d.desc(); bad.vehicles.push_back({"x", "no_such_model", {}});
    EXPECT_FALSE(d.commit(bad).has_value());
}
```

`tests/editor_test_support.hpp`, created here and used by every editor test, holds the shared helpers. Later tasks use them by these names:
- **Scenes:** `hover_scene()` (the path of `assets/scenes/hover.scene.yaml`) and `sample_scene()` (it loaded through `scene::load_scene_file`; its one vehicle is `hover_quad_0`, and it has no assets).
- **Models:** `quad_model()` (the hover scene's model), `scene_with(model)` (one vehicle `quad_0`) and `two_vehicles_sharing(name)` (`quad_0` and `quad_1` on one model of that name).
- **Assets:** `asset_named(name)` (a unit box asset) and `with_asset(scene, name)`.
- **Lookups:** `model_named(scene, name)` and `scene_of(yaml_text)`.
- **Files:** `temp_dir()`, `read_bytes(path)`, `write_bytes(path, text)`, `make_read_only(path)`, `committed_scene_files()` and `canonical_bytes(path)` (a file's bytes without its `# provenance:` lines).
- **Starts:** `raised_start()`, the hover start one metre higher.

- [ ] **Step 2: run** `spade_tests --gtest_filter=EditorDocument*`. Expected: C1083, no `editor_document.hpp`.
- [ ] **Step 3: implement** the template in the header: `std::deque<Entry>` undo and redo stacks, each `Entry{Desc desc; uint64_t serial;}`; `commit()` calls the validator first and touches nothing on failure; the bound drops from the front of the undo deque.
- [ ] **Step 4: run** the filter; expected PASS. Then the gcc check on both files.
- [ ] **Step 5: commit** `tests/test_editor_document.cpp tests/editor_test_support.hpp tests/CMakeLists.txt sandbox/editor_document.hpp`, red then green as two commits.

### Task 2: Scene edits

**Files:** create `sandbox/editor_scene_edits.hpp`; test `tests/test_editor_scene_edits.cpp`.

**Interfaces — consumes** Task 1's `SceneDocument`. **Produces:**

```cpp
struct AddAsset { scene::SceneAsset asset; };                 // its name is made unique
struct AddVehicle { scene::SceneVehicle vehicle; vehicles::ModelType model; };  // the model is copied for it (Q1)
struct RemoveObject { std::string name; };                   // an asset or a vehicle; a vehicle's own model goes too
struct RenameObject { std::string from; std::string to; };   // refused if `to` is taken
struct DuplicateObject { std::string name; };                // the copy is "<name>_<n>", n the lowest free
struct MoveObject { std::string name; std::size_t to; };     // reorder within its list (SCN-005)
struct SetAssetPose { std::string asset; SdfPose pose; };
struct SetAssetMaterial { std::string asset; std::string material; };   // the visual's material
struct SetVehicleStart { std::string vehicle; VehicleSpawn start; };     // design frame (DBE-013)
struct AddMaterial { MaterialDesc material; };
struct RepointWorld { std::string file; uint64_t hash; };
using SceneEdit = std::variant<AddAsset, AddVehicle, RemoveObject, RenameObject, DuplicateObject, MoveObject,
                               SetAssetPose, SetAssetMaterial, SetVehicleStart, AddMaterial, RepointWorld>;
[[nodiscard]] Result<void> apply(SceneDocument& doc, const SceneEdit& edit);
[[nodiscard]] std::string unique_name(const scene::SceneDesc& scene, std::string_view base);
```

Every `apply` copies the description, changes the copy, and commits it, so validation (`validate_scene`) and undo come from Task 1.

- [ ] **Step 1: failing tests**, one per edit kind plus:

```cpp
// Review focus 2.
TEST(EditorSceneEdits, ANewNameIsUniqueAcrossAssetsAndVehicles) {
    scene::SceneDesc s = sample_scene();          // holds vehicle "hover_quad_0"
    s.assets.push_back(asset_named("gate_2"));
    EXPECT_EQ(unique_name(s, "gate"), "gate");
    s.assets.push_back(asset_named("gate"));
    EXPECT_EQ(unique_name(s, "gate"), "gate_3");   // gate_2 is taken
    EXPECT_EQ(unique_name(s, "hover_quad_0"), "hover_quad_0_2");
}
TEST(EditorSceneEdits, DuplicatingAVehicleCopiesItsModelToo) {
    auto d = make_scene_document(sample_scene(), {});
    ASSERT_TRUE(apply(d, DuplicateObject{"hover_quad_0"}).has_value());
    ASSERT_EQ(d.desc().vehicles.size(), 2u);
    EXPECT_NE(d.desc().vehicles[0].model, d.desc().vehicles[1].model);   // one model per vehicle (Q1)
}
TEST(EditorSceneEdits, ARenameOntoATakenNameIsRefusedAndChangesNothing) { /* ... */ }
TEST(EditorSceneEdits, ReorderingIsAnEditAndIsUndone) { /* MoveObject, then undo restores scene_to_yaml bytes */ }
TEST(EditorSceneEdits, AnEditTheValidatorRefusesNamesTheCause) {
    auto d = make_scene_document(with_asset(sample_scene(), "box_1"), {});
    SdfPose p; p.scale = 0.0f;
    const auto r = apply(d, SetAssetPose{"box_1", p});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("scale"), std::string::npos);
}
```

- [ ] **Step 2: run**; expected C1083. **Step 3: implement** with `std::visit`. **Step 4: run**; PASS; gcc check. **Step 5: commit** red, then green.

### Task 3: Model parameters, one model per vehicle

**Files:** create `sandbox/editor_model_params.hpp`; modify `sandbox/editor_scene_edits.hpp` (adds `SetModelParam` to `SceneEdit`); test `tests/test_editor_model_params.cpp`.

**Interfaces — produces:**

```cpp
enum class ParamKind { scalar, vec3, quat, text };
struct ModelParam { vehicles::ModelIssue::Element element; std::string_view field; ParamKind kind; };
// The inspector's table: every ModelType parameter, in the struct's order.
[[nodiscard]] std::span<const ModelParam> model_params();
using ParamValue = std::variant<float, glm::vec3, glm::quat, std::string>;
struct SetModelParam { std::string vehicle; vehicles::ModelIssue::Element element; std::size_t index;
                       std::string field; ParamValue value; };
[[nodiscard]] Result<ParamValue> get_model_param(const vehicles::ModelType& m, vehicles::ModelIssue::Element e,
                                                 std::size_t index, std::string_view field);
```

`apply(doc, SetModelParam)`:
1. find the vehicle's model;
2. if another vehicle uses the same model, copy it under `unique_name(scene, model.name)` and point this vehicle at the copy (`EDT-019`);
3. set the field;
4. refuse with every `issues()` entry, field by field, if any (`EDT-014`);
5. commit.

There is no edit that adds or removes a rotor, drag body or IMU mount (`EDT-013`).

- [ ] **Step 1: failing tests:**

```cpp
TEST(EditorModelParams, EveryFieldInTheTableReadsAndWritesBack) {
    vehicles::ModelType m = quad_model();
    for (const ModelParam& p : model_params()) {
        const std::size_t idx = 0;
        auto v = get_model_param(m, p.element, idx, p.field);
        ASSERT_TRUE(v.has_value()) << p.field;
        auto d = make_scene_document(scene_with(m), {});
        EXPECT_TRUE(apply(d, SetModelParam{"quad_0", p.element, idx, std::string(p.field), *v}).has_value()) << p.field;
    }
}
TEST(EditorModelParams, AnEditToASharedModelReachesThatVehicleAlone) {
    auto d = make_scene_document(two_vehicles_sharing("quad"), {});
    const std::string before = scene::scene_to_yaml(d.desc()).value();
    ASSERT_TRUE(apply(d, SetModelParam{"quad_1", Element::rotor, 0, "radius", 0.15f}).has_value());
    EXPECT_EQ(d.desc().vehicles[0].model, "quad");
    EXPECT_NE(d.desc().vehicles[1].model, "quad");
    EXPECT_EQ(model_named(d.desc(), "quad").rotors[0].radius, model_named(scene_of(before), "quad").rotors[0].radius);
}
// Review focus 1.
TEST(EditorModelParams, OpeningASharedModelSceneCopiesNothing) {
    const scene::SceneDesc s = two_vehicles_sharing("quad");
    auto d = make_scene_document(s, {});
    EXPECT_EQ(scene::scene_to_yaml(d.desc()).value(), scene::scene_to_yaml(s).value());
}
TEST(EditorModelParams, AnInvalidValueIsRefusedNamingItsField) {
    auto d = make_scene_document(scene_with(quad_model()), {});
    const auto r = apply(d, SetModelParam{"quad_0", Element::rotor, 2, "radius", -1.0f});
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find("radius"), std::string::npos);
}
TEST(EditorModelParams, NoEditChangesAPartCount) { /* every table field edited once: rotors/drag/imu sizes unchanged */ }
```

- [ ] **Steps 2–5** as in Task 1.

### Task 4: World edits and the new-world template

**Files:** create `sandbox/editor_world_edits.hpp`; test `tests/test_editor_world_edits.cpp`.

**Interfaces — produces:**

```cpp
struct RenameWorld { std::string name; };
struct SetEnvironment { Environment environment; };
struct SetCapacities { Capacities capacities; };
struct AddWorldMaterial { MaterialDesc material; };
struct SetWorldMaterial { std::string name; MaterialDesc material; };
struct SetLighting { LightingDesc lighting; };
struct AddSpawn { SpawnPoint spawn; }; struct SetSpawn { std::string name; SpawnPoint spawn; }; struct RemoveSpawn { std::string name; };
struct AddProp { PropDesc prop; }; struct SetProp { std::size_t index; PropDesc prop; }; struct RemoveProp { std::size_t index; };
using WorldEdit = std::variant<RenameWorld, SetEnvironment, SetCapacities, AddWorldMaterial, SetWorldMaterial, SetLighting,
                               AddSpawn, SetSpawn, RemoveSpawn, AddProp, SetProp, RemoveProp>;
[[nodiscard]] Result<void> apply(WorldDocument& doc, const WorldEdit& edit);
// An empty world with a ground plane at y = 0: the "new world" template.
[[nodiscard]] WorldDesc new_world(std::string name);
```

The validator is `validate_world_desc()` (its `Result<uint32_t>` mapped to `Result<void>`).

- [ ] **Step 1: failing tests:** each edit's effect and undo; a zero capacity is refused naming the field; `new_world("w")` validates and round-trips through `world_to_yaml`/`world_from_yaml` bit for bit.
- [ ] **Steps 2–5** as in Task 1.

### Task 5: Files: open, save, save as

**Files:** create `sandbox/editor_files.hpp`, `sandbox/editor_files.cpp`; test `tests/test_editor_files.cpp`. The `.cpp` is added to the `spade_sandbox` sources, and to `spade_tests` with `target_sources()` in the sandbox block of `tests/CMakeLists.txt`, so the tests run the same code; `editor_run.cpp` and `editor_library.cpp` follow the same pattern in Tasks 7 and 9.

**Interfaces — produces:**

```cpp
struct OpenedScene { SceneDocument scene; WorldDocument world; };
[[nodiscard]] Result<void> write_atomically(const std::filesystem::path& target, std::string_view text);
[[nodiscard]] Result<OpenedScene> open_scene(const std::filesystem::path& scene_file);   // and its world
[[nodiscard]] Result<WorldDocument> open_world(const std::filesystem::path& world_file);
[[nodiscard]] Result<void> save_world(WorldDocument& world);
// Saves a dirty world first, so the scene never pins a hash that is not on disk.
[[nodiscard]] Result<void> save_scene(SceneDocument& scene, WorldDocument& world);
[[nodiscard]] Result<void> save_scene_as(SceneDocument& scene, WorldDocument& world, const std::filesystem::path& target);
// The world's path relative to a scene file's folder, forward slashes (SCN-001).
[[nodiscard]] std::string relative_world_path(const std::filesystem::path& scene_dir,
                                              const std::filesystem::path& world_file);
```

`write_atomically` writes `<target>.tmp-spade` in the target's folder (binary, LF), flushes and closes it, then `std::filesystem::rename`s it over the target. On any failure it removes the temporary file and leaves the target as it was.

- [ ] **Step 1: failing tests:**

```cpp
TEST(EditorFiles, EveryCommittedSceneSavesBackByteForByte) {
    for (const auto& file : committed_scene_files()) {   // assets/scenes/*.scene.yaml, golden/scenes/*.scene.yaml
        auto opened = open_scene(file);
        ASSERT_TRUE(opened.has_value()) << file;
        const fs::path out = temp_dir() / file.filename();
        ASSERT_TRUE(save_scene_as(opened->scene, opened->world, out).has_value());
        EXPECT_EQ(read_bytes(out), canonical_bytes(file)) << file;   // provenance lines aside
    }
}
// Review focus 3.
TEST(EditorFiles, AFailedSaveLeavesTheOldFileAndTheDocumentDirty) {
    const fs::path out = temp_dir() / "locked.scene.yaml";
    write_bytes(out, "old"); make_read_only(out);
    auto opened = open_scene(hover_scene());
    ASSERT_TRUE(apply(opened->scene, SetVehicleStart{"hover_quad_0", raised_start()}).has_value());
    const auto r = save_scene_as(opened->scene, opened->world, out);
    ASSERT_FALSE(r.has_value());
    EXPECT_NE(r.error().context.find(out.filename().string()), std::string::npos);
    EXPECT_EQ(read_bytes(out), "old");
    EXPECT_FALSE(fs::exists(out.string() + ".tmp-spade"));
    EXPECT_TRUE(opened->scene.dirty());
}
TEST(EditorFiles, SaveAsElsewhereRewritesTheWorldPathSoItStillComposes) {
    auto opened = open_scene(hover_scene());
    const fs::path out = temp_dir() / "deeper" / "copy.scene.yaml";
    fs::create_directories(out.parent_path());
    ASSERT_TRUE(save_scene_as(opened->scene, opened->world, out).has_value());
    EXPECT_TRUE(scene::compose_file(out).has_value());
}
TEST(EditorFiles, SavingASceneSavesItsDirtyWorldFirst) { /* edit world, save scene: both on disk, scene pins the new hash */ }
TEST(EditorFiles, OpeningASceneWhoseWorldIsMissingNamesTheWorld) { /* ... */ }
TEST(EditorFiles, RelativeWorldPathsUseForwardSlashes) {
    EXPECT_EQ(relative_world_path("C:/a/scenes", "C:/a/worlds/w.world.yaml"), "../worlds/w.world.yaml");
}
```

- [ ] **Steps 2–5** as in Task 1.

### Task 6: The scene follows its world

**Files:** modify `sandbox/editor_files.hpp`, `.cpp`; test `tests/test_editor_hash_follow.cpp`.

**Interfaces — produces:**

```cpp
struct WorldSaved {
    uint64_t old_hash = 0;
    uint64_t new_hash = 0;
    bool repinned = false;              // the open scene now pins new_hash
    std::string why_not;                // compose()'s refusal when not repinned
    std::vector<std::filesystem::path> others;  // project scenes still pinning old_hash
};
[[nodiscard]] Result<WorldSaved> save_world_and_follow(WorldDocument& world, SceneDocument& scene,
                                                       const std::filesystem::path& project_dir);
// Re-pins another scene file to `world` on the user's word; refuses if it does not compose.
[[nodiscard]] Result<void> repin_scene_file(const std::filesystem::path& scene_file, const WorldDesc& world);
// compose()'s refusal for the open documents, or "" when they compose (EDT-017).
[[nodiscard]] std::string compose_conflict(const SceneDocument& scene, const WorldDocument& world);
```

The re-pin is a `RepointWorld` edit, so it can be undone.

- [ ] **Step 1: failing tests:**
  - a saved world edit re-pins the open scene, and `compose_file` composes it;
  - a world edit that adds a material named as the scene's own is not re-pinned, and `why_not` names the material;
  - a copy of the scene in the project folder appears in `others` and is unchanged until `repin_scene_file`;
  - `compose_conflict` is empty for a clean pair.
- [ ] **Steps 2–5** as in Task 1.

### Task 7: The run, rebuilt with carry

**Files:** create `sandbox/editor_run.hpp`, `.cpp`; test `tests/test_editor_run.cpp`.

**Interfaces — consumes** `BuiltinRecords`, `instance_of` (`builtin_records.hpp`). **Produces:**

```cpp
class EditorRun {
  public:
    [[nodiscard]] static Result<EditorRun> start(const scene::SceneDesc& scene, const WorldDesc& world,
                                                 const BuiltinRecords& records);
    [[nodiscard]] Result<void> step(uint32_t steps);
    // Composes and instantiates the edited documents and carries each vehicle
    // that keeps its name. On a refusal this run is unchanged and the error
    // is compose()'s or create()'s, verbatim.
    [[nodiscard]] Result<void> rebuild(const scene::SceneDesc& scene, const WorldDesc& world);
    [[nodiscard]] uint64_t tick() const noexcept;
    [[nodiscard]] const Simulation& sim() const noexcept;
    [[nodiscard]] std::optional<FrameState> state_of(std::string_view vehicle) const;   // design frame (DBE-013)
    // Records the world file cannot hold yet (EDT-018): true until world file v3.
    [[nodiscard]] static constexpr bool records_saved_with_world() noexcept { return false; }
};
[[nodiscard]] BuiltinRecords records_for(const std::filesystem::path& scene_file);   // built-in stem, else defaults
```

The carry takes `vehicle_state()` (pose, velocity, rates, in the design frame) into the new spawn. Its single `rotor_omega` is the mean of the vehicle's rotor speeds; Task 16 makes it per-rotor.

- [ ] **Step 1: failing tests:**

```cpp
TEST(EditorRun, StartingFromTheDocumentsIsComposeFileOnTheirFiles) {
    auto opened = open_scene(hover_scene());
    auto run = EditorRun::start(opened->scene.desc(), opened->world.desc(), records_for(hover_scene()));
    auto from_file = scene::instantiate(*scene::compose_file(hover_scene()), instance_of(records_for(hover_scene())),
                                        kBuiltinStepNs, kBuiltinSubsteps);
    EXPECT_EQ(spade::testing::state_digest(run->sim()), spade::testing::state_digest(from_file->sim));
}
TEST(EditorRun, ARebuildCarriesAVehiclesPoseAndRates) { /* step 200, move an asset, rebuild: hover_quad_0 within 1e-5 of before */ }
TEST(EditorRun, ARefusedRebuildKeepsTheRun) { /* a scene that does not compose: rebuild fails, tick unchanged, state digest unchanged */ }
// Review focus 5.
TEST(EditorRun, ARenamedVehicleStartsFreshAndARemovedOneIsDropped) {
    /* rename hover_quad_0 -> q: its state is its start pose; remove it: state_of("q") is nullopt */
}
TEST(EditorRun, ARemodelledVehicleKeepsItsPose) { /* SetModelParam rotor radius, rebuild: pose carried */ }
```

- [ ] **Steps 2–5** as in Task 1.

### Task 8: The edit text and the command line

**Files:** create `sandbox/editor_edit_text.hpp`; modify `sandbox/main.cpp`; test `tests/test_editor_edit_text.cpp` and a ctest CLI test in `tests/CMakeLists.txt`.

**Interfaces — produces:**

```cpp
[[nodiscard]] std::string format_edit(const SceneEdit& edit);   // one line, 9 significant digits
[[nodiscard]] Result<SceneEdit> parse_edit(std::string_view line);
[[nodiscard]] std::string format_world_edit(const WorldEdit& edit);
[[nodiscard]] Result<WorldEdit> parse_world_edit(std::string_view line);
```

Lines look like `scene set-asset-pose gate_1 pos 0 1.79999995 0 rot 1 0 0 0 scale 1`, or `world set-environment ...`.

The command line: `spade_sandbox --edit <scene.yaml> [--apply <edits.txt>] [--save <out.yaml>] [--run <steps>]`. It runs headless; exit 0 on success, 1 when an edit is refused (printing its line number and the cause), 2 on bad arguments.

- [ ] **Step 1: failing tests:**
  - every `SceneEdit` and `WorldEdit` alternative round-trips: `parse(format(e))` re-formats to the same line, and the value equals bitwise;
  - a malformed line names its first bad token;
  - the CLI test edits a copy of `assets/scenes/hover.scene.yaml`, saves it, and `compose_file` composes the result.
- [ ] **Steps 2–5** as in Task 1. The CLI test is `add_test(NAME EditorCli.EditsSavesAndComposes COMMAND spade_sandbox --edit ...)`, comparing against expected text in the test folder.

### Task 9: The library, built-in items, placement by copy

**Files:** create `sandbox/editor_library.hpp`, `.cpp`; test `tests/test_editor_library.cpp`.

**Interfaces — produces:**

```cpp
struct LibraryItem {
    enum class Kind { model, asset };
    Kind kind = Kind::asset;
    std::string name;
    std::optional<vehicles::ModelType> model;
    std::optional<scene::SceneAsset> asset;
};
[[nodiscard]] std::vector<LibraryItem> builtin_library();   // quadrotor template; box, sphere, cylinder, capsule
[[nodiscard]] std::string vehicle_kind_of(const vehicles::ModelType& model);   // "quadrotor" today
// Adds a copy: a model as AddVehicle (its own model copy, Q1), an asset as AddAsset. Returns the placed name.
[[nodiscard]] Result<std::string> place(SceneDocument& doc, const LibraryItem& item, const SdfPose& at);
```

- [ ] **Step 1: failing tests:**
  - placing an asset twice gives two assets with unique names, and the scene holds no path;
  - placing the quadrotor twice gives two vehicles and two models;
  - a placed model equals the library's, except for its name;
  - `vehicle_kind_of` of the template is "quadrotor".
- [ ] **Steps 2–5** as in Task 1.

### Task 10: The window

**Files:** create `sandbox/editor_pick.hpp`, `sandbox/editor_session.hpp`, `.cpp`; modify `sandbox/gl_target_sink.hpp`, `.cpp` (an `attach_editor(EditorPanels*)` like `attach_drone`), `sandbox/main.cpp` (`--editor [scene.yaml]`); test `tests/test_editor_pick.cpp`.

**Interfaces — produces:**

```cpp
// The object under a ray: the nearest hit of the composed world's SDF,
// attributed to the asset whose posed collider is nearest the hit point, or a
// vehicle whose body is within its proxy radius. Display-free.
[[nodiscard]] std::optional<std::string> pick(const scene::SceneDesc& scene, const scene::ComposedScene& composed,
                                              const EditorRun* run, glm::vec3 origin, glm::vec3 direction);
struct EditorPanels {                      // what the sink's ImGui code reads and writes; no rules in it
    std::vector<std::string> hierarchy;    // scene order
    std::string selected;
    std::vector<SceneEdit> pending;        // committed by the session on release (INT-2)
    std::vector<WorldEdit> pending_world;
    std::string status;                    // the last refusal, verbatim
};
class EditorSession { /* create(OpenedScene) no window; attach(GlTargetSink&); frame(const FrameInput&, float dt) */ };
```

The window follows the spec's §13 (look and feel, the user's direction of 2026-10-06; EDT-021). Its panels, docked into one dockspace with a default layout, are:
- the toolbar: run, pause, step and the tick; undo and redo; save; and the menus (Open, Save, Save As, scene sources);
- the hierarchy, in scene order, with the scene and the world as its first two rows;
- the inspector, for the selection:
  - a vehicle: its start, then its model through `model_params()`, with the parts and the IMU noise folded;
  - an asset: its pose and material;
  - the scene: its materials and spare counts;
  - the world: its environment, capacities, lighting, spawns and props, with the sky colours folded;
  - the records not saved with the world (EDT-018), labelled as such;
- the library, by kind;
- the viewport, as the dockspace's pass-through centre.

Each widget queues an edit on release; the session applies the queue, so nothing in ImGui code decides.

**Files, in addition:**
- create `sandbox/ui_theme.hpp`, holding `apply_spade_theme()`, `property_row()`, `status_text()` and the default dock layout;
- test `tests/test_sandbox_ui_conventions.cpp`, the source scan.

- [ ] **Step 1: failing tests:**
  - `pick`: a ray at a box asset returns its name; one at a drone returns the vehicle's name; a ray into the sky returns nothing.
  - The source scan: no `Separator`, `SeparatorText` or bordered `BeginChild` in `sandbox/`, and style calls only in `ui_theme.hpp`. It is red against today's panels until the sandbox cleanup lands.
- [ ] **Steps 2–5** as in Task 1. Then a live check in the window, and the live smoke in Task 12.
- [ ] **Acceptance:** the live smoke's stills of the default layout and of each panel (hierarchy, inspector for a vehicle, a model with its sections open, the world, the library, the toolbar). The lead sends the set to the user.

### Task 11: The scene picker (`SL13`)

**Files:** modify `sandbox/editor_library.hpp`, `.cpp` (`scene_sources()`); test in `tests/test_editor_library.cpp`.

**Interfaces — produces:**

```cpp
struct SceneSource { enum class Kind { builtin, test_scenario, saved }; Kind kind; std::string name; std::filesystem::path file; };
[[nodiscard]] std::vector<SceneSource> scene_sources(const std::filesystem::path& assets, const std::filesystem::path& saved_dir);
```

- [ ] **Step 1: failing tests:** every `assets/scenes/` file is listed as built-in; a file in the saved folder is listed as saved; each listed file opens. Test scenarios join as composer task 5 converts them; none are listed until then.
- [ ] **Steps 2–5** as in Task 1.

### Task 12: The live smoke's `editor` main function (`EDT-011`)

**Files:** modify `sandbox/live_tour.cpp` (declared steps and the main function).

Steps, each checking what it shows:

| Step | What it does | What it checks |
|---|---|---|
| `editor.open` | opens `assets/scenes/hover.scene.yaml` and its world | it opened |
| `editor.place` | places a box from the library | names are unique |
| `editor.move` | drags it in the ground plane | its pose changed |
| `editor.inspect` | edits the drone's rotor radius | copy-on-edit, and the other vehicles are unchanged |
| `editor.undo` | undoes, then redoes | `scene_to_yaml` bytes equal |
| `editor.run` | runs, then edits while running | the rebuild carries the drone's pose |
| `editor.world` | edits the world's wind, saves to a temporary folder | the scene re-pinned |
| `editor.save` | saves the scene to a temporary folder and reopens it | byte-equal |

- [ ] **Steps:** the steps are written and declared; a red run with `--inject skip:editor.save` FAILs; a green run PASSes; the video stays under `kSendableBytes`.

### Task 13: Library files and Save to library (`EDT-020`) — needs Core's model and asset YAML

**Files:** modify `sandbox/editor_library.hpp`, `.cpp`; test in `tests/test_editor_library.cpp`.

**Interfaces — produces:**

```cpp
[[nodiscard]] std::vector<LibraryItem> load_library(const std::filesystem::path& folder);   // plus builtin_library()
// Writes the configured model or asset as a NEW item; never overwrites. Returns its file.
[[nodiscard]] Result<std::filesystem::path> save_to_library(const std::filesystem::path& folder, const LibraryItem& item);
```

- [ ] **Step 1: failing tests:**
  - saving an item whose name is taken writes `<name>_2` and leaves the existing file's bytes unchanged;
  - a saved item loads back equal;
  - an unreadable file is reported by name and does not stop the rest loading.
- [ ] **Steps 2–5** as in Task 1, using Core's `model_to_yaml`/`model_from_yaml` and `asset_to_yaml`/`asset_from_yaml`.

### Task 14: The physics records saved with the world (`EDT-018`) — needs world file v3

**Files:** modify `sandbox/editor_run.hpp`, `.cpp`, `sandbox/editor_world_edits.hpp`; delete `sandbox/builtin_records.hpp` and `tests/test_sandbox_builtin_records.cpp`; regenerate `assets/worlds/*.world.yaml` with their records.

- [ ] **Step 1: failing tests:**
  - a world edit of turbulence, contacts and grid saves and reloads;
  - `records_saved_with_world()` is true;
  - `test_viewer_successors.cpp` takes the records from the world files, not from code, and still matches every viewer row.
- [ ] **Steps 2–5** as in Task 1. No golden moves: the successor test is the check.

### Task 15: Terrain edits (`EDT-015`) — Interface writes, Core reviews

**Files:** create `sandbox/editor_sdf_edit.hpp`; modify `sandbox/editor_world_edits.hpp` (adds the terrain edits to `WorldEdit`); test `tests/test_editor_sdf_edit.cpp`.

**Interfaces — produces:**

```cpp
// The pieces a union joins at the top of the program: [first node, last node] ranges.
[[nodiscard]] std::vector<std::pair<uint32_t, uint32_t>> union_pieces(const SdfProgram& program);
[[nodiscard]] Result<SdfProgram> add_primitive(const SdfProgram& program, SdfPrim kind, glm::vec4 params, const SdfPose& pose);
[[nodiscard]] Result<SdfProgram> remove_piece(const SdfProgram& program, std::size_t piece);
[[nodiscard]] Result<SdfProgram> repose_piece(const SdfProgram& program, std::size_t piece, const SdfPose& pose);
[[nodiscard]] Result<SdfProgram> set_node_params(const SdfProgram& program, uint32_t node, glm::vec4 params);
```

- [ ] **Step 1: failing tests:**
  - each operation leaves a program that `SdfProgram::validate()` accepts;
  - at sample points, `eval()` of the result equals the analytic union, difference or pose;
  - a piece inside an intersection or a subtraction is refused for remove and re-pose, naming the operator.
- [ ] **Steps 2–5** as in Task 1. **Gate:** Core reviews the helper before merge, last in Core's queue.

### Task 16: Per-rotor carry — needs Core's per-rotor spawn speeds

**Files:** modify `sandbox/editor_run.cpp`; test in `tests/test_editor_run.cpp`.

- [ ] **Step 1: failing test:** after a rebuild, each rotor's speed equals its speed before, row by row.
- [ ] **Steps 2–5** as in Task 1.

---

## Placeholder: the airflow HUD and views

To be filled from Physics' airflow joint spec (the user's ruling, 2026-10-05). Interface's part, as the lead gave it:
- an **airspeed and forces HUD**: each vehicle's airspeed, angle of attack, and aero forces and moments, live;
- **the airflow views** in the sandbox and the editor's viewport;
- an **airflow main function** in the live smoke.

The tasks, their interfaces and their tests are written once the spec names its data: the field and probe API, and the per-vehicle aero readouts. They slot in after Task 10, which owns the viewport and panels.

## Self-review against the spec

- §1 done-when 1–7: Tasks 5, 2/9/1, 3/10, 4/5/6/15, 7, 8, 12.
- §2 scope: everything in the first-cut column has a task. The later column stays out: components, mesh browser, model authoring, regions, scrub.
- §3 documents: Tasks 1, 2 and 4. §4 library and inspector: Tasks 3, 9, 10 and 13. §5 running: Tasks 7 and 16. §6 worlds: Tasks 4, 5, 6, 14 and 15. §7 saving and opening: Tasks 5 and 11. §8 API needs: the order table.
- EDT-001 to EDT-021 each map to a task in the order table (EDT-021, the look and feel, added 2026-10-06 from the user's direction).
- The review focus items map to Tasks 3, 2, 5, 1 and 7, each with its test.
