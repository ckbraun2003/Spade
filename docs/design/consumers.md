# Spade — consumers

Spade is a library. A consumer builds against the installed `spade::` package (`find_package(spade CONFIG)`) and the file formats Spade defines. **This is the only document in the library where a consumer's own rule series may appear.**

## KAT

KAT is a drone SDK and editor, and the consumer Spade was first built for. Spade was extracted from the KAT monorepo with its full history on 2026-09-28.

| What KAT uses | From Spade |
|---|---|
| Simulation, worlds, vehicles, sensors, rendering | the installed `spade::` targets |
| World description | the world-file format (`world_version`, owned by Core) |
| Session ring and replay | the snapshot format (`kSnapshotVersion`, owned by Core) |
| Scene description (what Kat compiles a build into) | the scene-file format (`engine/scene/scene_file.hpp`, owned by Core) |

**Format changes a consumer sees:**
- **Snapshot format v2** (module API stage 1, merged `062e1fa`). The header grows to 40 bytes and carries the configuration identity: the module set, versions and compiled order. A v1 blob is refused with the version message. A blob restores only into a simulation with the same identity (`L2`). KAT passes blobs through without parsing them, so a KAT session ring that holds v1 blobs cannot restore them after the upgrade.
- **Module API stage 3** (merged `42b352a`). The standard module set gains an `environment` module and a `dryden.sample` pass, so its configuration identity changes. A snapshot taken before this merge is refused on restore. The snapshot format itself is unchanged.
- **Design frame** (drone builder, Task B, merged `7532259`). A model type carries its design frame: `design_to_principal` and `com_offset`. `spawn` takes and `vehicle_state` returns poses in the design frame. The default is the identity frame, which changes nothing bit for bit.
- **Snapshot format v3** (drone builder, Task C, merged `7d46b6d`). The header grows to 48 bytes and carries the model registry's identity: each registered model's identity (`vehicles::model_identity`) in registration order. A blob restores only into a Simulation that has registered the same models in the same order. Register them before `restore()`, even in a fresh Simulation. A v2 blob is refused with the version message.
- **Scene file** (drone builder, Task D, merged `43eac56`). A new format: a scene holds a world reference plus placed assets and vehicles. It names its world by `world_hash()`, and composing it against another world is refused (`SCN-001`). Its text is canonical (`SCN-002`). Models register in the scene's `models` order (`SCN-005`). The first golden is `tests/golden/scenes/gate_run.scene.yaml`.
- **The airframe compile** (drone builder, `DBP-44`, merged `58abbf4`). `vehicles/airframe_compile.hpp` turns a part-built `AirframeSpec` (design frame, Kat's part-block units) into a `ModelType` in its principal body frame: `compile_airframe()`, `check_airframe()` (every problem in one list) and `airframe_propulsion_chain()`. A 4-in-1 ESC's `current_total` caps the drive current over its channels (`DBP-26`). Until a list of ESC boards lands, a total needs `channels` equal to the rotor count. The reference quad's `model_identity` is pinned and reproduced on gcc.
- **`ModelType::issues()`** (merged `58abbf4`). The installed `vehicles/model_type.hpp` gains `ModelIssue` and `issues()`, which list every problem with its element, index and field. `validate()` returns the first of them, with unchanged messages. `physics/forces.hpp` gains `DragLawCheck` and `check_drag_law()`. Both doors now run `check_drag_law()`: `ModelType::validate()` (and so `register_model()`) and `Simulation::add_drag_element` refuse a negative or non-finite area or coefficient, which they used to accept. One drag message changed: validate()'s "coeffs and local_pos must be finite" became one message per field (the drag law's own reasons for mode, area and coeffs, and a separate local_pos message).
- **UTF-8 paths in messages** (merged with `core/path-text-utf8`). Error messages name file paths in UTF-8 on every platform (`spade::path_text`, `core/path_text.hpp`). No file loader throws while reporting one: `load_gltf` used to throw `std::system_error` for a path outside the ANSI code page on Windows, and now returns `io_error`.
- **GPU admission and banded parity** (the user's ruling of 2026-10-05; merged `e146575`). Any Vulkan 1.1+ device is admitted, whatever its fp32 denormal support (`CORE-5`); no kernel requests a denormal mode (SPIR-V `P3`). `Simulation::vulkan_device_report()` (and `compute::DeviceReport` / `describe()`) says which device, driver and float settings produced a Vulkan run (`L6`). GPU results are banded against the CPU, not bit-identical (`TD-14`): about 0.01% relative at worst in-horizon, positions within micrometres, and in long or chaotic runs the two paths drift apart as any two float implementations do. Each path is deterministic on its own (CPU across MSVC and gcc; GPU per device and driver), and the CPU is the reference.
- **The IMU feels contact** (`PHY-7`). The accelerometer's specific force includes the contact response: a body resting on the ground reads `+g` up, not about 0, and an impact reads its full reaction, with no range clamp (thousands of m/s² on a hard landing). In-flight readings are unchanged. Every scene with contact has new digests. The reported velocity at rest still reads about `-g·dt` until `PHY-8`.
- **Module API stage 4: modules own their state** (merged `9763b93..`, stage 4 complete). The installed `sim/module.hpp` gains `ArrayDecl`/`Extent`, `StreamDecl`, `ConfigTableDecl`, `VehicleRows`, `ScratchDecl`, `QuantityAccess::optional` and `PassDecl::before`. `Simulation` gains `attach_row`, `module_rows<T>`, `module_array` and `config_table`. `SubstepContext::state` gives passes views of their declared state, and `StepShape` gains `arrays` and `scratch`. The standard `<module>.state` tokens are renamed (e.g. `dryden.dryden`), and `create()` refuses a set that drops or reshapes a built-in array or stream. Snapshot format stays v3; identity, schema hash and goldens are unchanged, so stored blobs still restore.

**What lives in KAT, not here:**
- The sim-host contract **C5** and its adapter (`dronesim/spade/`): the mapping from a drone description to Spade constructs, plus KAT-side timing and stamping.
- KAT's content: worlds, scenes, meshes.
- KAT's training system and editor.

**KAT's rule series** that older Spade text cites. KAT owns them; Spade does not treat them as authority:

| Series | What it is |
|---|---|
| `C1`–`C5` | KAT's platform contracts (C5 = sim host) |
| `HS1`–`HS12`, `F-1`–`F-6` | KAT's sim-host spec |
| `TS*`, `TR*` | KAT's training structures and training system |
| `B*` | KAT's amendment batches |
| `G*` | KAT's configuration architecture |
| `CP*`, `DM*` | KAT's component catalog and demo missions |
| `R1`–`R15` | KAT's errata |
| `CN-*`, `CG*`, `CS*`, `SC*`, `SD*`, `TA*`, `MV*`, `T0`–`T3` | other KAT series |

**Legacy Spade rulings that are about KAT.** Each keeps its ID and is homed here. None binds Spade's design.

| ID | Ruling | Status |
|---|---|---|
| charter `SA1` | Spade is the backend of KAT's M1b milestone | record |
| charter `P5` | A thin adapter above Spade (dronesim implements C5); no KAT types below the seam | superseded by this page and `L7`. The boundary holds from Spade's side as "no domain in the core" |
| `SL16` (KAT half) | KAT-side vocabulary of the 24th spec | record; the Spade half is in `test-docs/00-decisions.md` |
| `interleaving-shuffle.md` (§13.3, open) | A KAT runtime proposal; Spade only supplies `state_digest` | KAT's to decide (`superseded/2026-09-consolidation/interleaving-shuffle.md`) |

**Moved to KAT** (Kat `2a7b63cc`, 2026-10-03): the 30 render-agreement cases and the bookmark check, now against Spade's installed public render API only, with their data owned by Kat. Kat's sim-host tests no longer read Spade's `tests/golden`; nothing in Kat reads a Spade checkout. Spade rebuilds its agreement bands on its own content (`backlog.md`).

**The drone builder** is a joint spec with Kat (`plans/2026-10-03-drone-builder-engine-design.md`). Spade pushes and Kat pulls, by the user's ruling of 2026-10-03.

**Shared build machine** (2026-10-04). Both trees build on one box (32 GB, 16 threads).
- There is no build budget. The user lifted every memory-based limit on 2026-10-04: "building is free, testing is free." That ended slots, shares and the free-memory check. The leads' 2+2 budget from earlier that day is withdrawn. If the box ever starves (a build killed or out of heap), the two leads compare numbers before adding any rule.
- Only Kat writes `build-host/` and `install-host/` in this checkout (Kat's `spade-prefix.ps1`). Spade builds use `build-ninja/` and the worktrees' own directories.

## Other consumers

None yet. A vehicle-dynamics or F1-style consumer would assemble its domain the same way: templates and modules on the public API, with no engine changes for its domain.
