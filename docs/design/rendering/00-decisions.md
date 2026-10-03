# Rendering — decisions

**The only place a Rendering decision's status lives.** One row per ID. **Status** is one of: `live`; `live, KAT-side` (the ruling governs code that now lives in KAT, see `../consumers.md`); `historical` (it governed work that is finished); `superseded by X`; `proposed`. Sources are in `../superseded/2026-09-consolidation/` unless noted. "03" means `03-world-and-render.md`.

## `RS` — the render program (signed as the enumerated 20 on 2026-09-17)

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `RS1` | The renderer is an engine module with a backend seam. S7 splits into S7a (CPU reference) and S7b (Vulkan, HUD, camera seat) | 2026-09-17 | live; "S7a/S7b" superseded by the engine model's techniques | 03 §1 |
| `RS1a` | KAT's interim host raster is deleted, and its host calls the engine renderer | 2026-09-17 | live, KAT-side | 03 §1 |
| `RS2` | A render scene is built once per world and roster; per frame only body poses change | 2026-09-17 | live | 03 §2 |
| `RS3` | Three geometry paths (tessellated unions, CSG meshed at load, glTF) plus ray-march truth; fixed resolution tables | 2026-09-17 | live | 03 §3 |
| `RS4` | The ray-marched image is the collision field; raster silhouettes must agree within measured, pinned bands | 2026-09-17 | live; no bands on Spade's own worlds yet | 03 §4 |
| `RS5` | World schema v2 adds `materials`, `lighting`, `props` (render-only) | 2026-09-17 | live; **split**: Core owns the format, Rendering owns the meaning | 03 §5 |
| `RS6` | Material = RGBA + {lambert, unlit, emissive}; one sun, ambient, sky gradient; no textures, PBR or post-processing | 2026-09-17 | live | 03 §6 |
| `RS6a` | One orthographic sun shadow map, fitted to the scene bounds, baked at load | 2026-09-17 | live | 03 §6 |
| `RS7` | glTF meshes are render-only; the caller resolves asset IDs; a miss draws a fallback marker and warns once | 2026-09-17 | live (the resolver itself is KAT-side) | 03 §7 |
| `RS8` | Kit-of-parts content in three lanes (SDF prefabs, procedural glTF, authored with provenance) | 2026-09-17 | live, KAT-side; Spade's own content is `../backlog.md` | 03 §8 |
| `RS9` | KAT's ten scenes dressed from the kit | 2026-09-17 | live, KAT-side | 03 §9 |
| `RS10a` | A standalone sandbox window | 2026-09-17 | superseded by `SL10`–`SL13` | `06-sandbox-and-v1.md` |
| `RS10b` | Delete the v1 GL tree | **excluded** | superseded by `SL14a` (quarantine, never delete); never ratified | `06-sandbox-and-v1.md` |
| `RS10c` | A lightweight ImGui world/scene builder seat, held open | 2026-09-17 | live; the editor (Interface) takes the seat | `06-sandbox-and-v1.md` |
| `RS11` | Every executable the program touches is registered with its target, path, build and run command | 2026-09-17 | live, KAT-side; Spade has no register | 03 §10 |
| `RS11a` | The register is a tracked document with a test guard | 2026-09-17 | live, KAT-side | 03 §10 |
| `RS12` | Visualisation checkpoints CK-1 to CK-4 | 2026-09-17 | historical; CK-4's "v1 deleted" is dead text | 03 §11 |
| `RS13` | Verification: byte-exact frame goldens, `RS4`, cadence invariance, tessellation and CSG goldens, bench | 2026-09-17 | live (`03-verification.md`); cadence invariance not tested in this repo | 03 §12 |
| `RS14` | The register of amendments the program owes | 2026-09-17 | historical | 03 §13 |
| `RS15` | Out of scope: Vulkan render backend, ImGui HUD and builder, camera sensors, textures/PBR/post-processing, mesh colliders, movers, editor placement, LOD | 2026-09-17 | partly superseded by the engine model (techniques, cameras as sensors); **the post-processing exclusion stands** | 03 §14 |

## `SR` — rulings of the S7a render program

Reconstructed 2026-10-01 from their code citations (`SR-17`, `SR-17a`: 03 §15–16). Each row is the ruling as the code states it. **Signed as reconstructed on 2026-10-02 (user, via lead)**, except `SR-3`. Where citations disagree (below), the signature covers the reconstruction and the disagreement stays recorded.

| ID | Ruling | Signed | Status | Source (enforced by) |
|---|---|---|---|---|
| `SR-2` | The agreement matrix measures bare geometry only; re-measure after dressing worlds | signed 2026-10-02 (user, via lead) | historical (KAT worlds) | agreement test header, removed at `ab2a21e` |
| `SR-3` | Content unknown; listed as orphaned in 2026-09, cited nowhere | **unsigned**: there is no content to sign | unknown | 03 §15.1 |
| `SR-9` | One static draw item per split-program entry (a union leaf or a whole CSG root), in authoring order; `local_to_world` = float inverse of `world_to_local` | signed 2026-10-02 (user, via lead) | live | `scene_from_world`; `SceneFromWorld.*` |
| `SR-11` | Submesh contract: empty submesh arrays mean one submesh at material 0; non-empty arrays are parallel and partition the indices | signed 2026-10-02 (user, via lead) | live | `MeshData`; `RasterCpu.EmptySubmeshArrays…`, `…ExplicitSubmeshes…` |
| `SR-13` | Shaded mode culls back faces; *"wireframe mode does not — both sides draw"* | signed 2026-10-02 (user, via lead) | live | `raster_cpu.hpp`; `RasterCpu.ShadedModeRenders…`, `…WireframeModeDrawsBothWindings…` |
| `SR-14` | No libm transcendental in engine source or golden-feeding test source; trig via `sin32`/`cos32`, `sqrt` allowed | signed 2026-10-02 (user, via lead) | live | `BitPortability.NoLibmTranscendentalInEngineOrGoldenTestSource` |
| `SR-15` | Exact near/far triangle clipping on both mesh and overlay paths, never whole-triangle rejection | signed 2026-10-02 (user, via lead) | live | `clipTriangleNearFar`; `RasterCpu.SpawnMarkerOverlaySurvives…`, `…NearPlaneClipIsGeometricallyExact…` |
| `SR-17` | The analytic infinite ground: standalone planes only, world-space, alongside the tessellated grid, bit-identical seam, unshadowed past the bounds | signed 2026-10-02 (user, via lead) | live, except clause 5 (superseded by `SR-17a`) | 03 §15 |
| `SR-17a` | Every shaded surface blends toward the sky colour by view distance; a pure per-pixel function of three values; exact at strength 0 | scope user-ruled 2026-09-17; signed 2026-10-02 (user, via lead) | live | 03 §16; the `shadowed_ground_with_caster_atmospheric` frame pair |
| `SR-18` | Gouraud: Lambert N·L + ambient via the one `shade_vertex_color`, *"shade at the vertices"*, colour interpolated | signed 2026-10-02 (user, via lead) | live | `shade_vertex_color`; `RenderShading.LambertFaceTowardSun…` |
| `SR-21` | Overlays get a depth bias toward the camera so a coincident ground mesh does not win the z-tie | signed 2026-10-02 (user, via lead) | live | `kOverlayDepthBias`; `OverlayDepthBias.*` |
| `SR-22` | The analytic ground draws in shaded mode only; the sky draws in every mode | signed 2026-10-02 (user, via lead) | live | `draw_sky_and_ground_background`; `RenderShading.WireframeModeNeverDrawsTheAnalyticGround…` |
| `SR-23` | The sky gradient is keyed on ray elevation, not screen row; one shared `sky_gradient_color` | signed 2026-10-02 (user, via lead) | live | `RenderShading.SkyColourAtTheTrueHorizonMatches…RulingSR23` |
| `SR-24` | Shadows sample a perspective-correct world position, never a screen-affine one | signed 2026-10-02 (user, via lead) | live | `sample_shadow`; `ShadowPerspectiveCorrectness.*` |
| `SR-25` | A shadow attenuates the sun term only; ambient is never shadowed | signed 2026-10-02 (user, via lead) | live | `ShadedColor`; `ShadowStep1.*` |
| `SR-27` | Ray-march takes each hit's material from its owning leaf, not material 0 | signed 2026-10-02 (user, via lead) | live | `RaymarchSmoke.MaterialMisresolution…`; `AgreementGuard.Step1c*` |
| `SR-28` | The ray-march surface epsilon is `1e-4`, so the reference's own error is negligible | signed 2026-10-02 (user, via lead) | live | `kRaymarchSurfaceEpsilon`; `RaymarchSmoke.SphereAnalyticSilhouette…` |
| `SR-30` | Every agreement band proves its own detection surface, via the bare-ground probe, re-checked live; bands are per world and camera | signed 2026-10-02 (user, via lead) | live as a rule; no matrix enforces it until bands return | `03-verification.md`; `strip_to_ground_plane_only`, `AgreementProbe.*` |
| `SR-31` | Agreement bands are versioned; a re-measure adds a version and never edits one | signed 2026-10-02 (user, via lead) | live as a rule; no bands file exists since `ab2a21e` | `03-verification.md` |
| `SR-33`, `SR-35` | Default material 0.8 grey, not white; default sun off-axis `(0.4, 0.8, 0.6)`, not overhead. Never cited apart | signed 2026-10-02 (user, via lead) | live | `MaterialDesc`, `LightingDesc`; `RenderShading.DefaultMaterialUnderWorstCase…` |
| `SR-34`, `SR-37` | "Standard resolution" (Task VQ-B). The resolution change is not in this tree; only a byte-exact perf hoist cites them | signed 2026-10-02 (user, via lead) | live, KAT-side (probably) | `BackgroundRayBasis` comment |
| `SR-42` | *"C0 cannot merge without C4"*: a KAT-era merge-order rule, discharged by a user ruling | signed 2026-10-02 (user, via lead) | historical | `06-sandbox-and-v1.md` (`SL17`) |
| `SR-54` | KAT Task C4 placed render-only props in nine KAT worlds | signed 2026-10-02 (user, via lead) | historical (KAT content) | — |
| `SR-57` | A dimension note on a KAT gate mesh | signed 2026-10-02 (user, via lead) | historical (KAT content); quoted only in `tests/fixtures/render/gate-ring.gltf` | — |

Also on the 2026-09 orphan list but cited nowhere in this tree, and probably KAT-side: `SR-10`, `SR-38`–`SR-41`, `SR-52`, `SR-58`.

**Where citations disagree** (signed as reconstructed; fix the comments when the code is touched):
- `SR-23`: dated to Task R6 in `scene.hpp`, to VQ-A elsewhere. A `raymarch.hpp` note still describes the old per-row gradient.
- `SR-25`: also cited, in `agreement.hpp`, for "ray-march never casts shadows", which `raymarch.hpp` credits to a different amendment.
- `SR-9`: the original mapping and its revision share the number. Three placeholder comments are from before the revision.
- `SR-14`: cited both for the scan widening and for the older no-libm rule as a whole.

## `PA` — plan amendments of the S7a render plan

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `PA-1` | A render target never owns its pixel memory; the caller does | — | live | `render/target.hpp` |
| `PA-2` | Per-node materials are a parallel host-only array, not the SDF node's padding word | — | live | `world/sdf.hpp`, 03 §5 |
| `PA-4` | Ground grid, bounds box and spawn markers are presentation overlays behind `RenderOptions::overlays` | — | live | `raster_cpu.hpp` |
| `PA-5` | Unbounded primitives (plane, heightfield) tessellate as a bounded grid fitted to world bounds | — | live | `tessellate.hpp` |

## Charter pillar and engine addendum

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `P6` | Offscreen render boundary: Spade renders to an offscreen image that a host composites; headless is simply not rendering; stepping never depends on render state | approved 2026-08-06 (KAT-era charter) | superseded by `L5` (stepping never depends on rendering) and the camera model (`01-techniques-and-channels.md`). The offscreen, caller-owned image lives on as `PA-1` | `01-charter.md` §2 |
| `A2` | A frame pool hands out frames that stay valid across host calls; slots are deferred, never recycled, while a frame is outstanding; at least two can be outstanding | 2026-08-10 | live, with nothing in the engine to bind yet. It binds the engine frame pool the camera sensor needs (`01-techniques-and-channels.md`, camera). Today only KAT's host-side pools meet it (`../consumers.md`) | `02-engine.md` §9 (presentation frames) |
| `A5` | The renderer may run on a second thread against the frame pool only. Degradation order: renders drop first; sensor synthesis inside stepping is never shed | 2026-08-10 | live as a rule. No engine frame pool exists yet; `render()` reads only its arguments, so any thread may call it with its own inputs | `02-engine.md` §9, §10 (threading contract) |
| `A6` | Camera cost model: rendered camera cost ≈ drones × W×H × per-pixel technique cost per sample. A camera sensor is benched against its rate envelope when it is built; a miss triggers design review | 2026-08-10 | live; applies when a camera-sensor technique is built (`01-techniques-and-channels.md`, camera). The original assumed a ray-marched camera; the cost term is now per technique | `02-engine.md` §8 |
| `A8` | The 25 Hz camera lane is host-synthesised analytic features, not pixels; rendered cameras come later | user-ruled 2026-08-10 | superseded by the engine model (a camera is a technique plus channels, and can be a sensor) | `02-engine.md` §8 |

## `RND` — new rulings

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `RND-1` | **One sun convention.** `sun_direction` points from the scene toward the sun; every path shades with `dot(n, +sun_direction)` | signed 2026-10-02 (user, via lead) | live; in the code since `94deae3` | restructure defect 4; `02-scene-and-appearance.md` |
| `RND-2` | **A partial technique carries a partial name.** A backend that cannot draw a full frame exposes an entry point named for what it does draw (`render_background()`), never `render()` | signed 2026-10-02 (user, via lead) | live; `render/raster_gpu.hpp` already follows it | `01-techniques-and-channels.md` |
| `RND-3` | **Grades:** CPU raster reference (fp64, declared); CPU ray-march reference for coverage; OpenGL best-effort; Vulkan raster banded once colour is guarded | signed 2026-10-02 (user, via lead) | live as the declared grades; no grade API exists in code yet (`../01-engine-model.md`) | `01-techniques-and-channels.md` |
| `RND-4` | **The Vulkan raster resumes after field channels and Core's module API**, as the headless and camera-sensor technique. OpenGL stays the editor's interactive path; retiring it is reconsidered once Vulkan draws full frames | signed 2026-10-02 (user) | live | user ruling 2026-10-02 (`../backlog.md`) |
| `RND-5` | **`render::Lighting{}`'s default sun is flipped above the horizon**, with one deliberate regeneration of the four frame goldens that shade under it, once the Docker gcc leg (`TD-12`) can cross-check the regeneration | signed 2026-10-02 (user) | **done** at `a46bb86`: the default is `normalize(LightingDesc{}.sun_direction)`; four goldens regenerated and reproduced by the Docker gcc leg | user ruling 2026-10-02 (`../backlog.md`); `plans/2026-10-03-default-sun-plan.md` |
| `RND-6` | **The raster draws a heightfield only within the world bounds**, as `PA-5` grids it, and `RS4` holds there. Past the bounds the SDF, and collision, have terrain that the raster does not draw. An analytic heightfield background, which would amend `SR-17`, is a separate decision (backlog) | **proposed** 2026-10-03 (Rendering; the lead puts it to the user), **unsigned** | not in force | `plans/2026-10-03-raster-defects-plan.md` (A2); `07-status.md` debt |
