# Spade 03 -- World Format, Render Scene, and Rendering

✅ **Status: SIGNED 2026-09-17.** The `RS` series was drafted 2026-08-24 and carried the note below
for three weeks. Its program (engine stage **S7a**) landed on master ff-only at `9778260`, pushed and
CI-green, long before the signature -- *the code shipped, the spec did not get its signature*, and
both were true at once. **The signature covers the ENUMERATED 20**, not the range: `RS1`, `RS1a`,
`RS2`-`RS9`, `RS10a`, `RS10c`, `RS11`, `RS11a`, `RS12`-`RS15`. ⛔ **`RS10b` is EXCLUDED -- recorded
as superseded by `SL14a`, NOT ratified.** Provenance and the full record: `07-status.md` s.3.1.

**What this is.** The single subject document for how a Spade world is described on disk, how
that world becomes a render scene, and how that scene becomes pixels.

**What it harvests.**

- `superseded/kat-spade-render-scene.html` (twenty-first spec, DRAFT 2026-08-24) -- sections 1-9 and 11-15.
  Section 10 (`RS10a`/`RS10b`/`RS10c` -- the sandbox, the v1 GL retirement, the ImGui builder
  seat) is deliberately **not** harvested here; document 06 owns it. `RS10a`-`RS10c` are
  referenced by identifier where a ruling in this document depends on them, and nowhere
  restated.
- `superseded/kat-spade-engine-design.html` -- section 8 (Worlds and the file format), the frame-pool
  paragraph of section 9, and section 10 (Rendering (S7) -- designed now, built later). These
  are the *approved* baseline that the `RS` series amends.

**Series owned:** `RS1`, `RS1a`, `RS2`, `RS3`, `RS4`, `RS5`, `RS6`, `RS6a`, `RS7`, `RS8`, `RS9`,
`RS11`, `RS11a`, `RS12`, `RS13`, `RS14`, `RS15`.

**Not covered here** (other documents own them): the engine core and substep schedule (02), the
object model (04), the C5 host seam (05), the sandbox and the v1 retirement (06), phase status
(07).

---

## 0. The starting position

Nothing in Kat rendered geometry before this program. Both C5 sim hosts presented through a CPU
*wireframe* rasterizer -- ground grid, a derived bounds box, one wireframe proxy per SDF node
with CSG operators skipped entirely, spawn diamonds, drone tetrahedra. That was the whole visual
vocabulary. The eight shipped glTF meshes and `CS7`'s `visual_resolver` existed, were correct,
and had **zero consumers**: the raster hook was parked as a documented S7 seam. Spade's
`engine/` tree had no `render/` module at all; the only thing in the repo that drew a shaded
pixel was the frozen v1 OpenGL engine behind `spade_viewer`, which `D10` scheduled for death
*at* S7.

This spec is that S7, scoped to what makes a scene **designable** rather than what makes it
photographic.

**Session rulings absorbed (user, 2026-08-24).** (1) The renderer is **S7, CPU reference first**
-- a real engine module with a backend seam, not a grown host-side raster; Vulkan lands behind
the same frame pool as S7b. (2) Appearance lives in the **world file**: schema v1's freeze is
lifted to v2. (3) Content is **kit-of-parts first**, then all ten scenes dressed from it. (4)
Assets are a **hybrid with a declared line** -- SDF prefabs for structure, procedural glTF for
parametric geometry, an admitted authored-asset lane gated on provenance and licence. (5)
Authoring is **tool-side; the editor views**. (6) The program includes a **standalone sandbox
window and the retirement of the v1 GL engine** (document 06). (7) Long-term, Spade-as-a-library
gets a **lightweight ImGui world/scene builder** -- a design seat this program's shapes must not
foreclose (document 06).

---

## 1. `RS1` -- Placement, module shape, and the S7 split

**`RS1`** puts the renderer inside the engine as a module with a backend seam, structured like
`compute/`: one interface, backends beneath it.

| File | Contents |
| --- | --- |
| `render/scene.hpp` | `RenderScene` -- the renderer's own view of a world (draw items, material palette, lights, resolved meshes). Built from a `WorldDesc` plus a tick-boundary body-pose copy. Immutable once built; cheap to rebuild. |
| `render/raster_cpu.{hpp,cpp}` | The CPU reference rasterizer (S7a). |
| `render/target.hpp` | The render target -- a caller-owned pixel span plus width/height/stride/format. |
| `render/vulkan/` | S7b, behind the same pool. Not built here. |

**S7a adds no engine-side frame pool.** `dronesim/spade/frames.h` already implements a
`B10`-conformant `FramePool` that the C5 suite proves; the engine writes into a slot that pool
hands it. `B10` is a C5-boundary contract and stays where C5 lives -- duplicating it engine-side
would create two pools and one of them would be the untested one. `MV5`'s "slots map 1:1 onto
pool targets" is unaffected: slots map onto host pool slots.

### The split itself

**S7 splits in two.** This is the ruling's core, and it amends an approved document, so state it
plainly:

- **S7a** (this program) -- scene model, CPU rasterizer, frame pool, sandbox, v1 retirement.
- **S7b** (later) -- Vulkan backend, ImGui debug HUD, camera-sensor seat.

The reasoning is that "renderer" was one word covering two jobs with different risk profiles.
Making a scene *designable* -- shaded geometry, materials, a sun, shadows, a kit of parts --
needs no GPU at all, and a CPU path buys a property the GPU path can never buy back:
**byte-exact frame goldens** (`RS13`). Making the renderer *fast* needs Vulkan and buys nothing
for scene design. Shipping them as one stage would have gated all the content work behind the
graphics-API work and thrown away exact goldens for both halves. So the CPU reference lands
first, behind a seam, and Vulkan lands behind the same seam later.

`RS1` further reassigns S7's original exit proof -- "GL sandbox retired; headless/rendered
equivalence green" -- to **S7a**, with S7b inheriting the Vulkan half of engine-design section
10.

> **OPERATIVE (`C4`):** `RS1` governs, because it describes what shipped. The approved engine text
> names a Vulkan renderer as v1's replacement; S7a shipped a CPU rasterizer and there is no Vulkan
> render backend in the tree (verified 2026-09-07). The approved page is out of date, not in
> dispute. What remains is paperwork: `RS14` books the `D10` amendment as owed rather than made.

> **OPERATIVE, with a bookkeeping gap (`C10`):** the approved engine text says the render module
> exposes a `FramePool` with `render(world, camera, target)`. **`RS1` governs -- it describes what
> shipped.** Verified 2026-09-07: there is no frame pool anywhere in `spade/engine/render/`; the
> only `FramePool` in the tree is a compute-side concept in `engine/compute/vulkan/context.hpp`,
> which is a different thing. The obligation is met at the C5 seam, as `RS1` says.
>
> **The real defect is the bookkeeping.** `RS14` claims to enumerate every amendment this program
> owes -- it lists engine s.13, s.8/`D7` and `D10`, and **omits s.9**. A register that claims
> completeness and is short by one row is the same shape as `C11`. Fix it before sign-off.
>
> ✅ **FIXED 2026-09-17, before the signature, and it was short by TWO not one.** s.9 now has its
> row in section 13, and so does the 24th spec -- which this table had never named although
> `SL14a` supersedes `RS10b`. **A register found short by one row is worth re-counting rather
> than patching**: the second gap was invisible from the side that found the first, because
> `C10` was looking outward at what this program owes and the `SL` gap points inward at what is
> owed *to* it.
>
> ⚠⚠ **AND `FramePool` NAMES FOUR DIFFERENT OBJECTS, which is why this row was easy to lose.**
> (1) The one **s.9 specifies** in the render module -- never built. (2) The one
> `engine/compute/vulkan/context.hpp` names as **deferred** to S7/Addendum `A8` -- a compute-side
> future, explicitly not present. (3) `kat::hostspade::FramePool` (`dronesim/spade/frames.h:71`) and
> (4) `kat::hostfake::FramePool` (`dronesim/fake/frames.h:55`) -- **both exist and ship**, the
> second a straight port of the first, same slot shape and refcount mechanism, differing only in
> namespace.
>
> ⚠ **This entry said THREE until 2026-09-17, and the missing one was a SHIPPING class**, not an
> obscure one -- I counted the pool I had just measured and not its twin one directory across.
> Runtime swept `runtime/`, `editor/`, `sdk/` and `components/` to bound it: **four is the whole
> population.** *A count of things sharing a name is itself a thing that can be wrong about which
> objects it counted.* A reader who measures (1) or
> (2) and reports "there is no FramePool" is right about their object and wrong about the tree;
> a reader who finds (3) and reports "the FramePool exists" is right about theirs and wrong about
> s.9. **Both readings are available, both feel conclusive, and the editor's camera panels depend
> on (3) while this row is about (1).** Name the object, never the word.

> **STALE:** engine-design section 10 is titled *"Rendering (S7) -- designed now, built
> later."* Half of it is now built and merged. The title is accurate only for the S7b
> residue (instanced mesh pipeline over v1's flatten + `instanceStartIndex` scheme,
> colour/velocity/wireframe debug modes, the Dear ImGui debug HUD). The raymarch bullet is
> not merely built -- `RS4` promoted it from a view mode into a gate.

### Constraints held, not renegotiated

| Constraint | How this design satisfies it |
| --- | --- |
| `HS2 R1` -- Spade stays kat-free | The render module gains no kat dependency. Mesh *resolution* (id -> path) stays kat-side in `CS7`'s resolver; the engine is handed loaded geometry, never an id it must interpret. |
| `TA1` / `MN-14` -- editor owns no graphics context | Frames remain opaque BGRX8 across C5. The editor is unchanged by this program, exactly as it was unchanged by M1b. |
| `TA1` -- render-cadence invariance | Holds **by construction**: the renderer is a pure function of (RenderScene, camera, target) and touches no sim state, no RNG stream and no clock. Calling it zero times, once, or every tick cannot perturb stepping. |
| `A5` / `B11` -- degradation order | Renders drop first; sensor synthesis inside stepping is never shed. Unchanged. |
| `MV5` -- camera slots | The entry point is per-call-camera (`render(scene, camera, target)`) as the viewport spec already assumes, so Phase G's slots map 1:1 onto pool targets and are a **no-op against this renderer**. |
| `D3` / `DM8` / `CS7` -- SDF is collision truth | No mesh collider is introduced. Meshes are presentation only; `RS4` makes the agreement between them a test rather than a hope. |

### `RS1a` -- the interim raster is retired, not amended

**`RS1a`.** `dronesim/spade/raster.cpp` and `raster.h` are **deleted**; `kathost_render` calls
the engine renderer instead.

This is worth naming for what it is: **the one clean three-document handoff in the whole
estate.** `HS5` wrote its own endpoint when it was authored -- "explicitly interim,
D10-strangler style, retired when S7 reaches parity behind the same
`kathost_render`/`frame_acquire` calls." S7a reached that parity behind exactly those calls. So
`RS1a` *discharges* `HS5` as written and needs no amendment to it. A predecessor spec named its
own successor's condition, the successor met the condition verbatim, and nothing had to be
renegotiated. `RS14` records `HS5` in the amendments table precisely to say **no amendment is
owed**.

`dronesim/fake/raster.cpp` is **kept unchanged**: the fake is the kat-free conformance reference
and the editor's deterministic test host, and must never acquire a Spade dependency. The two
hosts' pixels differ; C5 conformance is about semantics, not pixels.

---

## 2. `RS2` -- The render scene model

**`RS2`.** A `RenderScene` is built once per (world, roster) and refreshed per frame only with
body poses. It carries four kinds of draw item, a material palette, and a light rig.

| Draw item | Source | Rebuild cost |
| --- | --- | --- |
| Tessellated SDF primitive | Each non-op node of a union subtree (`RS3`) | World load |
| Meshed CSG subtree | Each subtract / intersect / smooth_union subtree (`RS3`) | World load |
| Static prop mesh | Schema v2 `props:` entries (`RS5`) via the `CS7` resolver | World load |
| Body mesh | `ModelType::visual_ref`, instanced per live body | Per frame: transform only |

Only the last is per-frame work, and only its transform changes -- geometry is never re-uploaded
or re-tessellated to move a drone. This is the property that lets the same scene serve the
editor viewport, the sandbox, a frame-dump batch, and (at S7b) a camera sensor without three
code paths.

The world-builder rebuild path is kept cheap and explicit for the ImGui-builder seat (`RS10c`,
document 06): rebuilding a `WorldDesc` and swapping the `RenderScene` must be a supported,
tested operation, not a process restart.

---

## 3. `RS3` -- How a world becomes pixels

The shipped content is **88 unions and 2 subtracts** across ten worlds. That distribution --
overwhelmingly unions, but with a real subtract that matters visually (`gate-square` is `box -
box`: the hole you fly through) -- sets the algorithm. The design follows the measured content
rather than the general case.

**`RS3`** declares three geometry paths and one truth path.

**(a) Union subtrees -> tessellated primitives.** Each primitive kind (plane, box, sphere,
cylinder, capsule, torus, heightfield) has an exact parametric tessellation at a declared
per-kind resolution. **A union is free**: draw both operands and the depth buffer resolves the
union exactly. No approximation, no CSG evaluation, no cost. This covers 88 of the 90 operators
in shipped content.

**(b) CSG subtrees -> meshed once at world load.** A `subtract`, `intersect` or `smooth_union`
subtree is meshed by surface nets over the subtree's AABB, deterministically, and cached in the
`RenderScene`. The cost is paid at load, never per frame. **Resolution is a renderer-side
constant table**, not a world-file field and not an adaptive heuristic -- the same discipline as
(a)'s tessellation table, so worlds stay portable across renderer versions and emission is
golden-testable byte-identical for a given (subtree, table version).

**(c) glTF meshes** -- bodies via `ModelType::visual_ref`, static props via schema v2. Resolved
through `CS7`'s `visual_resolver`, which finally acquires a consumer. Miss and malformed refs
render the fallback marker and raise exactly one warning, per `CS7`'s declared behaviour --
never a blank, never a fault.

**(d) Exact raymarch mode** -- sphere-tracing of the real `SdfProgram`, evaluating the identical
field the collision passes evaluate. Too slow on CPU to be the default presentation path, and
that is fine: **its job is truth, not throughput** (`RS4`). Selectable per render call, used at
reduced resolution.

**Declared simplifications**, each written into the code and the docs rather than discovered
later: an infinite `plane` tessellates as a bounded grid fitted to world bounds;
`smooth_union`'s blend is honoured by path (b) rather than approximated as a plain union;
tessellation resolution is a fixed table, not an adaptive heuristic, so output stays
byte-reproducible.

---

## 4. `RS4` -- The visual/physics agreement test

**`RS4`: the raymarched image is the collision field.** Path (d) evaluates the same SDF program
the `CollisionStatic` pass evaluates. So for every shipped world, at every shipped camera
bookmark, the build renders **both ways** -- fast path and raymarched path -- and asserts
silhouette agreement within a declared per-world tolerance. A gate whose tessellation or CSG
meshing does not match its SDF **fails a test** instead of surprising a pilot.

This is the engine design's *"the debug view draws the exact geometry collision sees; visual
world and physics world cannot disagree"* (section 10) turned from a property of one view mode
into a mechanised gate over all of them. It is cheap because it runs offline at reduced
resolution, and it is the direct answer to the structural hazard `DM8`/`CS7` create by ruling
meshes render-only: the moment scenes look good, a visual wall that isn't there becomes a real
failure mode.

Tolerance is measured then pinned with provenance, per the standing execution-record discipline
-- no invented bands. The comparison is silhouette/coverage-based, not per-pixel colour, because
shading legitimately differs between the two paths.

---

## 5. `RS5` -- World schema v1 -> v2

> **The fake host TOLERATES v2; it does not support it.** Verified 2026-09-08:
> `dronesim/fake/world.h:203-204` sets `kWorldFileMinReadVersion = 1` / `kWorldFileMaxReadVersion = 2`,
> with a named test (`AcceptsWorldVersion2StructurallyIgnoringNewFields`). But the comment above
> those constants is the operative part -- v2 is **"accepted structurally, v2's own new fields never
> parsed."**
>
> `RS5` lifted the schema to v2 precisely to add **materials, lighting and props**, which is exactly
> what the fake ignores. So a v2 world loads through the fake and renders **without its v2 content,
> silently**. "The fake loads v2" is true and misleading; the accurate form is "accepts v2
> structurally, ignores its new fields."
>
> All six `conformance_c5` fixtures are still `world_version: 1` -- three in `fixtures/`, three in
> `fixtures_spade/` -- so nothing in the conformance corpus exercises v2 on either leg.


### Who owns the format

The world file is **Spade's**, owned by Spade. The configuration spec's **`G2`** ruling settles
this: the world format leaves `editor/schemas/`, the engine owns it per engine-design section 8,
C5 versions it as a companion via the `world_version` field, and the editor authors and consumes
it while owning neither. `G2` ratifies an owner rather than choosing one. An earlier
configuration draft placed a `world.schema.json` under `dronesim/` as canonical JSON; that was
rejected because it would have created a second world definition beside `WorldDesc`. **There is
deliberately no world schema on the Kat side**, and adding one would violate the configuration
spec's own thesis in its own catalogue.

### The approved baseline (engine design, section 8)

- **`WorldBuilder`** (fluent API): add SDF nodes (primitive, transform, material, CSG op), spawn
  points (named poses), environment (gravity vector, medium params, default seed), capacities.
  `build()` validates (capacity sanity, SDF program depth, name uniqueness) -> immutable
  `WorldDesc`.
- **World file:** versioned YAML serializing exactly the builder's product -- `world_version`,
  environment, SDF scene tree, spawns, capacities, visual attachments (mesh refs, render-only).
  Load = parse -> same validation -> `WorldDesc`. C5's `world_ref` = a file path or an in-memory
  `WorldDesc` handle.
- The golden-world test corpus is world files + input scripts + expected snapshots: **data, not
  code**.
- `D7` deferred glTF import as "a later, render-only add-on"; physics never reads imported
  meshes until triangle-mesh colliders land (`D3`'s later rung). `RS7` discharges that deferral.

### The lift

**`RS5`: the v1 freeze is lifted, once, with a shape that admits `DM3` additively.** World
schema v1 has been **user-frozen since 2026-08-12** -- strict unknown-keys, a `schema_mismatch`
version gate, and a uniform op-node shape, all user-ratified, with evolution by version bump
only. This program lifts it to **v2** to carry appearance: three additions and one zero-cost
field change.

**Per-node material -- a parallel host-only array, *not* `SdfNode._pad`.** `SdfProgram` gains
`std::vector<uint32_t> node_materials`, same length as `nodes`, default 0, carried as its own
world-file section.

The draft's "repurpose the spare pad word" idea was checked and **rejected during planning**,
and the argument matters more than the conclusion: `_pad` is single-sourced into
`compute::SdfNodeRow`, `layouts.slang` and generated layout asserts that index it *by name*, and
`world_to_yaml` deliberately **refuses** a node with non-zero padding ("non-zero padding" --
schema v1 cannot represent it, and `test_world_file.cpp` pins this). A parallel array leaves the
device image, both writer guards, the Slang layout and every generated assert untouched -- and
is the more honest split anyway: **materials are a render concern, the SDF program is a physics
artifact.** `_pad` stays 0 and remains available to S7b if materials ever need to reach the GPU.

- **`materials:`** -- the palette: name, base colour, and a small shading-flag set (`RS6`).
  Index 0 is a defined default so a v1 file upgrades without authoring.
- **`lighting:`** -- sun direction and colour, ambient, sky gradient (`RS6`).
- **`props:`** -- collision-free visual instances: `{ mesh_ref, transform, material }`. This is
  the home for dressing that must not affect physics, and it is the one place a "visual-only"
  object may exist.

`visual_refs:` (the flat opaque string list) is **retained** -- v1 files carry it and it
round-trips -- with its role narrowed to world-level references that are not per-node or
per-prop. Nothing that exists today stops working.

### Migration and the round-trip contract

The world file's constitutional rules stand: every key required, an unknown key is an error at
every level, and `world_version` is the one upgrade path. A v1 file therefore continues to load
-- into a v2 `WorldDesc` with the default material, default lighting and no props -- and
`world_to_yaml` writes v2. `CS8`'s byte-exact round-trip discipline extends to every new field,
charconv-only floats included, and the ten shipped worlds are migrated in place with their v1
forms provable from git.

**Coordination -- `DM3`'s world schema v2.** The demo program's mover/Target/ kinematic block is
*also* waiting on a v2. This spec does **not** build movers, and deliberately shapes v2 so
`DM3`'s block is a purely additive later change (new top-level keys plus a body flag; no
restructuring of what v2 introduces here). Whether `DM3` folds into this bump or takes its own
is a decision for the demo program's engine addendum, filed as a coordination row -- but the
shape it will need is not foreclosed here.

---

## 6. `RS6` / `RS6a` -- Material and light model

**`RS6`.** Low-poly-appropriate and deliberately small. A material is a base colour -- **RGBA**,
the alpha carrying the one admitted transparency case -- plus a shading flag from a closed set
(`unlit`, `lambert`, `emissive`). Transparency exists for envelope and marker geometry only (the
competition-700 mesh already ships a translucent collision-envelope disc) and is drawn in a
declared, order-stable pass so it cannot make output non-reproducible. No textures, no PBR, no
post-processing.

The light rig is one directional sun (direction, colour, intensity), hemispheric ambient, and a
sky gradient rendered as the frame's background so the horizon reads.

**`RS6a`: shadows ship in S7a.** A single sun shadow map -- orthographic, fitted to the scene
bounds, one declared resolution -- is in scope for the CPU path. This is not polish: without
shadows a viewer cannot judge where a drone sits relative to a gate or the ground, which is
precisely the job "enough to properly design the scene" names. It is affordable on CPU because
the shadow pass is depth-only over the same static geometry, and static geometry's shadow map is
rebuilt at world load, not per frame.

---

## 7. `RS7` -- Meshes and the resolver's first consumer

**`RS7`.** `CS7`'s contract is unchanged and finally exercised: `mesh:<family>/<name>` resolves
to `<content_root>/meshes/<family>/<name>.gltf`, with path-traversal guards, a mixed-namespace
filter (`material:` entries are not malformed mesh refs), and miss/malformed both rendering the
fallback marker plus exactly one warning.

Loading is glTF 2.0, render-only, positions and indices and per-primitive material association
-- the subset `tools/gen_meshes.py` already emits. Kat-side resolution hands the engine loaded
geometry; the engine never learns what an id is (`HS2 R1`). This discharges `D7`'s "glTF visual
imports later, render-only".

**The fallback marker becomes visible.** Until now a resolver miss had no renderer to fall back
*in*. `RS7` makes the marker a real, recognisable object -- deliberately conspicuous, never a
blank space -- so a missing asset is obvious in the viewport and not just in a log line.

---

## 8. `RS8` -- The kit-of-parts

**`RS8`: three lanes with a declared line between them.**

**Lane 1 -- SDF prefabs (structure + collision).** The primary lane. Today's nine prefabs grow
into a real vocabulary; every entry is a parameterised SDF fragment expanded at authoring time
into plain schema-v2 nodes, so worlds stay self-contained and the engine never learns the word
"prefab" (`CS4`, unchanged). Because expansion now emits material assignment alongside geometry,
**collision and appearance are authored in one act and cannot drift.**

**Lane 2 -- procedural glTF (parametric, technical).** `tools/gen_meshes.py` grows from eight
hard-coded builders into a parametric library. Deterministic output, sha-pinned goldens, the
`.gitattributes` CRLF guard retained. This lane owns the drone, rotor geometry, gate rings,
markers -- anything whose dimensions must cite a descriptor.

**Lane 3 -- authored assets (admitted, gated).** Hand-modelled or licensed geometry *may* be
committed, but only carrying provenance and licence metadata in the asset's `extras.kat` block,
verified by a content test. The lane may well stay empty through S7a; the point is that **the
rule is written before someone needs it, so it cannot drift in silently**. Licence provenance is
a product-shipping concern under the charter's export posture, not a formality.

**Roster direction** (final list settled in the implementation plan, sized to the ten scenes'
actual needs): gate variants beyond the two shipped, pylons and masts, fences and barriers,
containers and crates, building and hangar volumes, towers, ramps and platforms, landing and
delivery pads, terrain treatments beyond flat and heightfield, and a shared **material palette
shipped as content** so every scene reads as one product rather than ten unrelated demos.

---

## 9. `RS9` -- The ten scenes, dressed

**`RS9`.** `CS3`'s roster is unchanged -- the same ten scenes, the same families, the same spawn
and sim-physics content. Each gains material assignment, a lighting and sky setup, ground
treatment, and composition from the `RS8` kit: the racing family reads as a venue, the demo
family as a site. Existing camera bookmark sets are preserved (`CS5`), because they are what the
gallery and the Camera panels initialise from.

Every dressed world still round-trips byte-exact and still validates; **dressing is content, not
a new mechanism.** `hover-pad` stays the minimal smoke scene and is dressed only enough to
remain recognisable as the smallest possible world.

Static-only, per `CS3`'s `DP-G`: movers and Targets are not introduced here, and the demo family
upgrades in place when `DM3`'s addendum lands.

---

## 10. `RS11` / `RS11a` -- The executable register

**Why this section exists.** The M-2 smoke failed twice for reasons that were entirely about
executables rather than physics: a host DLL that silently lacked three exports the editor's
all-or-nothing loader required, and a path the user could not connect. Nothing in the repo
documented where `kat-editor.exe` or `kat_frame_dump` land, and `kat_content_author` is
`EXCLUDE_FROM_ALL` -- it does not build unless named explicitly. **`RS11`** therefore registers
every executable this program builds, changes or retires with its target name, output path,
build command and run command, and requires the register be mechanically verified.

**`RS11a`: the register is a tracked, guarded document.** The implementation ships
`docs/dev/executables.md` -- one table covering every runnable artifact in the repo (not only
this program's), with target name, owning CMake list, build command, resolved output path and
one-line purpose. A tools-tier test walks that table and asserts each named target exists in the
build graph and lands at the named path after the declared configure+build, in the same class as
the existing `test_c5_export_surface.py` export guard and `test_openable_packages.py` layout
guard. **A register that drifts is a failing test, not a stale page.**

### Spade-side executables

Spade sets `CMAKE_RUNTIME_OUTPUT_DIRECTORY = ${CMAKE_BINARY_DIR}/bin`
(`spade/CMakeLists.txt:63`), and its configure presets bind
`msvc-ninja-debug`/`msvc-ninja-release` to `spade/build-ninja/<preset>`. So every Spade
executable resolves to `spade/build-ninja/<preset>/bin/<target>.exe`.

> **NOT THE CURRENT STATE -- verified 2026-09-07.** The three rows below describe the end of the
> *whole* RS program, including its Phase 4 (`RS10`, sandbox + v1 retirement). **Phase 4 was
> absorbed into the 24th spec and has not been built.** There is no `SPADE_BUILD_SANDBOX` option
> and no `spade_sandbox` target anywhere in `spade/CMakeLists.txt`; `spade/engine/tools/viewer/`
> still holds `bridge.cpp`, `bridge.hpp`, `main.cpp`, `scenes.cpp`, and nothing is retired.
> S7a shipped the render half only. For what exists today see `07-status.md`; for why the
> retirement has not run see `06-sandbox-and-v1.md`.

| Target | State at the END of the RS program (**planned, not current**) | Output path | Run |
| --- | --- | --- | --- |
| `spade_sandbox` | **PLANNED, does not exist** -- the windowed scene viewer (`RS10a`, doc 06) | `spade/build-ninja/<preset>/bin/spade_sandbox.exe` | `spade\scripts\demo.ps1 -Scene <name> [-Preset release] [-Backend cpu\|vulkan]` -- `-Backend` selects the **compute** backend (S6 physics), not the renderer; the renderer is CPU throughout S7a |
| `spade_viewer` | **STILL PRESENT.** Planned retirement -- scenes ported to `spade_sandbox` first (`RS10b`, doc 06) | -- | -- |
| `Sandbox` | **STILL PRESENT.** Planned retirement with v1 -- its three scenes ported first (`RS10b`, doc 06) | -- | -- |
| `spade_tests` | unchanged; gains the render suites | `spade/build-ninja/<preset>/bin/spade_tests.exe` | `spade\scripts\test.ps1` -- never invoke the .exe directly (AGENTS.md) |
| `spade_bench` | unchanged; gains render-pass baselines | `spade/build-ninja/<preset>/bin/spade_bench.exe` | no wrapper script exists today -- `RS11a`'s guard pins the real invocation rather than this spec guessing it |

**Build:** `spade\scripts\build.ps1 -Preset release` (foreground only -- backgrounded builds are
killed on this box). `spade\scripts\demo.ps1`'s scene map is updated in the same wave that ports
the scenes, so `-Scene` keeps naming every scene that exists, v1-derived ones included.

### Kat-side executables

The kat root sets no unified runtime output directory, so targets land under their
source-relative path in the build tree -- the one documented instance,
`build-ninja/dronesim/spade/kat_host_spade.dll`, is the rule's witness. `RS11a`'s guard resolves
and pins the rest rather than this spec asserting them.

| Target | Role in this program | Build | Notes |
| --- | --- | --- | --- |
| `kat_host_spade` | Its render half is rewritten to call the engine renderer; the raster is deleted | `cmake --build build-ninja --target kat_host_spade` | Needs `-DKAT_BUILD_SPADE_HOST=ON` and an installed Spade at `KAT_SPADE_PREFIX` (`scripts\spade-prefix.ps1`) |
| `kat_frame_dump_spade` | **The checkpoint evidence tool** -- renders scenes to PNG for every book | `cmake --build build-ninja --target kat_frame_dump_spade` | Built only when `KAT_BUILD_SPADE_HOST=ON`; POST_BUILD copies the host DLL beside it, so it runs in place |
| `kat_frame_dump` | Fake leg; unchanged (fake keeps its wireframe raster) | `cmake --build build-ninja --target kat_frame_dump` | Always available |
| `kat_content_author` | Prefab expansion -- the authoring path for the `RS8` kit and `RS9` dressing | `cmake --build build-ninja --target kat_content_author` | **`EXCLUDE_FROM_ALL`** -- a plain build does NOT produce it; it must be named. Registered explicitly for this reason. |
| `kat-editor` | **Unchanged by this program** -- it simply receives better frames through the same C5 calls | `cmake --build build-ninja --target kat-editor` | Needs `-DKAT_BUILD_EDITOR=ON`; Preferences must name BOTH DLL paths |

Test executables are named for completeness only. The standing rule stands: tests run through
`pytest`, `ctest` or `scripts\test.ps1`, never by invoking a compiled test binary directly.

---

## 11. `RS12` -- Checkpoints

**`RS12`.** Every checkpoint is visualisation-bearing and every checkpoint names the exact
executable that produces its evidence and the exact command the user runs. **A checkpoint whose
evidence cannot be reproduced from this table by copy-paste has failed its own bar.**

| CK | Gate | Evidence and the executable that makes it | User action |
| --- | --- | --- | --- |
| (1) | First light -- shaded world, drone mesh, sun shadow | PNG book over `hover-pad` + `gate-corridor` from `kat_frame_dump_spade`; the same scenes live in `spade_sandbox` | `spade\scripts\demo.ps1 -Scene gate`, then open the book |
| (2) | Schema v2 + materials/lighting + `RS4` agreement | Ten worlds migrated and round-tripping byte-exact; raymarch-vs-raster agreement table with measured, pinned tolerances; side-by-side book from `kat_frame_dump_spade` | Review the agreement table + book |
| (3) | Kit-of-parts + ten scenes dressed | Gallery book: 10 scenes x 3 bookmarks from `kat_frame_dump_spade`; the kit's roster rendered as a parts sheet; **plus the same scenes opened in `kat-editor.exe` through File > Open Project** | `kat-editor.exe` -> open `demo_packages\racing-competition` and `test_packages\hover-pad-smoke` |
| (4) | Sandbox complete + v1 retired | All 11 ported scenes (8 v2 + 3 v1-derived) runnable in `spade_sandbox`; v1 tree deleted; build green with no GL dependency | `spade\scripts\demo.ps1 -Scene v1-fluid` (and each remaining scene) |

> **`RS12` checkpoint (4) carries dead text.** Its gate says the v1 tree is *deleted*. **`SL14a`
> supersedes that: v1 is QUARANTINED to `spade/legacy/`, never deleted** -- see `06-sandbox-and-v1.md`,
> which rules the word out. The checkpoint is also unreachable today: it is blocked on the open SPH
> transfer row. Quoted here as written because it is a checkpoint definition, not because it governs.


**CK-(3) carries the editor half deliberately.** The editor needs zero code changes for this
program, which means the only way to prove that claim is to *open the editor and look*. CK-(3)
therefore runs the real File > Open Project flow against the standing package-layout ruling's
roots -- `demo_packages/` and `test_packages/`, both fully openable -- rather than accepting a
frame-dump book as proof that the editor path works. The M-2 history is the reason: headless
evidence was green while the user could not open anything.

**Prerequisite, stated because it is exactly what failed before:** the dressed scenes render
through `kat_host_spade.dll`, so CK-(3) requires BOTH Preferences DLL paths set (runtime and sim
host) against freshly staged binaries. The fake host keeps its wireframe raster by design
(`RS1a`) -- an editor pointed at the fake will correctly show wireframes and **that is not a
defect**. The checkpoint's staging step names the exact DLL paths and their hashes, per the M-2
runbook correction.

---

## 12. `RS13` -- Verification

**`RS13`.**

- **Byte-exact frame goldens.** The CPU rasterizer has no RNG, no clock, no static mutable state
  and a fixed operation order, so same scene + same camera + same resolution => byte-identical
  pixels. Goldens are exact, not banded -- this is the payoff of choosing CPU first, and it is a
  stronger guarantee than S7b will be able to make.
- **`RS4` agreement:** raymarch vs fast path, every shipped world x every shipped bookmark,
  tolerances measured then pinned with provenance.
- **Render-cadence invariance** re-proven against the real renderer: stepping byte-identical for
  any cadence including none. Extends the existing case rather than replacing it.
- **Tessellation and CSG-meshing goldens:** byte-identical triangle emission per (primitive
  kind, params, resolution) and per (CSG subtree, resolution).
- **World v1 -> v2:** migration correctness, byte-exact round-trip for all ten shipped worlds
  plus the golden corpus, unknown-key rejection preserved at every level.
- **Resolver in situ:** hit renders the mesh, miss and malformed render the fallback marker and
  warn exactly once (`C3b`'s unit coverage plus, now, a rendered case).
- **C5 conformance:** both legs green, unchanged. The Spade host's semantics must not move
  because its pixels did.
- **Executable register guard** (`RS11a`) and the ported-scene inventory guard (`RS10b`, doc 06:
  every retired scene has a live successor before deletion).
- **Bench:** per-pass render timings recorded as baselines; the CPU path's frame cost at slot-0
  and 640x360 size classes measured and pinned, feeding `MV3`'s still-unmeasured multi-slot
  table.
- All testing local until the Actions quota resets (standing directive at drafting time).

---

## 13. `RS14` -- Sequencing, collisions, and amendments owed

**`RS14`.** The program runs standalone in worktrees, touching `spade/`, `content/`,
`dronesim/spade/` and `tools/`. **Zero editor changes** -- the same proof M1b made, and the
reason CK-(3) verifies it by opening the editor rather than by assertion.

| Owed to | What |
| --- | --- |
| Engine design section 13 | S7 splits into S7a/S7b; section 10's Vulkan half moves to S7b. **Amendment.** |
| Engine design `D7` / section 8 | World file grows the v2 appearance fields; "glTF import later, render-only" is discharged. **Amendment.** |
| **Engine design section 9** | ⭐ **ADDED 2026-09-17 -- the row `C10` found missing.** s.9 says the render module exposes a **`FramePool`** with `render(world, camera, target)`. Neither shipped in that shape: there is **no frame pool anywhere in `spade/engine/render/`**, and the entry point is `render(const RenderScene&, const Camera&, const RenderOptions&, RenderTarget&)` over a **caller-owned, non-owning** `RenderTarget` (`render/target.hpp`, ruling `PA-1`). `RS1` governs -- it describes what shipped -- and **the pooling obligation is met at the C5 seam**, `kat::hostspade::FramePool` (`dronesim/spade/frames.h`), which is refcounted, slot-indexed and shipping. **Amendment.** |
| Engine design `D10` | Strangler endpoint reached; v1 verdict rows close. **Amendment.** |
| **24th spec (`SL1`-`SL18`)** | ⭐ **ADDED 2026-09-17 -- this table named no successor at all, and one exists.** `SL14a` **supersedes `RS10b`**: v1 is *quarantined* to `spade/legacy/` as pure renames, never *deleted*, so `RS10b` and `RS12` checkpoint (4) are dead text. `SL10`-`SL13` **subsume `RS10a`**, keeping its presentation model and adding the object-centric UI. ⚠ **The direction is inward** -- this is not an amendment this program owes outward, it is one owed *to* this document by a later spec, and its absence is why the render spec "reads as authoritative to anyone who opens it alone. It is not, on this clause." **Amendment, inbound.** |
| Demo program / `DM3` | World schema v2 exists; `DM3`'s mover block is additive to it. Coordination row. |
| Phase G / `G2` | `G2`'s brief targets a raster that will no longer exist; `MV5` means the renderer is slot-ready, so the brief simplifies rather than grows. Pointer note. |
| `CS3` / `CS7` / `CS8` | Content evolves (dressing, materials); the resolver acquires its consumer; verification extends. Pointer notes. |
| `HS5` | **No amendment** -- the interim raster's retirement is `HS5`'s own stated endpoint (`RS1a`). |
| `MV3` | The measured multi-slot budget table this program's bench data finally makes pinnable. |

### Still owed, and not resolved here

`RS14` records that **the M-2 flight-fidelity decision (options A/B/C) remains OWED.** It is
unaffected by this program but is not closed by it. The note as drafted: a better-looking
viewport makes a ground-pinned drone *more* conspicuous, not less -- worth deciding **before**
CK-(3) puts ten dressed scenes in front of a viewer.

---

## 14. `RS15` -- Out of scope

**`RS15`.**

- **The Vulkan render backend** -- S7b, behind the same frame pool.
- **The ImGui debug HUD and the ImGui world/scene builder** -- seat named and held open
  (`RS10c`, doc 06), not built.
- **Camera sensors and rendered training pixels** -- `A8` stands: the vision lane remains
  host-synthesised analytic features from the SDF. Rendered camera pixels arrive with S7b.
- **Textures, PBR, post-processing, dynamic global illumination.**
- **Mesh colliders** -- `D3`'s later rung. SDF remains collision truth; `RS4` polices the gap
  rather than closing it.
- **Movers, Targets, kinematics** -- `DM3`'s engine addendum, additive to v2.
- **Editor placement UI** -- prefab palette, gizmos, world documents in the editor. Authoring
  stays tool-side; the editor views.
- **Level-of-detail, culling beyond frustum, occlusion** -- revisit only if a dressed scene
  misses the bench envelope.

---

## 15. `SR-17` -- The analytic infinite ground, reconstructed from its citations

**Status: IN FORCE, and this is the first time it has been written down.** Added 2026-09-17 by
user ruling, which required that `SR-17` be *written as the ruling in force* **before** anything
supersedes it. That order is the point: a horizon-blur ruling supersedes this section, and
superseding a rule that exists only in code comments is not possible.

### 15.1 Why this section reads differently from `RS1`-`RS15`

**This is archaeology, not authorship.** `SR-17` was ruled during the S7a render program (Task
R6) and **was never carried into any specification.** It is reconstructed here from its
citations in the tree, which are the only surviving statement of it. Two fragments below are
marked as verbatim because the code marks them so -- `test_render_scene.cpp:731` writes
*"(SR-17's own ruling text)"* and `:766` quotes another clause the same way. Everything else is
reconstruction, and is labelled.

> ⚠ **`SR-17` is not an isolated orphan, and the next reader should know the size of the
> population.** Measured 2026-09-17 at master: **40 distinct `SR-nn` rulings are cited in code**
> (`spade/`, `tools/`, `content/`, `dronesim/`, `editor/`, `runtime/`) and **15 appear anywhere
> in `design-specs/`**, leaving **29 orphaned** -- `SR-3 9 10 11 13 14 15 17 18 21 22 23 24 25
> 27 28 30 31 33 34 35 37 38 39 40 41 52 57 58`. The render program's rulings were harvested
> into this document under a **renumbered series** (`RS-*`), and the `SR-*` numbers the code
> actually cites were not carried across. So every `SR-nn` in a comment points at a series this
> spec set abandoned. **Writing this one section fixes 1 of 29.** The rest are in the state the
> user just ruled unacceptable for `SR-17`.

### 15.2 The ruling, as it currently stands

**`SR-17`** -- *the analytic infinite ground plane.* A world's standalone plane primitive is
rendered as an **analytic, infinite ground** in the background pass, rather than only as
tessellated geometry.

1. **Which planes qualify.** One `GroundPlane` candidate per **standalone plane primitive** --
   a node `split_program()` (`render/csg_mesh.hpp`) classifies as a union-primitive leaf.
   VERBATIM: *"a plane inside a subtract is a cutting half-space, not a floor."* A plane buried
   in a `subtract`/`intersect`/`smooth_union` subtree is excluded.
2. **No Y-up special case.** VERBATIM. Candidates are stored in **world space**, general for any
   transform; `scene_from_world()` bakes the node's own `SdfTransform` once at scene-build time.
   This is explicitly *"a DIFFERENT, newer, more general mechanism"* than the older
   `has_ground`/`ground_y` overlay-grid heuristic, which is `+Y`-only and is untouched.
3. **Alongside, never instead of.** The analytic ground is drawn **in addition to** the
   tessellated grid, not as a replacement for it (`test_render_raster.cpp:1317`).
4. **The bit-identity seam -- the load-bearing clause.** The tessellated-grid path and the
   analytic-ground path must agree **bit-for-bit**, and they do so by calling the *same*
   function on the *same* input rather than by two coincidentally-equal derivations
   (`scene.hpp:85`, `:116`; `raster_cpu.cpp:433`, `:575`). The code calls this *"SR-17's own
   load-bearing seam."*
5. **Hard horizon.** The sky is a two-stop zenith-to-horizon gradient; `horizon_fraction` is
   clamped to 1 at and below the true horizon so a pixel below it reads as flat horizon colour
   rather than an out-of-gamut extrapolation. The sky/ground boundary is therefore a **single
   hard transition with no blended row.** Enforced by `test_render_shading.cpp:559`, which
   rejects any row that is *"neither the exact sky colour nor the exact ground colour."*
6. **Unshadowed past `scene.bounds`.** The shadow map is orthographic and fitted to
   `scene.bounds`, so the infinite plane is **unshadowed beyond that box -- accepted and
   expected, not a bug** (`shadow.hpp:18`, `:23`, `:120`; `shadow.cpp:276`).
7. **Planes only.** The hard-horizon ground path handles `prim: plane` and nothing else
   (`content/prefabs/README.md:209`). A heightfield ground is explicitly forwarded, unbuilt
   (`tests/golden/render/agreement_bands.json`).

**Enforcement.** `test_render_shading.cpp:559` (hard horizon), `test_render_scene.cpp:726-766`
(classification, world-space generality), `test_render_raster.cpp:1317` (alongside the grid),
`test_render_shadow.cpp:299`, `:565` (bounds-fitted shadow), and the pinned golden
`spade/tests/golden/render/agreement_bands.json`.

**Scope limit, stated in the tree.** `agreement.hpp:236` records that the agreement harness
*"does not replicate `SR-17`'s own classification of which plane leaf is really a ground"* -- a
world shaped unusually can pass that harness's kind/op test while having an empty
`ground_planes`. A caller with such a world must confirm `RenderScene::ground_planes` is
non-empty for it independently.

### 15.3 Two disagreements among the citations, recorded before anything supersedes them

⚠ **(a) "No fog" is a COROLLARY in the citations, not a ruled clause -- and this narrows the
supersession.** Of **42 citations across 16 files**, exactly **two** mention fog (`scene.hpp:215`
and `test_render_shading.cpp:559`), both phrasing it as the parenthetical *"hard horizon, no
fog"*. Neither is marked as ruling text, while two *other* fragments explicitly are (15.2 clauses
1 and 2). The mass of the citations is about the ground plane, its classification, world-space
generality, the bit-identity seam and shadows. **The most defensible reading is that `SR-17`
ruled an analytic ground and a clamped sky gradient, and "no fog" is what that architecture
produces** -- there is no atmospheric term anywhere to switch off -- **rather than a prohibition
anyone enacted.** This matters: a horizon-blur ruling then supersedes **clause 5 only** and
leaves clauses 1-4, 6 and 7 standing. It is not resolvable from the tree alone, and the
alternative reading (that "no fog" was ruled) is available; it is recorded here so the
supersession states which reading it adopts instead of quietly picking one.

⚠ **(b) The shadow clause has two different attributions.** `shadow.hpp:18` attributes the
orthographic, `scene.bounds`-fitted shadow map to *"ruling SR-17"*, while `raster_cpu.cpp:1042`
calls the same mechanism *"R7's scene.bounds-fitted shadow frustum"* -- Task R7, where the rest
of `SR-17` is Task R6. A reconciliation is available (R6 ruled it, R7 implemented it) but the
tree does not say so, and **clause 6 should be treated as provisionally attributed** until
whoever holds the S7a record confirms it.

### 15.4 What a horizon-blur supersession must address

Recorded here so the successor ruling cannot be written without meeting them:

1. **`RS15` lists "post-processing" as out of scope.** A horizon blur implemented as a
   screen-space filter collides with that line; one implemented as an atmospheric term inside
   the background pass does not. **The successor must say which it is**, and amend `RS15` if it
   is the former.
2. **Clause 4 is the real engineering risk.** The bit-identity seam holds because both paths
   call one function on one per-pixel ray. `sky_gradient_color`'s own comment notes the gradient
   form is exact *"when `sky_zenith == sky_horizon`"* and that a double-to-float narrowing
   difference could round a flat sky to a different byte in each path. **A spatial filter is
   exactly the shape of change that breaks this**, and `agreement_bands.json` is pinned to it.
3. **`test_render_shading.cpp:559` goes red by design** and is rewritten, per the user's ruling.
   It is currently the only executable statement of clause 5.

---

## 16. `SR-17a` -- the atmospheric term supersedes `SR-17` clause 5 (**IN FORCE**, amended to all geometry)

> ✅ **STATUS: IN FORCE.** Two conditions gated this section and both are now met.
>
> **(1) The precondition this section named for itself is satisfied.** It was drafted fenced, and
> its own banner read: *"Until that signature lands, `SR-17` clause 5 is the rule in force and this
> section is not authority for anything."* **`RS1`-`RS15` was signed by the user on 2026-09-17** --
> the enumerated twenty, `RS10b` excluded -- recorded at `f3825028`.
>
> **(2) The user ruled the one question the draft left open enough to matter: its SCOPE.** Ruled
> 2026-09-17 -- **the atmospheric term applies to ALL GEOMETRY AT RANGE**, not to the sky/ground
> boundary alone. The draft proposed the narrower, background-pass-only form. **The ruling is
> against that reading**, and this section is amended to it rather than fenced behind it.
>
> ⚠ **What is the user's and what is this document's, kept separate.** The user ruled the scope, in
> those words, and nothing else here. Everything below -- the mechanism, the shape, the `RS15`
> argument, the test plan -- is this realm's derivation *under* that ruling and carries no
> signature. **"Horizon blur"** was the working name and remains the sandbox flag's spelling
> (`--horizon-blur`); the code spells it `horizon_blend_strength`. One thing, three spellings; the
> load-bearing word is **atmospheric**, because the whole `RS15` argument turns on it not being a
> blur in the screen-space sense.

> ⚠⚠ **WHY THE OLD BANNER WAS REWRITTEN RATHER THAN DELETED -- `R5` MODE 4, IN A SPECIFICATION
> CLAUSE.** Between the signature landing and this amendment, the old banner still read *"NOT IN
> FORCE ... until that signature lands"*, and it was **still correct about the status and wrong
> about the cause.** The section genuinely was not in force -- but no longer because a signature was
> owed; because the scope ruling superseded its text before that text ever took effect. **The label
> stayed legible while the cause underneath it was replaced.** A reader arriving here concluded the
> `RS` signature was outstanding, while line 3 of this same document said `SIGNED` -- two sentences
> contradicting each other, each one reading correct alone. ⭐ *Clearing a stated cause produces no
> visible change, which is why a row survives the event meant to close it.* Deleting the banner
> would have removed the only evidence that this happened.

### 16.1 Which reading of `SR-17` this adopts, declared rather than assumed

s.15.3(a) records two readings and requires the successor to declare one. **This adopts the
corollary reading:** `SR-17` ruled an analytic ground and a clamped sky gradient, and "no fog" is
what that architecture *produces* rather than a prohibition anyone enacted. The evidence is that of
42 citations exactly two mention fog, both parenthetical, neither marked as ruling text, while two
other fragments explicitly are.

**Consequence: this supersedes CLAUSE 5 ONLY.** Clauses 1, 2, 3, 4, 6 and 7 stand unchanged -- the
standalone-plane classification, "no Y-up special case", alongside-not-instead-of, the bit-identity
seam, unshadowed-past-`scene.bounds`, and planes-only. A wholesale supersession of "`SR-17`" would
have retired all six as collateral.

**⚠ THE WIDENING PUTS THE TERM ALONGSIDE TWO CLAUSES IT DOES NOT SUPERSEDE, AND BOTH NEED SAYING
OUT LOUD.** While the term lived in the background pass it could not meet clauses 6 or 7 at all.
Applied to all geometry at range it now sits beside both, and a reader who finds them in one
document will otherwise reconcile them privately and wrongly.

* **Clause 6 (unshadowed past `scene.bounds`) is NOT repealed, and the term must never be offered as
  its remedy.** Blending distant geometry toward the sky makes the unshadowed far ground *less
  visible*; it does not make it shadowed. ⭐ **An artifact made harder to see is not an artifact
  removed** -- and the difference becomes load-bearing the moment someone turns the strength down
  and the old look returns with no code having changed. If clause 6 is ever closed, it is closed by
  fitting the shadow, not by hiding the evidence.
* **Clause 7 (planes only) and "all geometry at range" are about different subjects and do not
  contradict.** Clause 7 scopes *which ground is drawn analytically* -- `prim: plane` and nothing
  else. The term scopes *what gets an atmospheric weight applied once it has been shaded.* A
  heightfield ground still has no analytic path, and clause 7 stands; if one is ever built the term
  applies to it because it is geometry at range, **not because clause 7 moved.**

### 16.2 The mechanism

**`SR-17a`** -- **every shaded surface is blended toward the sky colour along its own view ray, as a
function of its distance from the eye.** The sky/ground boundary is the case of this rule that has
no surface in front of it, and is therefore an atmospheric blend rather than a hard transition.
⭐ **The clause-5 supersession is a CONSEQUENCE of the rule, not the rule** -- which is the whole
difference between this and the draft, and the reason the draft's definition sentence had to be
replaced rather than widened.

#### ⛔ What this replaced -- preserved verbatim, because the amendment is only legible against it

The draft was **narrower than the ruling**, and three sentences carried the narrowing. They are kept
here as the record of what was superseded. Each reads perfectly correct in isolation, and a future
reader who finds only the new text cannot otherwise tell which sentences were load-bearing or why
they were replaced.

| # | the superseded sentence, verbatim | why it had to go |
|---|---|---|
| 1 | *"the **sky/ground boundary** is an atmospheric blend evaluated per-pixel **inside the background pass**, not a hard transition"* | **The definition.** It names the subject as the boundary and the site as the background pass. Both are now wrong, and it is the sentence a reader would quote. |
| 2 | §16.2's `stage` row: *"inside the existing background pass, **before geometry**"* | **The `RS15` argument's entire foundation.** It distinguished this from post-processing by WHEN it runs. The term now runs after geometry, on geometry, so the distinction must be rebuilt on a different axis or `RS15` is collided with. |
| 3 | *"one **pure function of the ray direction**"* | ⭐⭐⭐ **The clause that fails quietly.** See the derivation below -- this is the one worth the space. |

#### ⭐⭐⭐ The shape, re-derived -- and why the old one could go false with nothing going red

**The old phrase did four jobs at once, and did all four well *only inside the background pass*.** It
said the term is *one pure function of the ray direction*. In a pass containing no geometry, that
single phrase (a) named the entire input, (b) forbade reading any neighbouring pixel -- a ray is not
a framebuffer, (c) guaranteed both render paths compute the same value, since both reconstruct the
same ray from the same camera, and (d) left nothing for draw order to perturb.

**Extend it to geometry and exactly one of the four breaks -- silently.** Distance stops being a
function of direction the moment a mesh decides it: two rays with the same direction reach different
surfaces in different scenes, and the same ray reaches a different surface when an object moves.
⚠⚠ **Nothing goes red.** The code still compiles, the function still accepts a direction, the
sentence still parses, and the value it returns is simply no longer the one the clause promised.

⭐⭐⭐ **IT FAILS QUIETLY BECAUSE ITS TRUTH CONDITION IS A PROPERTY OF THE SCENE, NOT OF THE CODE.**
*"Distance is a function of direction"* is a fact about a world with nothing in it. **Content can
falsify it without one edit to the renderer** -- and that is the single class of clause no test, no
compiler and no reviewer can hold, because there is no moment at which anybody did anything wrong.

**So the replacement is chosen to have a truth condition that is a property of the SIGNATURE.**

> **`SR-17a`'s shape.** The term is **one pure function of three values the pixel already owns**:
> the **colour** already computed for that pixel, the **distance along its own view ray** to the
> surface that produced that colour, and the **sky colour along that same ray**. It takes nothing
> else.

**That is a shape, not a prohibition, and it guards by construction the three things the old phrase
guarded by assertion:**

* **No neighbourhood, therefore not post-processing.** There is no framebuffer parameter, no screen
  coordinate, no second sample. ⭐ **A neighbourhood filter cannot be written against this signature
  at all** -- it would need an argument that is not there, and adding one means editing a
  declaration in a shared header, which is a visible act in a diff. *The old phrase forbade
  neighbourhood access; this one makes it inexpressible.*
* **Clause 4's bit-identity, stated against its REAL subject.** ⚠⚠ **Corrected here against an
  earlier version of this very section, which had it wrong and which I carried forward without
  checking.** Clause 4's seam is **the tessellated-mesh path and the analytic-ground path, BOTH
  INSIDE THE RASTER RENDERER** -- `scene.hpp:85` names them exactly: *"two call sites that must
  agree bit-for-bit (SR-17's load-bearing seam): the tessellated-mesh per-vertex normal transform
  (raster_cpu.cpp's draw_mesh_item) and the analytic background ground plane's precomputed world
  normal (scene_from_world())."* Raymarch is *"a THIRD call site with the identical requirement"*
  (`scene.hpp:118`) -- it shares the function, but **raster-vs-raymarch is governed by `RS4`'s
  agreement BAND, a tolerance, not by byte equality.** s.15.4 already recorded why: a
  double-to-float narrowing difference can round even a flat sky to a different byte in each path.
  ⭐⭐⭐ **AND THE CORRECTED SUBJECT HANDS THE IMPLEMENTATION ITS SHARPEST CONSTRAINT, which the
  wrong one did not.** The analytic ground applies the term **per pixel**. So the mesh path must
  apply it **per pixel too** -- **a per-VERTEX application, interpolated, breaks clause 4 by
  construction**, because the two sides of the seam would then be computing the term at different
  rates on the same ground. The Gouraud fill shades per vertex (`SR-18`); the term must not join
  it there. *The false claim was the weaker one: "both paths agree" would have been satisfied by
  two wrong-but-equal implementations, while the true one forbids the natural mistake.*
* **Order-independence.** The function reads only values belonging to the pixel being written, so no
  draw order can perturb it.
* **Exactness at strength 0 is a REQUIREMENT, not a style preference.** The blend must be written
  `base + (other - base) * t`, which returns `base` bit-for-bit at `t == 0`. ⚠ **`base * (1 - t) +
  other * t` does NOT** -- it is exact at zero only by the luck of rounding. The two are
  algebraically equal and are not the same instruction sequence, and this one feeds a sha256 golden.
* ⚠ **No transcendentals.** The fade is rational -- `1 / (1 + r*r)` and its relatives; `+ - * /
  sqrt`, `floor` and `abs` only. `exp()` is a cross-libm coin flip on a byte the frame manifest pins
  across platforms, so the standing physics rule applies here for the same reason.

⭐ **And the superseded rule becomes the degenerate case rather than a contradiction.** Where the ray
hits nothing, the "surface" is the analytic ground or the sky, and its distance **is** determined by
direction. `SR-17a` therefore *contains* the sentence it replaced, at the one place that sentence was
true.

⚠ **A property of a scene can be falsified by content; a property of a signature can only be
falsified by an edit.** That is the entire reason for the swap, and it is the sentence to keep if
only one survives.

#### `RS15` needs no amendment -- and this argument is REBUILT, not inherited

⚠⚠ **THE DRAFT'S ARGUMENT IS GONE AND MUST NOT BE CARRIED FORWARD.** It rested on the `stage` row:
*this is not post-processing because it runs before geometry.* The term now runs after geometry and
on geometry. **That sentence is false, and everything resting on it is unsupported.**

⚠ **The ground moved in the other direction too: `RS15` was an UNSIGNED DRAFT when the old argument
was written, and it is SIGNED now.** Arguing that a draft "needs no amendment" is cheap; arguing
that a *signed ruling* is not collided with has to stand on the signed text. It is rebuilt from
scratch here.

**`RS15` as signed excludes "Textures, PBR, post-processing, dynamic global illumination."** The only
member at issue is post-processing, so the entire question is what that word denotes.

⭐⭐ **POST-PROCESSING IS DEFINED BY ITS INPUT, NOT BY ITS STAGE -- AND THE STAGE TEST WAS ALWAYS THE
WEAKER OF THE TWO.** A pass that reads the rendered framebuffer, sampling its own neighbours, is
post-processing whenever it runs. A pass that reads only values belonging to the pixel it is writing
is shading, however late it runs: the industry's name for shading that runs late, on geometry already
rasterised, is *deferred shading*, and nobody calls that post-processing.

**By the shape above, the term reads three values belonging to its own pixel and nothing else. It is
therefore shading, and `RS15` is not collided with.** No amendment to `RS15` is owed.

⭐⭐⭐ **AND THE EXCLUSION IS WHY THE FORM IS RIGHT, NOT AN OBSTACLE THE FORM ROUTES AROUND.** `RS15`
forbids the screen-space version. The screen-space version is also the one that breaks clause 4,
because the two render paths **do not share a framebuffer** and a neighbourhood filter has nothing to
be bit-identical *on*. **`RS15` and clause 4 are one constraint seen from two sides**, and a form
satisfying either satisfies both. The per-pixel shape is not a compromise struck to fit inside the
exclusion -- it is what both clauses were already demanding.

| | screen-space blur (REJECTED) | atmospheric term (ADOPTED) |
|---|---|---|
| input | the rendered framebuffer, sampling **neighbouring** pixels | **three values belonging to this pixel alone** -- its shaded colour, its own ray's hit distance, its own ray's sky colour |
| stage | a pass after geometry | **after geometry, on geometry -- and the stage is not what decides it** |
| `RS15` | collides -- post-processing | no collision -- **shading, by input** |
| clause 4 | **breaks it** -- the two paths share no framebuffer | **preserves it** -- both paths reach the same three arguments |
| basis | a camera artifact | aerial perspective -- what a real horizon does |

**The precedent is in the tree, and it is the reason a shared function is available at all.**
`sky_gradient_color()` (`scene.hpp:246`) is already a per-ray function called by both paths on the
ray each reconstructs, and its own comment records that its form is **exact, bit-for-bit, when
`sky_zenith == sky_horizon`**. `SR-17a` joins that family and inherits that property as an
obligation. ⛔ **Clause 4 forbids a parallel implementation: ONE function, called from both paths.
Two expressions that happen to agree is the precise thing clause 4 exists to rule out.**

### 16.3 What moves, what must not, and why -- stated BEFORE it moves

s.15.4 requires whoever implements this to say why the agreement golden moves before it does.
⭐⭐ **THE GOLDEN MOVES; THE INVARIANT DOES NOT.** These are different objects, and conflating them
is how a real regression gets waved through as "expected":

* **`agreement_bands.json` MOVES, and that is correct.** Pixel values near the horizon change by
  construction -- that is the feature. Predicted here, regenerated with this section cited.
* **⭐⭐ THE POTENTIAL RADIUS IS THE WHOLE SHADED CORPUS -- AND NOTHING IN IT ACTUALLY MOVED.**
  Under the draft only background pixels could move. Under the ruling, **every pixel showing a
  surface at range moves IN ANY SCENE THAT ENABLES THE TERM.** The engine default is strength **0**,
  the same ruling the infinite grid got (the reference application opts in; no consumer inherits a
  render change it did not ask for), so **no committed golden enables it and none moved.**
  ⭐⭐⭐ **SO THE FOUR PRE-`SR-17a` FRAMES BECAME CONTROLS INSTEAD OF CASUALTIES** -- they now prove
  the degenerate case is bit-exact, which is a stronger thing than a regenerated hash can ever be.
  **Adding a golden is strictly safer than moving four**, and the earlier expectation that "goldens
  move once" was inherited from the draft's assumption that the term would default on.
  ⚠ **A golden whose subject is a FEATURE must be falsified by that feature's absence**, so the
  added frame is a **pair**: one scene rendered twice, asserting the two differ before either hash
  is consulted, with the off arm re-deriving the pre-`SR-17a` hash. A lone "term ON" frame would
  have pinned that the pipeline ran, not that the term applied -- a fifth control wearing a
  feature's name.

> ⚠⚠ **THE BULLET BELOW IS TRUE OF THE GRID THAT EXISTS AND INCOMPLETE ABOUT THE GRID BEING
> ASKED FOR. Corrected 2026-09-17 before implementation, by me, against my own claim.**
>
> It says the grid cannot widen the band, and that holds **for the grid as an OVERLAY**. But
> **"infinite" cannot be drawn as line segments** -- `draw_ground_grid` emits 21+21 finite
> `draw_world_segment` calls over ±10 m, and extending that to "infinite" only ever means a bigger
> finite number, with worsening aliasing at range. **A genuinely infinite grid is ANALYTIC**:
> evaluated per pixel where the camera ray meets the ground plane, exactly as the ground itself
> already is.
>
> **That moves it out of the overlay pass and into the background pass -- which is SHARED WITH
> RAYMARCH, so `overlays = false` no longer excludes it and the agreement band DOES move.** The
> raster background pass's own comment already names the distinction, calling `draw_ground_grid`
> *"the OLDER, unrelated ground-grid overlay below"*.
>
> ⭐⭐⭐ **A BLAST-RADIUS MEASUREMENT IS ABOUT A MECHANISM, NOT A FEATURE NAME.** I measured "the
> grid" and wrote the answer about "the grid", and the implementation changes which mechanism the
> word denotes. The measurement was correct and its subject was the thing I was about to replace.
>
> **Consequences, stated before the code:**
> * the analytic grid joins the **shared** per-ray function family, like `sky_gradient_color`, so
>   both paths compute it from the one ray each already reconstructs -- **not two expressions that
>   happen to agree**;
> * `agreement_bands.json` **moves for the grid too**, not only for the blur;
> * the analytic grid inherits `SR-22`'s gating: **shaded mode only**, like the analytic ground;
> * ✅ **the ±10 m overlay grid STAYS.** In wireframe there is no analytic ground to draw a grid
>   on, so the overlay is the only grid that mode has. Retiring it would silently remove the grid
>   from wireframe while "adding an infinite grid" -- two mechanisms, one word, opposite effects.

* ✅ **THE INFINITE GRID CANNOT WIDEN THE AGREEMENT BAND -- measured 2026-09-17, not assumed.**
  The grid is an **overlay**, and `test_render_agreement.cpp` sets `fast_options.overlays = false`
  at **three** call sites (`:679`, `:713`, `:856`), so overlays are excluded from the `RS4`
  comparison by construction. The raymarch path declares the same from its own side:
  `raymarch.hpp:232` -- *"NO-OPS for this draw mode: no ground grid, world-bounds box, spawn/body
  markers."* **So making the grid infinite moves FRAME goldens (any scene with overlays on) and
  moves the agreement band NOT AT ALL.** The blur moves both. ⭐ **Two changes shipping in one wave
  with DIFFERENT blast radii -- say which moved what in the regeneration note, or the next reader
  attributes the band movement to the grid and looks for a bug that is not there.**

* ⛔ **The bit-identity SEAM must NOT move -- and its subject is the TESSELLATED/ANALYTIC pair
  inside raster.** ⚠⚠ **The draft said *"if raster and raymarch disagree by a single byte, that is
  a REGRESSION"* and that sentence is FALSE.** It was written here, carried forward once, and is
  struck now: those two paths are compared by `RS4`'s tolerance band, and s.15.4 records in this
  same document that a double-to-float narrowing can already put them a byte apart on a flat sky.
  **Holding them to byte equality would report a known, accepted numerical difference as a
  regression** -- and the first person to hit it would have had to argue against a spec.
  * **What must hold byte-for-byte:** a ground pixel drawn by the tessellated mesh and the same
    ground drawn analytically. **If those disagree by one byte after this change, that is a
    REGRESSION -- revert rather than regenerate.**
  * **What must stay inside a band:** raster vs raymarch. The band may move; it may not widen
    without a stated cause.
* ⚠ **`agreement_bands.json` MOVES BY A NEW MEASUREMENT VERSION, NEVER AN IN-PLACE EDIT.** `SR-31`
  is enforced by the file's own schema note: measurements are a versioned list under
  `measurements.<version>` with `active_measurement_version` selecting the live one, and a
  re-measure **adds a version and bumps the selector.** ⭐ The note records that the pre-fix file
  read `worlds.<name>.band` directly, so *"the ONLY way to make a re-measurement take effect was
  the in-place edit the prose forbade"* -- **a rule whose only compliant path was the one it
  banned.** Editing version 2 would destroy the record this change is supposed to be measurable
  against.

⭐⭐⭐ **THE DEGENERATE CASE IS A FREE, EXACT REGRESSION CONTROL, AND IT IS THE REASON TO PREFER THIS
FORM EVEN IF BOTH WERE IN SCOPE.** `sky_gradient_color`'s own comment records that its form is
**exact, bit-for-bit, when `sky_zenith == sky_horizon`** -- `horizon - zenith` is then exactly zero
and the multiply-add cannot perturb the result. `SR-17a` inherits that as a **requirement**: at
atmospheric strength **0**, output must be **bit-identical to today's hard horizon, on both paths,
for every pixel.**

That buys three things at once. Every existing golden must hold unchanged at strength 0, so the
change is **opt-in and falsifiable rather than sweeping**. A single knob separates *"the blend is
wrong"* from *"something else moved"*. And the rewritten test gets a control that is **a real member
of the corpus rather than a synthetic one** -- this realm's recorded requirement for a discriminating
fixture.

### 16.4 The tests -- and two of them respond to this change in OPPOSITE directions

⚠⚠ **THE DRAFT NAMED ONE TEST. THERE ARE TWO, THEY SHARE A NOMINAL SUBJECT, AND THE SAME CHANGE
PRESERVES ONE EXACTLY AND MOVES THE OTHER.** Measured 2026-09-17 by reading both bodies, not inferred
from their names. The difference is not about the term at all -- it is about **what each one
compares.**

| test | what it actually compares | under an all-geometry term |
|---|---|---|
| `ShadowPerspectiveCorrectness.MeshReceiverMatchesExactAnalyticGroundAcrossALargeDepthRange` (`test_render_shadow.cpp:632`) | **the same pixel, across two SCENES** -- one with an analytic ground, one with a huge tessellated quad, one camera | ✅ **STRUCTURALLY PRESERVED; numerically a measurement, not a deduction.** At a given pixel both scenes' ground is at the same world position, so the same distance yields the same weight. ⚠ But it compares with a **1% fringe tolerance**, and its own comment says why: the two variants *"differ in HOW each recovers the ground's own world position per pixel (exact ray/plane intersection vs perspective-correct Gouraud interpolation)"* |
| `RenderShading.TessellatedAndAnalyticGroundAgreeAcrossTheHardHorizonSeam` (`test_render_shading.cpp:395`) | **two DIFFERENT pixels, in one scene** -- one column inside the tessellated grid's bound, one outside, on one row | ⚠ **MOVES.** The two columns sit at different world X and therefore at different distances from the eye; a distance-keyed term correctly gives them different values |

⚠⚠ **"PRESERVED EXACTLY" WAS MY OWN OVER-CLAIM AND IT IS CORRECTED ABOVE.** I reported that row as
exact before reading how the test compares, and it compares with a tolerance whose existence is the
whole point of the test. ⭐ **Same species as the clause-4 error one commit earlier: a stronger,
tidier word than the evidence carried, arriving in the position of a conclusion.** Twice in one
section, from the same cause -- *the tidier sentence closes the paragraph better, and nothing in the
writing resists it.*

⭐⭐⭐ **AND THE CORRECTION DISSOLVED WORK RATHER THAN ADDING IT.** Believing the two paths had to
agree byte-for-byte, I was designing a scheme to reconstruct the mesh path's ray from NDC so its
distance and sky direction would be bit-identical to the analytic path's. **Clause 4 never asked for
that** -- `scene.hpp:85` names its bit-identical pair as two *normal* transforms, and the tree
already tolerance-compares position-derived quantities on both seams. **An over-strong reading of a
constraint does not merely mislead; it commissions work.**

⭐⭐ **AND THE NARROWER FORM WOULD HAVE BROKEN BOTH, WHICH IS THE ARGUMENT FOR THE RULING THAT IS NOT
AESTHETIC.** Under a background-pass-only term the analytic ground receives the term and tessellated
ground does not -- so the shadow test's two frames diverge entirely, and the shading test acquires a
**hard discontinuity at exactly the tessellated grid's edge**: blended outside, unblended inside.
⛔ **That is a visible seam at the precise line `SR-17` clause 3 exists to make invisible.** The
all-geometry form is not the more ambitious reading of the two; **it is the only one that does not
reintroduce the artifact the whole `SR-17` design was built to remove.**

⭐⭐⭐ **THE SHADING TEST'S BYTE-EQUALITY WAS ONLY EVER VALID BECAUSE THE SHADING MODEL HAD NO
POSITION DEPENDENCE.** A flat plane under a fixed sun shades to one constant colour, so two pixels of
that ground matched wherever they were. **That is what made the fixture convenient, and it is exactly
what made it unable to distinguish *"both paths implement the same model"* from *"both pixels have
the same bytes."*** The term introduces a position dependence -- by ruling, correctly -- and the test
reports it as the two paths disagreeing. ⚠ **This is a false positive for disagreement, not a
regression**, and it is this realm's recorded coincidence lesson arriving inside a verification:
*the convenient fixture is convenient precisely because two things coincide in it.*

⛔ **AMEND, NEVER DELETE -- and the reason is structural.** The body at `:395` holds **two
independent assertions with different subjects**, and its name covers only one of them:

1. `:511-520` -- the **seam agreement**, four BGRX bytes compared across the tessellated/analytic
   boundary. This is clause 3 and clause 4's executable statement.
2. `:545-565` -- the **hard-horizon column scan**, requiring every row to be exactly the sky oracle's
   colour or exactly the ground colour, with no third value anywhere. This is **clause 5's only
   executable statement.**

**Deleting or rewriting the test wholesale destroys (1) as collateral -- the very clause `SR-17a`
must preserve.** ⭐ *A test named for where it looks is not named for what it checks*, and here one
body is named for its first assertion while silently carrying the second.

**The amendment, therefore:**

* **The existing assertions survive WHOLE as the strength-0 arm.** Not weakened, not re-scoped: run
  at `horizon_blend_strength == 0` they must pass byte-for-byte exactly as they do today. ⭐ **This
  is what makes the strength-0 requirement load-bearing instead of decorative -- it is how the
  superseded clause keeps an executable statement instead of being deleted.**
* **A new strength-above-0 arm is added beside them**, asserting the properties that survive a
  position-dependent model: the blend is **monotone** from surface colour toward sky colour with
  increasing distance; it is **bounded** by the two endpoint colours and never out of gamut -- the
  property clause 5's clamp existed to protect; and **the tessellated and analytic renderings of
  one ground produce the same bytes**, which is clause 4's actual subject. ⚠ **NOT "both render
  paths produce the same bytes"** -- that was the draft's wording, it is wrong for raster vs
  raymarch, and asserting it would build a test that fails on an accepted narrowing difference.
* ⚠ **The new arm must compare each pixel against an independent oracle evaluated at THAT PIXEL'S
  OWN DISTANCE** -- never one pixel against another. Comparing two pixels is the defect above, and
  re-using that shape in the replacement would rebuild it with a wider tolerance.
* ⚠ **A monotonicity assertion needs a positive control.** A term that is identically zero is
  monotone, bounded, and passes both. The arm must also assert that the blended value **differs**
  from the unblended one at some measured distance, or it cannot tell a working term from a dead one.

⚠ **`:545-565` is currently the only executable statement of clause 5**, so until the new arm lands
that clause has exactly one guard and it is the one being amended. **The amendment ships in the SAME
commit as the behaviour change, never after.**

📎 **Recorded so it is not re-discovered:** the comment at `:531-544` documents that this scan's
*earlier* form could not fail -- it restated the loop's own break condition at an index it had
already passed, and **passed a synthetic 11-row sky-to-ground blend fed to it deliberately.** The
current form scans the whole column and rejects any third value. ⭐ **That synthetic blend is the
positive control this section's new arm needs, and it already exists in the record** -- the arm that
must now go GREEN on a blend is the one whose predecessor was falsified by exactly that input.

---

## Provenance

Twenty-first spec, drafted 2026-08-24, ✅ **SIGNED 2026-09-17** (the enumerated 20; `RS10b`
excluded). Amends the approved Spade engine design
at sections 8, 9, 10 and 13 and at `D7` and `D10` -- see the two `UNRESOLVED` markers in section
1 for the two places where an approved wording has not yet been changed to match. Companion
source documents: `superseded/kat-spade-host.html`, `kat-viewport-cameras.html`, `kat-default-content.html`.
