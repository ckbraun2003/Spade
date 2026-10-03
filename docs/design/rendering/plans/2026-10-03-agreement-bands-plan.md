# Agreement bands on Spade's own worlds: plan

**Owner:** Rendering, with Test/Docs for the band file's governance (`TD-2`) and the Docker leg. **Status:** done, merged at `da2fcf5` on 2026-10-03. The lead answered Q1–Q3 (golden worlds only; cameras in the band file; Test/Docs owns the margin, signed off with four conditions). Measuring found two raster defects, so version 1 has 5 cases, not 7: see `../07-status.md`. It answers the backlog row "Spade-owned content for render agreement bands" (`../../backlog.md`).

## Goal

`RS4` holds that the ray-marched image is the collision field, and that raster silhouettes must agree with it within measured, pinned bands. Since `ab2a21e` no world in this repository has a band. The 30 matrix cases, 10 KAT worlds × 3 cameras, moved to KAT with their data. After this plan, an agreement matrix runs on worlds this repository owns. Every case proves it can see its own geometry (`SR-30`), and every band carries its provenance (`TD-2`, `SR-31`).

**Done when:** the matrix's bands are pinned in a versioned file, every case discriminates, and the Docker leg runs the matrix green.

## What the KAT matrix taught

The last KAT measurement (version 2, at 160x120) pinned 30 cases. **Only 9 could detect the deletion of every object but the ground.** All 20 `default` and `top_down` cameras looked down at an infinite ground that filled the frame. The bare-ground probe disagreed by exactly as much as the full world did, so those bands could not tell geometry from no geometry. Only the low `family_third` framings, with geometry against the sky, discriminated.

**So the new matrix chooses cameras for discrimination first.** A camera puts geometry against the flat sky; it does not look down at the ground. A case that does not discriminate is reframed before pinning, or dropped with the reason recorded. It is never pinned silently.

## Which worlds

| World | Static geometry | Probe reference | Cases |
|---|---|---|---|
| `tests/golden/worlds/gate.world.yaml` | plane, torus ring, two posts | ground plane only (`strip_to_ground_plane_only`) | 2: through the ring at eye height, and three-quarter from the side |
| `tests/golden/worlds/maximal.world.yaml` | a CSG tree with every op (union, intersect, subtract, smooth union), a torus and a heightfield. Its plane sits under `intersect`, (plane ∪ sphere) ∩ box, so it is a cutting half-space, not a ground (`SR-17`) | **empty world** | 3: the CSG cluster against the sky, the torus edge-on, and the heightfield's ridge against the sky |
| `tests/golden/worlds/shower.world.yaml` | a hollow sphere shell (sphere minus sphere), no ground plane | **empty world** (no static geometry) | 2: outside, and inside looking at the shell's inner wall |

**Not used:**
- `bounce`, `two_world_isolation`: a plane alone. A case there cannot discriminate by construction, and `AgreementProbe.*` already covers the probe on a bare plane.
- `ballistic`, `quad_hover`: no static geometry.
- **Viewer scenes:** `drop`, `hover` and `swarm` are a plane alone, and `gate` repeats the golden gate. Only `shower`'s box container would add geometry (Q1).

**The probe for a world with no ground.** `strip_to_ground_plane_only()` refuses a world with no plane leaf (`not_found`). It also does not tell a ground from a cutting plane; its own header says so. So the matrix picks the probe from what the renderer found: the ground-only world when `RenderScene::ground_planes` is non-empty (`gate`), else the world with every static node removed, which is all sky (`maximal`, `shower`). A band that cannot detect that does not discriminate either. This is a test-local helper, not an engine API.

## How the bands are measured

Each case is a world, a camera and a resolution of 160x120, as before. All of it runs on the CPU, so it is reference grade and runs on the Docker leg too (`TD-13`: no `gpu` label).

1. **Flat sky:** set `sky_horizon` to `sky_zenith`. Then **Step 1c:** `assert_no_material_matches_sky()` must pass for the world's materials.
2. Render the raster and the ray-march, then compare them with `compare_silhouettes()`. This gives the disagreement `d`, coverage only (`RS4`).
3. **Probe (`SR-30`):** ray-march the probe reference and compare it with the real raster. This gives `d_probe`.
4. **Band:** `round_up(0.001, max(0.001, 1.5 × d))`. This is the margin rule of the KAT measurements (Q3). `detects_total_deletion` is `d_probe > band`.
5. **Pin (`SR-31`):** write measurement version 1 to `tests/golden/render/agreement_bands.json`. It holds `active_measurement_version`; per case, the camera, `d`, both coverage counts, `d_probe`, the band and `detects_total_deletion`; and the commit, date, toolchain and resolution. A re-measure adds a version, and never edits one.

**The test (`AgreementMatrix`, value-parameterized over the active version's cases):**
- re-renders each case and asserts `d ≤ band`;
- re-runs the probe and asserts `detects_total_deletion` still holds, live, never trusted from the record (`SR-30`);
- asserts that every case discriminates.

**The camera lives in the measurement.** A band is a fact about a framing, so a changed camera is a new version, not an edit.

**Measuring without editing by hand.** With `SPADE_AGREEMENT_MEASURE` set, the matrix writes its measured values as JSON under `<build>/tests/test-output/`, beside the frame dump, and changes no result. The pinned file is written from that output, with provenance added.

## What replaces the 30 cases

**7 cases on 3 worlds**, every one discriminating, in place of 30 cases of which 9 discriminated. Coverage moves from "many tracks seen from above" to "every SDF primitive and op seen against the sky": plane, sphere, box, cylinder, capsule, torus, heightfield, union, intersect, subtract and smooth union. KAT keeps its own matrix for its own worlds.

## Steps

1. **Branch.** Write the band file's schema, version 1 empty. Write `AgreementMatrix`, the probe for ground-less worlds and the measure mode. The cases fail until pinned. No slot is needed yet.
2. **Slot.** Measure. Reframe any case that does not discriminate, and record why. Pin version 1 with provenance. Then run the full suite.
3. **Docker leg.** The matrix runs under gcc. Raster and ray-march are both deterministic fp, so the fractions should match the MSVC run exactly. A difference is reported as a finding, never absorbed by the margin.
4. **Docs.** In `07-status.md`, the "Agreement bands" row says built, and the debt row closes. In `03-verification.md`, the matrix's worlds are named. The backlog row closes.

## Questions for the lead

- **Q1.** Golden worlds only (recommended), or add the viewer's `shower` box container through `spade_viewer_scenes`? It would add box-on-box silhouettes, but a Rendering test would then depend on Interface's tool library.
- **Q2.** Should the cameras live in the versioned band file (recommended, so a changed framing is a new version), or in the test code?
- **Q3.** Keep the KAT margin rule, `1.5 × d` with a 0.001 floor? Test/Docs may want a different margin under `TD-2`.
