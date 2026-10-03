# Rendering — verification

**Owner:** Rendering (each realm writes its own tests; Test/Docs sets the gates). **Normative.**

## What each kind of evidence is for

| Evidence | Proves | Lives in |
|---|---|---|
| **Frame goldens** (`RS13`) | The CPU raster is byte-exact for a fixed scene, camera and options | `tests/golden/render/frames/manifest.json`, `tests/test_render_raster.cpp` (`RasterGolden.*`) |
| **Tessellation and CSG goldens** (`RS13`) | Triangle emission is byte-identical per primitive, parameters and limits version | `tests/golden/render/{tessellation,csg}/manifest.json` |
| **Agreement** (`RS4`) | The raster's silhouettes match the SDF that collision evaluates | `render/agreement.*`, `tests/test_render_agreement.cpp`; the band matrix `tests/test_render_agreement_matrix.cpp` over `tests/golden/render/agreement_bands.json` |
| **Seam tests** (`SR-17` clause 4) | The tessellated and analytic ground produce the same bytes | `tests/test_render_shading.cpp` |
| **Cadence invariance** (`L5`) | Rendering zero times, once, or every tick leaves stepping byte-identical | **nothing in this repository yet.** It holds by construction (the renderer reads only its arguments), but the case that tested it lived on KAT's side. Debt in `07-status.md` |
| **Bench** (`RS13`) | Per-frame CPU render cost at fixed size classes | `tests/bench/bench_render.cpp`, `tests/bench/baselines.json` (recorded, not gated) |

## Goldens

- **CPU only.** A GPU frame is never a golden (`L4`).
- **One hash per frame, so promotion needs two platforms.** A new or moved hash is promoted only after Windows/MSVC and Linux/gcc compute the same full 64 characters. If they differ, nothing is promoted: two platforms disagreeing is a bigger finding than a stale golden.
- **A move names its cause.** A regenerated hash carries a changelog entry in its manifest saying what moved and why, in the same commit as the cause. A hash that moves with no such commit is rot.
- **A golden whose subject is a feature is a pair.** Render with the feature on and off, assert that the two frames differ before consulting either hash, and pin both. The `SR-17a` frame pair is the model.
- **Default-off features leave existing goldens alone.** At strength 0 the atmospheric term must reproduce every pre-existing frame exactly. Those frames are its controls.

## Agreement (`RS4`)

- **Coverage, never colour.** `compare_silhouettes()` classifies each pixel of each frame as sky or not and compares the two classifications. Shading may differ legitimately between raster and ray-march.
- **Flat sky.** The classifier uses one reference colour, so every compared scene forces `sky_horizon = sky_zenith` before rendering.
- **Step 1c.** Before rendering, `assert_no_material_matches_sky()` checks that no material can shade into the sky colour at any achievable N·L. A collision makes every count meaningless.
- **Every band proves its detection surface** (`SR-30`). The reference is re-rendered with all geometry except the ground removed (`strip_to_ground_plane_only()`), and the case records and re-checks whether its band would catch that. A world with an infinite ground can saturate the frame, so that a band is non-discriminating. Such a band is labelled, never hidden.
- **Bands are versioned** (`SR-31`). A re-measure adds a measurement version and bumps the active selector. It never edits a committed measurement in place, and every recorded number is checked live.
- **Measured, then pinned, with provenance.** Bands are never invented.
- **The matrix** (`AgreementMatrix`) runs on golden worlds this repository owns: `gate`, `maximal` and `shower`. Cameras live in the band file, so a changed camera is a new version. A case frames geometry against the flat sky, because a frame saturated by an infinite ground cannot discriminate. The band is `round_up(0.001, max(0.001, 1.5 d))` (TD-2, Test/Docs), and d must match bit for bit across msvc-release, msvc-debug and gcc-release. `SPADE_AGREEMENT_MEASURE` writes each case's values and frames under the build tree for a new version.
- **A band never covers a known defect.** A disagreement the frames show to be a raster defect is recorded as debt and kept out of view, with a `_framing_note` saying so. It is not absorbed into a band.

## GPU paths

- **Raster vs ray-march, and CPU vs GPU, are bands, not byte equality.** Each step from fp64 to fp32 and across drivers can move a byte. Holding them to equality reports accepted numerics as regressions.
- **Coverage alone cannot see a colour defect.** The Vulkan raster's first host-side run found an R/B swap, a lit grid line, and an ignored unlit flag, none of which a silhouette comparison can detect. **Before any GPU technique is selectable, it needs a colour oracle:** per-pixel comparison against the CPU reference within a measured tolerance, on a scene with a known palette.
- **No GPU raster in `test_gpu_parity.cpp`.** Those tables compare quantities that are fp32 on both sides. A GPU raster against an fp64 reference belongs in its own banded suite.

## Writing a render test

- Compare a pixel against an **independent oracle evaluated for that pixel**, never one pixel against another. Two pixels that match can both be wrong.
- Give every monotone, bounded or "no change" assertion a **positive control** that shows the assertion can fail.
- Prefer **unlit materials** for coverage and clipping tests, so lighting cannot hide or fake a result.
- Avoid libm transcendentals and `glm::angleAxis` in anything that feeds a golden. Spell quaternions as literal float32 components, or use `spade::math`'s fp32 kernels. Golden-feeding test sources are scanned for this.
