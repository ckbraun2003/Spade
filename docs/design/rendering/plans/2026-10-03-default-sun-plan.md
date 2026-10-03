# Default sun above the horizon (`RND-5`): plan

**Owner:** Rendering. **Status:** draft, for the lead's review. It runs once the Docker leg (`TD-11`) has merged and run end to end, because a regenerated golden is final only when that leg reproduces it (`TD-12`).

## Goal

`render::Lighting{}`'s default `sun_direction` is `{-0.35, -0.86, -0.37}`. Under `RND-1` the vector points toward the sun, so this sun is below the horizon, and every hand-built scene that keeps the default is lit from below. After this plan, the default points above the horizon. The four frame goldens that shade under it move once, deliberately, with each new hash measured on Windows/MSVC and reproduced on Linux/gcc before it is final.

World-loaded scenes do not change: `scene_from_world()` takes `LightingDesc`'s own default, which is already correct.

## The new value (Q1)

**Recommended: the world's default, by the world's own expression.** The member initializer becomes `glm::normalize(LightingDesc{}.sun_direction)`, the same expression `scene_from_world()` applies (`render/scene.cpp`). That is about (0.371, 0.743, 0.557). The two defaults then cannot drift apart, and a hand-built scene shades as a default world does.

**Alternative: negate the old value**, to about (0.35, 0.86, 0.37). It keeps the old elevation and mirrors it above the horizon. It is closer to what the R6 fixtures were first lit by in intent, but it leaves two defaults in the engine.

## What moves

| Frame | Moves | Why |
|---|---|---|
| `box_shaded_with_overlays` | yes | a lambert box under the default |
| `cylinder_static_dynamic_box_top_down` | yes | a lambert cylinder and box |
| `shadowed_ground_with_caster` | yes | a lambert ground and caster, and the shadow map's light direction |
| `shadowed_ground_with_caster_atmospheric` | yes | the same scene, with `SR-17a` on |
| `sphere_wireframe_no_overlays` | **no: the control** | wireframe is unlit, and the sky does not depend on the sun |

**No other test should move.** Every other render test that lights a lambert surface sets its own sun (the shadow, shading, field and GL suites). The rest compare coverage only (agreement, ray-march), or check unlit colours, or recompute expectations through `shade_vertex_color()` with the scene's own lighting. **If any other test fails, that is a finding:** stop and report it, and never regenerate it in this branch.

**Two in-test checks must still hold before any hash is read:**
- **`SR-17a` pair:** its two frames differ by more than 1000 bytes.
- **Shadow sensitivity:** `shadowed_ground_with_caster` changes when shadows are off. With either value above, the caster's shadow lands on the ground at about (−1.25, 0, −1.87), inside the frame and the shadow map. Under today's sun, the ground's N·L is 0, so the ground shows ambient only and a shadow cannot darken it.

## Steps

1. **Branch.** Cut `rendering/default-sun` from `master`. Commit A changes the default and its comment in `render/scene.hpp`. The comment's line saying the old value is "kept only because hand-built frame goldens shade under it" goes.
2. **Slot.** Build, then run `RasterGolden`. Exactly four cases fail, and the wireframe frame passes. Record the four actual sha256 values from the failure output, all 64 characters.
3. **Commit B: the manifest.** Write the four new hashes, plus two keys:
   - `_changelog_rnd5`: the cause (`RND-5`), the old and new value, old → new hash per frame, the control, and the MSVC measurement with its date and preset.
   - `_pending_rnd5`: "measured on Windows only; not final until the Docker gcc leg reproduces it (`TD-12`)". This follows the `_pending_regeneration` precedent.

   Then run the full suite.
4. **Docker leg.** Run `scripts\docker-leg.ps1 -Commit <B>`. Every `RasterGolden` case must pass under gcc; that is the same 64 characters on both platforms.
   - **Before that run, nothing merges.**
   - **If gcc computes a different hash for any frame, promote nothing.** Report the frame and both hashes, and leave the branch unmerged. Two platforms disagreeing is a bigger finding than a stale golden (`03-verification.md`).
5. **Commit C.** Discharge `_pending_rnd5` with the gcc run's commit, date and toolchain. After the merge:
   - in `07-status.md`, close the debt row;
   - in `00-decisions.md`, mark `RND-5` done at its merge.

**Cost:** one wide rebuild (`scene.hpp`), one full suite, and one incremental Docker run. Test/Docs measures the leg at 10–20 min after its first run.

## Questions for the lead

- **Q1.** Which value: the world's default by its own expression (recommended), or the negated old value?
- **Q2.** Who runs the Docker leg on commit B? I can run Test/Docs' script in my own slot, or Test/Docs can run it.
- **Q3.** May `RasterGolden` gain an opt-in frame dump? It would be an environment variable naming a directory, writing PPM files, and off by default. Before and after images of the four frames would then go to review with the hashes, because a deliberate regeneration should be looked at, not only counted.
