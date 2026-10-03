# The `SL2b` guard: plan

**Owner:** Interface (the subject, and packaging), with Test/Docs (the harness, `SL18` obligation 6) and Rendering (`render_gl`). **Status:** draft for the lead. Nothing is built yet.

`SL2b` (signed 2026-09-17): the sandbox may include only headers the engine installs, and link only `spade::` targets. Needing more is a finding about the library. Today the rule is kept by review only (`../07-status.md`).

## What a guard has to catch

- **Name scans are not enough.** In a build tree, `spade::core` exports the whole `engine/` directory as an include root (`engine/CMakeLists.txt:118-121`). So any engine header compiles, installed or not. Slot 2 showed it: a GL-off tree still compiled `render_gl/gl_renderer.hpp` and failed only at link.
- **The only faithful check is a build against an installed prefix,** where the include root is `include/` and only `spadeTargets` exist.

## The sandbox breaks `SL2b` today

With `SPADE_RENDER_GL=ON`, the default, `sandbox/main.cpp` includes `render_gl/gl_renderer.hpp` and `sandbox/CMakeLists.txt` links `spade::render_gl`. Neither is installed: `spade_render_gl` has no install rule and is not in `spadeTargets` (`engine/CMakeLists.txt:510-545`). So the editor's GL path (`RND-4`: "GL stays the editor's path") rests on a module that no out-of-tree consumer can reach.

Two ways out (decision for the lead, with Rendering):
1. **Recommended: install `spade::render_gl`** as an optional installed module, present when `SPADE_RENDER_GL` is on.
   - It links `glad` publicly today, and `install(EXPORT)` refuses a non-exported link target. So `glad` is handled the way `spade_compute` handles `volk`: its source is compiled into the archive, and its headers stay `BUILD_INTERFACE`.
   - `GlRenderer` takes a loader (`GlProcLoader`), so a consumer needs no GLFW.
   - Interface owns install rules (`02-realms`); Rendering reviews.
2. **Alternative: exempt `render_gl` in `SL2b`'s text.** That edits a signed ruling, so it is the user's decision. It also leaves Kat or any other consumer without the GL path.

## The guard

**A sandbox build against the installed prefix**, as a stage of `scripts/consumer-smoke.sh`, which the Docker leg already runs:
1. **`sandbox/CMakeLists.txt` works in two modes.**
   - In-tree, as today.
   - Standalone: with no `spade::core` target defined, it calls `find_package(spade CONFIG REQUIRED)` and fetches GLFW and ImGui at the pinned tags `vendor/` uses. It does this the way `tests/consumer/CMakeLists.txt` fetches glm and yaml-cpp.
2. **`consumer-smoke.sh --sandbox`** configures and builds `sandbox/` against the prefix it just installed, then runs `spade_sandbox --headless`.
   - Without X11 in the leg, GLFW has no backend, so the build takes the `SPADE_SANDBOX_HAS_GL=0` path. The CPU frame still renders.
   - Any include of an uninstalled header, or any link to an unexported target, fails here.
3. **A seeded violation on every run** (`SL18`): the stage also compiles a one-line probe TU against the same prefix, `#include "testing/replay.hpp"`, a header the engine does not install, and requires the compile to fail. If the probe compiles, the prefix is leaking headers and the guard is blind; the stage fails and says so.
4. **The Docker leg calls the stage** (Test/Docs's script) for Vulkan ON. The OFF prefix ships fewer headers, so it calls it there too.

## Tasks

1. The `render_gl` decision (lead and Rendering; the user if option 2).
2. If option 1: `render_gl` install rules, `glad` compiled into the archive, and `spadeConfig.cmake` docs. `tests/consumer` gains a link to `spade::render_gl`. Interface builds it; Rendering reviews.
3. `sandbox/CMakeLists.txt` standalone mode. In-tree behaviour must stay byte-for-byte the same, so the GL-off and GL-on in-tree builds both still pass.
4. `consumer-smoke.sh --sandbox`, with the probe and a stage line per step (Interface). Proof: green on the current tree; red when a scratch sandbox edit includes `testing/replay.hpp`; and the probe itself red if `testing/` were installed.
5. The leg calls the stage (Test/Docs).
6. `07-status.md`: the `SL2b` row becomes "Built and gated".
