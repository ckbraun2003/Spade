# The `SL2b` guard: plan

**Owner:** Interface (the subject, and packaging), with Test/Docs (the harness, `SL18` obligation 6) and Rendering (`render_gl`). **Status:** approved by the lead (2026-10-03) with the decision and the condition below. Nothing is built yet.

`SL2b` (signed 2026-09-17): the sandbox may include only headers the engine installs, and link only `spade::` targets. Needing more is a finding about the library. Today the rule is kept by review only (`../07-status.md`).

## What a guard has to catch

- **Name scans are not enough.** In a build tree, `spade::core` exports the whole `engine/` directory as an include root (`engine/CMakeLists.txt:118-121`). So any engine header compiles, installed or not. Slot 2 showed it: a GL-off tree still compiled `render_gl/gl_renderer.hpp` and failed only at link.
- **The only faithful check is a build against an installed prefix,** where the include root is `include/` and only `spadeTargets` exist.

## The sandbox breaks `SL2b` today

With `SPADE_RENDER_GL=ON`, the default, `sandbox/main.cpp` includes `render_gl/gl_renderer.hpp` and `sandbox/CMakeLists.txt` links `spade::render_gl`. Neither is installed: `spade_render_gl` has no install rule and is not in `spadeTargets` (`engine/CMakeLists.txt:510-545`). So the editor's GL path (`RND-4`: "GL stays the editor's path") rests on a module that no out-of-tree consumer can reach.

**Decided by the lead (2026-10-03): install `spade::render_gl` as an optional module**, present when `SPADE_RENDER_GL` is on. `RND-4` keeps GL as the editor's path, and Kat's editor may want it too. Exempting it in `SL2b`'s text would need the user's word, so that is not the route.

**The lead's condition: no glad symbol may leave the archive.** Compiling glad into a static archive, as `spade_compute` does with `volk`, would export global `glad_*` symbols, and a consumer that links its own glad would then get duplicate symbols. So it must be shown, with a consumer that links a second loader, that no clash is possible.

**How (Rendering's proposal, agreed with Interface): `render_gl` stops linking glad at all.**
- `GlRenderer::create()` already takes a `GlProcLoader`. It loads the GL functions it uses (about 40) into a private table in its `Impl`, not into glad's globals. No `glad_gl*` or `gladLoad*` symbol is defined or referenced by the archive.
- glad stays a private, header-only dependency of `gl_renderer.cpp`, for typedefs and enums. It enters through `target_include_directories(... PRIVATE $<BUILD_INTERFACE:...glad/include>)`, never by linking the `glad` target: even a `PRIVATE` link of a static library puts `$<LINK_ONLY:glad>` in the export, which `install(EXPORT)` refuses. No installed header includes GL: `gl_renderer.hpp` already includes none.
- `glm` gets the `BUILD_INTERFACE` treatment `spade_core` uses, so the export carries no non-exported target.
- Only `test_gpu_gl_renderer.cpp` calls GL directly (an FBO, a readback). It links glad itself, which is fine: tests are not installed.

**The seam:** Rendering writes the code (its module) first. Interface then writes the install rule, the `spadeTargets` entry and the consumer proof.

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

1. **Rendering:** the private GL table in `render_gl`; glad header-only, with no link. Proof: `render_gl`'s GL tests still pass on the GPU, and `test_gpu_gl_renderer.cpp` links glad itself.
2. **Interface: the install.**
   - `install(TARGETS spade_render_gl EXPORT spadeTargets ...)` with `EXPORT_NAME render_gl`, and `render_gl/*.hpp` to `include/render_gl`, only when `SPADE_RENDER_GL` is on. `spadeConfig.cmake` documents the optional target.
   - `tests/consumer` links `spade::render_gl` when the prefix has it. It calls `GlRenderer::create()` with a loader that returns null and expects a refusal, not a crash. It also links a second loader, its own glad, and calls it. A clean link and run is the no-clash proof.
   - `consumer-smoke.sh` checks, in the leg, that `nm -g --defined-only` on the installed `render_gl` archive shows no `glad` symbol.
3. `sandbox/CMakeLists.txt` standalone mode. In-tree behaviour must stay byte-for-byte the same, so the GL-off and GL-on in-tree builds both still pass.
4. `consumer-smoke.sh --sandbox`, with the probe and a stage line per step (Interface). Proof: green on the current tree; red when a scratch sandbox edit includes `testing/replay.hpp`; and the probe itself red if `testing/` were installed.
5. The leg calls the stage (Test/Docs).
6. `07-status.md`: the `SL2b` row becomes "Built and gated".
