# Spade — realms

**Owner:** lead. **Status:** signed by the user 2026-10-02 (approved 2026-10-01; `plans/2026-10-01-spade-restructure-design.md` §3).

| Realm | Library | Owns | Code today |
|---|---|---|---|
| **Core** | `core/` | Module API and registry; scheduler; state, snapshot, replay and config hash; world, region and object model; field registry and sampling interface; grades; backend seam and GPU-chain derivation; `Simulation`/`WorldSet` API; world-file schema; math, RNG, `Result` | `engine/core/` `state/` `sim/` `objects/` `world/` (description side) `compute/` (seam, context, mirror, recorder) |
| **Physics** | `physics/` | Field providers (gravity, media, turbulence, flow solvers including SPH); responders (integration, contact, drag and aero tiers, propulsor, buoyancy); physical sensors (IMU, GNSS); their kernels and parity bands | `engine/physics/` `vehicles/` `sensors/` `world/medium` and the physics kernels in `shaders/`; v1's SPH as the reference until it is replaced |
| **Rendering** | `rendering/` | Techniques (raster, ray-trace, ray-march); channels (colour, depth, IDs, field visualisation); camera component and camera sensor; scene representation; CPU, GL and Vulkan backends; render goldens and bands | `engine/render/` `render_gl/` `render/vulkan/` and the raster kernels |
| **Interface** | `interface/` | The editor (the sandbox grows into it); viewer; templates and examples; packaging, install, export and the consumer project; CLI and demos; user guides | `sandbox/` `engine/tools/` `examples/` (v2) `tests/consumer/` `cmake/spadeConfig*` and the install rules |
| **Test/Docs** | `test-docs/` | How things are verified and documented: harness, golden-corpus governance, parity harness, SPIR-V scanner, bench; build and gate tooling; the structure of the docs | `tests/` (each realm writes its own tests) `engine/testing/` `scripts/` the build parts of `cmake/` and the structure of `docs/` |
| **Lead** | this directory | Charter, engine model, realm map, register, index, backlog; cross-realm sequencing; build scheduling; review and merge | — |

## How realms meet

- **Core and the module realms.** Physics and Rendering reach Core only through the module API. A change to the scheduler, state or snapshot is a Core decision.
- **Physics and Rendering** meet only through fields and the published frame state.
- **Interface** uses only the installed public API. When the editor needs more, that is a finding for Core.
- **Test/Docs** sets verification rules and gates. Each realm owns its own tests and specs.
- **The laws** (`00-charter.md`) change only with the user's signature.

## Working together

- **Docs-only work** happens in the main tree.
- **Code work** happens in a per-realm git worktree (`../spade-wt/<realm>`, its own build directory), on a branch the lead reviews and merges.
- **Builds** run in the foreground, in slots the lead hands out. Spade shares its build machine with Kat under one budget (`consumers.md`, "Shared build machine").
- **Nothing is pushed** without the user's word.
