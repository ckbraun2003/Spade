# The v1 → v2 transfer register

**Normative (24th spec SL7).** v1 may not be quarantined while it is the only
implementation of anything. Every v1 system below is dispositioned as
`transferred`, `to-transfer`, `retired-with-reason` or `retired-to-sandbox`.

> **A silent drop is not a disposition.** This file is machine-checked:
> `spade/tests/test_transfer_register.cpp` parses the table and fails if any row
> carries a disposition outside that set, if a row loses its evidence, or if the
> row count changes without the guard being updated. The check exists so that
> §7's quarantine cannot execute over a live gap.

The table is transcribed from the spec's SL7 register, which was itself verified
against the v1 tree rather than recalled.

## Disposition vocabulary

| Disposition | Meaning |
|---|---|
| `transferred` | A v2 implementation exists and is exercised by tests. |
| `to-transfer` | Still open. v1 remains the only implementation; quarantine is blocked. |
| `retired-with-reason` | Deliberately not transferred, with the reason recorded here. |
| `retired-to-sandbox` | Leaves the engine by design; the capability survives in the sandbox. |

## Register

| v1 system | v1 surface | disposition | evidence |
|---|---|---|---|
| Gravity | `EnableGravity` | `transferred` | Applied inside Integrate for exact specific-force capture; the Gravity pass is deliberately inert (engine design A9). Pinned by `Schedule.GravityIsAppliedExactlyOnce`. |
| Motion / integration | `EnableMotion`, `Motion.comp` | `transferred` | `physics/integrator.hpp` — symplectic Euler + quaternion exp-map. |
| Grid collision | `EnableGridCollision`, `Grid*.comp` | `transferred` | `physics/grid` + the CollisionDynamic pass; v1's hash-collision bug fixed by exact cell compare. |
| Bitonic sort | `BitonicSort.comp` | `transferred` | The sorted-grid chain; the Vulkan recorder records its dispatch chain once per shape and reuses it. |
| Wireframe / colour render | `RenderWireframe`, `RenderColor` | `transferred` | `DrawMode::wireframe` / `DrawMode::shaded` in `render/target.hpp`. |
| Newtonian gravity | `EnableBruteForceNewtonianGravity` | `retired-with-reason` | Already retired before this audit: the shader existed but the C++ body was never implemented. Deleted during S1 as documented dead weight. |
| Brute-force collision | `EnableBruteForceCollision` | `retired-with-reason` | SL9a. Superseded by the sorted-grid path, which is strictly better at every body count v1 shipped. Recorded so it is a decision rather than an omission. |
| Custom shader loading | `RenderShader(frag, geom)` | `retired-with-reason` | SL9b. A user-supplied fragment/geometry shader has no meaning against a CPU rasterizer. Reserved for the Vulkan backend at S7b; not a v2 gap today. |
| SPH fluid | `EnableSPHFluid`, `FluidDensity.comp`, `FluidForce.comp`, `FluidComponent`, `FluidMaterial` | `to-transfer` | SL8 — **THE ONE OPEN ROW.** v2 has no fluid solver of any kind, and this is what the 50,000-body fluid scene runs on. Plan B implements it to v2 discipline (both `execute_cpu` and `record_gpu`, measured parity band) rather than porting it. `ComponentTypeId::fluid` is reserved so Plan B does not renumber the enum. |
| Velocity render mode | `RenderVelocity`, `Velocity.frag` | `transferred` | SL9c, Plan A Task 10. `DrawMode::velocity` plus `RenderOptions::velocity_scale_mps`; `BodyPose::velocity` → `DrawItem::speed_mps`. Pinned by `test_render_velocity.cpp`. |
| Camera component | `CameraComponent` | `transferred` | SL9d, Plan A Task 2. `ComponentTypeId::camera` — one of SL5's ten, with `fov_degrees`/`near_plane`/`far_plane`/`active`. |
| Input component | `InputComponent` | `retired-to-sandbox` | SL9d. Key bindings and fly-through speed are an application concern; the engine is headless by construction (SL1) and must not grow an input concept. The capability survives as the sandbox's camera controller (Plan C). |
| Instancing helpers | `SpawnInstancesInSphere/Cube`, `SetVelocity/Color/Mass`, `RandomizeVelocity/Color` | `transferred` | SL9e, Plan A Task 11. `spawn_in_sphere`/`spawn_in_cube` with `mass` and `velocity_radius_mps`, seeded from domain-separated splitmix64 streams rather than v1's `std::random_device`-seeded mt19937. **Colour has no body-side equivalent to transfer to:** in v2 colour is a `RenderScene` `Material`, not a property of a body. |

## What "zero open rows" means

One row is open: **SPH fluid**. Until Plan B closes it, §7's quarantine of v1
must not execute — v1 is still the only implementation of the fluid solver, and
quarantining it would retire a live capability rather than a superseded one.
