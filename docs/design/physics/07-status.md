# Physics — status

**The only place that says what exists today.** Checked against the tree at `master` `332a186` on 2026-10-02, by reading the code, CMake and tests. Per-file counts are `TEST` macros in the source. The measured suite is Test/Docs's baseline: 921 tests on both presets at `df33f09`, every physics and GPU test passing (`../test-docs/07-status.md`).

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| Integrator (`engine D1`) | **Built**, CPU and Vulkan | `physics/integrator.*`, `integrate.slang`; `test_integrator.cpp` (11) |
| Static contact, sphere proxy vs SDF (`engine D3`, `D-S6-2`) | **Built**, CPU and Vulkan. Linear response only, one contact per body, no CCD | `physics/contacts.*`, `collision_static.slang`; `test_contacts.cpp` (17) |
| Dynamic contact, sorted grid (`engine D3`) | **Built**, Gauss-Seidel, CPU and Vulkan. The Jacobi variant is built and tested but **not wired** (CPU `resolve_dynamic_contacts_jacobi`; GPU `collision_fill`/`collision_gather` compiled, not dispatched) | `physics/grid.*`, `grid_build`/`grid_sort`/`collision_dynamic.slang`; `test_grid.cpp` (26) |
| Capsule proxies (`engine D3`) | **Not built** | — |
| Mesh and convex colliders | **Not built** (roadmap) | — |
| Drag, aero T0 (quadratic, componentwise) | **Built**, CPU and Vulkan | `physics/forces.*`, `forces_drag.slang`; `test_forces.cpp` (13) |
| `LiftSurface` (aero T1), BEMT (T2), coupled (T3) | **Not built** | — |
| Rotor element (`engine D5`) | **Built**, CPU and Vulkan. Thrust and torque are quadratic in ω (`k_T`, `k_Q`); `engine D5`'s "curves (polynomial or table)" is not built | `vehicles/rotor.*`, `rotors.slang`; `test_rotor.cpp` (30) |
| Model types, quadrotor (`engine D4`) | **Built**. The quadrotor is still engine code (`vehicles/quadrotor.*`), not yet a template | `vehicles/model_type.*`, `quadrotor.*`; `test_quadrotor.cpp` (22) |
| Medium: density, wind, Dryden (`engine D6`) | **Built**, CPU and Vulkan. Position-independent | `world/medium.*`, `medium_update.slang`, `dryden.slang`; `test_dryden.cpp` (22), `test_rng_medium.cpp` (26) |
| Medium temperature (`engine D6`) | **Not built** | — |
| IMU | **Built**, CPU and Vulkan | `sensors/imu.*`, `sensor_imu.slang`; `test_imu.cpp` (17) |
| GNSS | **Built**, CPU and Vulkan | `sensors/gnss.*`, `sensor_gnss.slang`; `test_gnss.cpp` (20) |
| SPH field provider (`SL8`) | **Not built.** Paused by the restructure; the one open transfer-register row | v1: `src/Core/Engine.cpp`, `assets/shaders/[SYSTEM]Fluid*.comp` |
| Rotor wake, visualisation (`PHY-3`) | **Built**, CPU only (merge `84e435b`). Read only by the drone sim box's heatmap (`sandbox/drone_view.hpp`, merge `df33f09`) | `vehicles/rotor_wake.*`; `test_rotor_wake.cpp` (14) |
| Golden corpus | `ballistic`, `bounce`, `quad_hover`, `shower`, `two_world_isolation` | `tests/golden/scenarios/` |
| CPU↔GPU parity and invariance | **Built**; bands per scenario, measured on the developer GPU | `testing/parity.hpp`; `test_gpu_parity.cpp` (31), `test_gpu_invariance.cpp` (14) |
| Grades (`PHY-2`, signed) | **Declared in prose only** (`04-verification.md`); the engine has no grade check yet (Core) | — |

## Open items — needs a user decision

1. **A GNSS golden scenario.** No corpus scenario carries a receiver, so the CPU GNSS is not reference grade under `L4` (`PHY-2`). Adding one moves no existing digest.
2. **The Jacobi dynamic-contact path:** adopt it (re-pin the affected bands and goldens, with provenance) or delete it. Today it is built and tested code that nothing runs.
3. **When SPH resumes** as a field provider. The restructure pauses it until Core's module API and scheduler exist.

## Open items — debt

| Item | Detail |
|---|---|
| **A resting body's IMU reads ~0 specific force** | Contact impulses bypass `force_acc`. In-flight readings are correct. Documented in `imu.hpp`; it waits on the contact model's next fidelity step |
| **Rotor torque is uncorrected by inflow and ground** | Thrust is corrected; `Q = k_Q ω²` is not, so `Q` is not a power budget. Documented in `rotor.hpp`; it waits on BEMT |
| **No contact torque, manifold or CCD** | `contacts.hpp` states each limit |
| **The drone stand cannot run on Vulkan** | It pins translation with CPU behaviors, and a Vulkan step now refuses an attached registry (`CORE-1`) rather than skipping it. A translation-lock constraint on both backends is `../backlog.md` (Core / Physics) |
| **GPU parity runs only where the gate has a device** | On this box all 65 `gpu` tests run inside `scripts\test.ps1`; a machine without a device skips them. Whether the gate should require a device is Test/Docs's open decision |

## Next for Physics

1. **SPH as a field provider** (`SL8`), once Core's module API and scheduler exist. Its plan goes in `plans/`.
2. **The translation lock**, with Core: a constraint on both backends that lets the drone stand run on Vulkan.
3. **The two user decisions above:** the GNSS golden, and the Jacobi path.

## Corrections to earlier records

- 2026-10-02: the rotor-wake suite was reported as 15 tests; it is 14.

- 2026-10-01: `engine D3` names capsule proxies and `engine D6` names a temperature field. Neither was ever built; the consolidation's status table marked both decisions "BUILT" without the caveat.
