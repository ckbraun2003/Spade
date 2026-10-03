# Physics — status

**The only place that says what exists today.** Checked against the tree at `master` `332a186` on 2026-10-02, by reading the code, CMake and tests, and re-checked at `42ce419`: nothing outside `docs/` changed between the two. Per-file counts are `TEST` macros in the source. The measured suite is the lead's run at `88a3c8b`: 921 tests on both presets, 919 passed, 2 skipped by design, 0 failed, and all 65 `gpu` tests ran (`../test-docs/07-status.md`). The GNSS golden's branch measured 925 tests on release: 923 passed, 2 skipped by design, 0 failed, 67 `gpu` ran (`physics/gnss-golden`, 2026-10-03).

## Specified vs built

| Specified | Built | Where |
|---|---|---|
| Integrator (`engine D1`) | **Built**, CPU and Vulkan | `physics/integrator.*`, `integrate.slang`; `test_integrator.cpp` (11) |
| Static contact, sphere proxy vs SDF (`engine D3`, `D-S6-2`) | **Built**, CPU and Vulkan. Linear response only, one contact per body, no CCD | `physics/contacts.*`, `collision_static.slang`; `test_contacts.cpp` (17) |
| Dynamic contact, sorted grid (`engine D3`) | **Built**, Gauss-Seidel, CPU and Vulkan. The Jacobi variant is built and tested but **not wired** (CPU `resolve_dynamic_contacts_jacobi`; GPU `collision_fill`/`collision_gather` compiled, not dispatched), and stays so until it returns as a contact module (`PHY-5`) | `physics/grid.*`, `grid_build`/`grid_sort`/`collision_dynamic.slang`; `test_grid.cpp` (26) |
| Capsule proxies (`engine D3`) | **Not built** | — |
| Mesh and convex colliders | **Not built** (roadmap) | — |
| Drag, aero T0 (quadratic, componentwise) | **Built**, CPU and Vulkan | `physics/forces.*`, `forces_drag.slang`; `test_forces.cpp` (13) |
| `LiftSurface` (aero T1), BEMT (T2), coupled (T3) | **Not built** | — |
| Rotor element (`engine D5`) | **Built**, CPU and Vulkan. Thrust and torque are quadratic in ω (`k_T`, `k_Q`); `engine D5`'s "curves (polynomial or table)" is not built | `vehicles/rotor.*`, `rotors.slang`; `test_rotor.cpp` (30) |
| Model types, quadrotor (`engine D4`) | **Built**. The quadrotor is still engine code (`vehicles/quadrotor.*`), not yet a template | `vehicles/model_type.*`, `quadrotor.*`; `test_quadrotor.cpp` (22) |
| Medium: density, wind, Dryden (`engine D6`) | **Built**, CPU and Vulkan. Position-independent | `world/medium.*`, `medium_update.slang`, `dryden.slang`; `test_dryden.cpp` (22), `test_rng_medium.cpp` (26) |
| Medium temperature (`engine D6`) | **Not built** | — |
| IMU | **Built**, CPU and Vulkan | `sensors/imu.*`, `sensor_imu.slang`; `test_imu.cpp` (17) |
| GNSS | **Built**, CPU and Vulkan. The CPU path has a golden, `gnss_tumble` (`PHY-6`) | `sensors/gnss.*`, `sensor_gnss.slang`; `test_gnss.cpp` (20) |
| SPH field provider (`SL8`) | **Not built.** Resumes right after Core's module API lands (`PHY-4`); the one open transfer-register row | v1: `src/Core/Engine.cpp`, `assets/shaders/[SYSTEM]Fluid*.comp` |
| Rotor wake, visualisation (`PHY-3`) | **Built**, CPU only (merge `84e435b`). Read only by the drone sim box's heatmap (`sandbox/drone_view.hpp`, merge `df33f09`) | `vehicles/rotor_wake.*`; `test_rotor_wake.cpp` (14) |
| Golden corpus | `ballistic`, `bounce`, `gnss_tumble`, `quad_hover`, `shower`, `two_world_isolation`. `gnss_tumble` is the only one with GNSS receivers. Its digest is provisional until the Docker gcc leg reproduces it (`TD-12`) | `tests/golden/scenarios/` |
| CPU↔GPU parity and invariance | **Built**; bands per scenario, measured on the developer GPU | `testing/parity.hpp`; `test_gpu_parity.cpp` (32), `test_gpu_invariance.cpp` (15) |
| Grades (`PHY-2`, signed) | **Declared in prose only** (`04-verification.md`); the engine has no grade check yet (Core) | — |

## Ruled by the user (2026-10-02)

The three decisions this page held are ruled; the rulings live in `00-decisions.md`. No user decision is open in this realm.

1. **A GNSS golden scenario: now** (`PHY-6`). Built as `gnss_tumble`. No existing digest moved. Under `TD-12` its digest is provisional until the Docker gcc leg reproduces it.
2. **The Jacobi dynamic-contact path: kept unwired** (`PHY-5`). It returns as an alternative contact module with its own declared grade once modules exist.
3. **SPH: right after Core's module API lands** (`PHY-4`), as the first field provider that is not a built-in.

## Open items — debt

| Item | Detail |
|---|---|
| **A resting body's IMU reads ~0 specific force** | Contact impulses bypass `force_acc`. In-flight readings are correct. Documented in `imu.hpp`; it waits on the contact model's next fidelity step |
| **Rotor torque is uncorrected by inflow and ground** | Thrust is corrected; `Q = k_Q ω²` is not, so `Q` is not a power budget. Documented in `rotor.hpp`; it waits on BEMT |
| **No contact torque, manifold or CCD** | `contacts.hpp` states each limit |
| **The drone stand cannot run on Vulkan** | It pins translation with CPU behaviors, and a Vulkan step now refuses an attached registry (`CORE-1`) rather than skipping it. A translation-lock constraint on both backends is `../backlog.md` (Core / Physics) |

## Next for Physics

1. **The GNSS golden's cross-check** (`TD-12`): record the Docker gcc leg's result in `gnss_tumble`'s provenance when the leg exists.
2. **The translation lock**, with Core: Physics's requirements are in `plans/2026-10-02-module-api-requirements.md`. Core builds the lock with the module API.
3. **SPH as a field provider** (`PHY-4`, `SL8`), once the module API lands. Its plan goes in `plans/`.
4. **Jacobi as a contact module** (`PHY-5`), once modules exist.

## Corrections to earlier records

- 2026-10-02: the rotor-wake suite was reported as 15 tests; it is 14.

- 2026-10-01: `engine D3` names capsule proxies and `engine D6` names a temperature field. Neither was ever built; the consolidation's status table marked both decisions "BUILT" without the caveat.
