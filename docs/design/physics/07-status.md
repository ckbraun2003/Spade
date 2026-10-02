# Physics — status

**The only place that says what exists today.** Checked against the tree at `master` `7637f11` on 2026-10-01, by reading the code, CMake and tests. No build tree existed in this checkout, so nothing below is a measured test result. Test counts are `TEST` macros in the source, not a ctest tally. Unmerged branches are named where they change a row. The rotor-wake row was updated after its merge (`84e435b`).

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
| Rotor wake, visualisation (`PHY-3`) | **Built**, CPU only, on master (merge `84e435b`). Measured at branch head `34291ba` in the Physics worktree's `build-ninja/release`: `RotorWake` 15/15; full suite 911 run, 0 failed, 33 skipped (31 KAT-reach, 2 by design) | `vehicles/rotor_wake.*`; `test_rotor_wake.cpp` (15) |
| Golden corpus | `ballistic`, `bounce`, `quad_hover`, `shower`, `two_world_isolation` | `tests/golden/scenarios/` |
| CPU↔GPU parity and invariance | **Built**; bands per scenario, measured on the developer GPU | `testing/parity.hpp`; `test_gpu_parity.cpp` (30), `test_gpu_invariance.cpp` (14) |
| Grades (`PHY-2`) | **Declared in prose only** (`04-verification.md`); the engine has no grade check yet (Core) | — |

## Open items — needs a user decision

1. **Sign `PHY-2`** (the grade table) and `PHY-3` (the promotion rule for effects). The gaussian band `PHY-1` was moved to Core as `CORE-3` (lead's ruling, 2026-10-01).
2. **The Jacobi dynamic-contact path:** adopt it (re-pin the affected bands and goldens, with provenance) or delete it. Today it is built and tested code that nothing runs.
3. **When SPH resumes** as a field provider. The restructure pauses it until Core's module API and scheduler exist.

## Open items — debt

| Item | Detail |
|---|---|
| **GNSS has no golden** | No corpus scenario carries a receiver, so the CPU GNSS is not reference grade under `L4`. Adding one moves no existing digest |
| **A resting body's IMU reads ~0 specific force** | Contact impulses bypass `force_acc`. In-flight readings are correct. Documented in `imu.hpp`; it waits on the contact model's next fidelity step |
| **Rotor torque is uncorrected by inflow and ground** | Thrust is corrected; `Q = k_Q ω²` is not, so `Q` is not a power budget. Documented in `rotor.hpp`; it waits on BEMT |
| **No contact torque, manifold or CCD** | `contacts.hpp` states each limit |
| **The drone stand cannot run on Vulkan** | It pins translation with CPU behaviors. A translation-lock constraint on both backends is `../backlog.md` (Core / Physics) |
| **GPU parity is developer-machine-only** | No gate has a device (`04-verification.md`) |

## Corrections to earlier records

- 2026-10-01: `engine D3` names capsule proxies and `engine D6` names a temperature field. Neither was ever built; the consolidation's status table marked both decisions "BUILT" without the caveat.
