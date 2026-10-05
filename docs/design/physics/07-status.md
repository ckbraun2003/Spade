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
| IMU | **Built**, CPU and Vulkan. Reads the contact response (`PHY-7`) | `sensors/imu.*`, `sensor_imu.slang`; `test_imu.cpp` (20) |
| GNSS | **Built**, CPU and Vulkan. The CPU path has a golden, `gnss_tumble` (`PHY-6`) | `sensors/gnss.*`, `sensor_gnss.slang`; `test_gnss.cpp` (20) |
| SPH field provider (`SL8`) | **Not built.** Resumes right after Core's module API lands (`PHY-4`); the one open transfer-register row | v1: `src/Core/Engine.cpp`, `assets/shaders/[SYSTEM]Fluid*.comp` |
| Rotor wake, visualisation (`PHY-3`) | **Built**, CPU only (merge `84e435b`). Read only by the drone sim box's heatmap (`sandbox/drone_view.hpp`, merge `df33f09`) | `vehicles/rotor_wake.*`; `test_rotor_wake.cpp` (14) |
| Drone builder: motor and ESC, battery and bus, propeller tier (`DBP-01`..`33`) | **Built as pure functions**, float and double, CPU. `DBP-26`, the 4-in-1 ESC's total-current clamp, came with the airframe compile: `bus_solve` holds the motors' total drive current at `esc_current_total_max` through the same common duty scale and sets `esc_total_limited`. Braking current does not count toward it. Not in the step: the stateful rows wait on module-API stage 4, the kernels and goldens on the rows | `vehicles/motor.*`, `battery.*`, `propeller.*`, `propulsion_flags.hpp`; `test_motor.cpp` (13), `test_battery.cpp` (19), `test_propeller.cpp` (8) |
| Drone builder: composite inertia, steady-state solver (`DBP-40`..`43`, `50`..`52`) | **Built**, host utilities in double, never in the step. `composite_inertia_issues` lists every bad part; `steady_state_time_constant` gives the chain's small-signal τ | `vehicles/composite_inertia.*`, `propulsion_steady.*`; `test_composite_inertia.cpp` (8), `test_propulsion_steady.cpp` (8) |
| Drone builder: the airframe compile (`DBP-44`) | **Built**, a host utility in double. Mass, inertia and mounts come exactly from the parts. The momentum rotor's `k_T`, `k_Q` and τ are fitted to the chain at hover (best-effort). Drag is estimated from parts when none is given; supplied componentwise drag is mapped onto body axes. One `EscBlock` is one board, and until airframes carry several boards, a board with a total must drive every rotor. The reference quad's `model_identity` pin is final under `TD-12` | `vehicles/airframe_compile.*`; `test_airframe_compile.cpp` (19) |
| Golden corpus | `ballistic`, `bounce`, `gnss_tumble`, `quad_hover`, `shower`, `two_world_isolation`. `gnss_tumble` is the only one with GNSS receivers. The Docker gcc leg reproduced all six digests on its first full run (`fe4934a`), so `gnss_tumble` is final under `TD-12` | `tests/golden/scenarios/` |
| CPU↔GPU parity and invariance | **Built**; bands per scenario, measured on the developer GPU | `testing/parity.hpp`; `test_gpu_parity.cpp` (32), `test_gpu_invariance.cpp` (15) |
| Grades (`PHY-2`, signed) | **Declared in prose only** (`04-verification.md`); the engine has no grade check yet (Core) | — |

## Ruled by the user (2026-10-02)

The three decisions this page held are ruled; the rulings live in `00-decisions.md`. No user decision is open in this realm.

1. **A GNSS golden scenario: now** (`PHY-6`). Built as `gnss_tumble`. No existing digest moved. The Docker gcc leg reproduced it (`fe4934a`), so it is final under `TD-12`.
2. **The Jacobi dynamic-contact path: kept unwired** (`PHY-5`). It returns as an alternative contact module with its own declared grade once modules exist.
3. **SPH: right after Core's module API lands** (`PHY-4`), as the first field provider that is not a built-in.

## Open items — debt

| Item | Detail |
|---|---|
| **Rotor torque is uncorrected by inflow and ground** | Thrust is corrected; `Q = k_Q ω²` is not, so `Q` is not a power budget. Documented in `rotor.hpp`; it waits on BEMT |
| **No contact torque, manifold or CCD** | `contacts.hpp` states each limit |
| **The drone stand cannot run on Vulkan** | It pins translation with CPU behaviors, and a Vulkan step now refuses an attached registry (`CORE-1`) rather than skipping it. A translation-lock constraint on both backends is `../backlog.md` (Core / Physics) |

## Next for Physics

1. **The NVIDIA denormal probe kernel**, M1 of Core's `../core/plans/2026-10-04-nvidia-denorm-measurement-plan.md`.
2. **The translation lock**, with Core: Physics's requirements are in `plans/2026-10-02-module-api-requirements.md`. Core builds the lock with the module API.
3. **SPH as a field provider** (`PHY-4`, `SL8`), once the module API lands. Its plan goes in `plans/`.
4. **Jacobi as a contact module** (`PHY-5`), once modules exist.

## Corrections to earlier records

- 2026-10-03: the builder functions were recorded as built for `DBP-01`..`33`, and their plan claimed `DBP-20`..`26`. `DBP-26`, the 4-in-1 ESC's total-current clamp, was never built: `bus_solve` had only the pack-current and cutoff limits. It was added with the airframe compile on 2026-10-04.

- 2026-10-02: the rotor-wake suite was reported as 15 tests; it is 14.

- 2026-10-01: `engine D3` names capsule proxies and `engine D6` names a temperature field. Neither was ever built; the consolidation's status table marked both decisions "BUILT" without the caveat.
