# Physics — responders

**Owner:** Physics. **Normative.** A responder is a component that samples fields where its object is and produces forces (`../01-engine-model.md`). This file states each responder's model and limits; what is built is in `07-status.md`. Every header named below carries the derivation. This file is the map, not a copy.

## Rules for every responder

- **Pure and total.** A pass is a pure function of its spans, fields and parameters. It allocates nothing, reads no clock, and never turns a degenerate input into a NaN. A NaN in a wrench is invisible to every determinism guard (`../superseded/2026-09-consolidation/08-lessons.md` §5.3), so totality is the only defence.
- **Op order is the parity contract** (`engine D2`, Core). The numbered operation order in each `.cpp` is what its GPU kernel mirrors. Reordering arithmetic is a behaviour change even when it is algebraically identical, and it moves goldens.
- **Frames.** `force_acc` is world frame and `torque_acc` is body frame (`state/layout.hpp`). A responder adds to both and never clears them; Integrate clears them.
- **Transcendentals** come only from `core/fp32_math` on any path feeding state (the bit-portability rule, Test/Docs).

## Rigid body — integration (`engine D1`)

Symplectic Euler: velocities from the accumulated wrench, then pose. The quaternion advances by `q ⊗ exp(½ h ω)` and is renormalised every substep. Euler's term `ω × Iω` is included. Gravity is added inside Integrate (`engine A9`), which is also where **specific force** (acceleration minus gravity, body frame) is captured for the IMU. Accuracy comes from substeps, not order. `physics/integrator.*`.

## Contact (`engine D3`, `D-S6-2`)

- **Static:** each body is a sphere proxy against the world SDF. Depth is `φ(p) < r`, and the normal is the SDF gradient (analytic where it exists, otherwise a pinned central-difference stencil). The response is an impulse with restitution, Coulomb friction and clamped Baumgarte correction. The radius is the body's own `proxy_radius`, or the world default (`D-S6-2`). `physics/contacts.*`.
- **Dynamic:** v1's sorted-grid pipeline, atomics-free, with exact cell-ID compare (v1's hash-bucket bug fixed). Pairs resolve by Gauss-Seidel across all worlds. A Jacobi variant (`resolve_dynamic_contacts_jacobi`, plus the `collision_fill`/`collision_gather` kernels) is built but not wired in: switching to it changes pinned numbers. `physics/grid.*`.
- **Limits, stated in `contacts.hpp`:**
  - the response is linear only: no contact torque, so nothing rolls or tips;
  - one contact per body per substep, with no manifold;
  - no continuous collision detection, which is safe while `|v|·h < r`.
- **Roadmap (`engine D3`):** capsule proxies, then convex hulls and triangle-mesh statics. Contact torque and a manifold are the next fidelity step.

## Aero — fidelity tiers

| Tier | Model | Today |
|---|---|---|
| T0 point drag | `DragBody`: quadratic, isotropic, `F = −½ρ C_d A |v_rel| v_rel` in the world frame; or componentwise, `F_i = −c_i |v_i| v_i` per body axis | Built, `physics/forces.*`. `v_rel` = body velocity minus the medium's wind |
| T1 coefficient model | `LiftSurface`: flat plate plus polar tables (the fixed-wing seat) | Not built |
| T2 blade element / panel | BEMT per rotor (`engine D5`) | Not built; the `RotorRow` reserved lanes hold its parameters |
| T3 coupled | Two-way coupling to a solved flow field | Not built; needs a solved provider (`01-fields-and-media.md`) |

## Propulsor — rotor element (`engine D5`)

- **Lag:** shaft speed lags its command by a first-order lag, evaluated exactly (`1 − e^{−h/τ}`), never by Euler.
- **Thrust:** `T = k_T ω² · f_inflow · f_ground`.
  - `f_inflow`: momentum-theory induced-velocity curve, with the normal-working, vortex-ring (linear, clamped) and windmill-brake branches.
  - `f_ground`: Cheeseman–Bennett, sampled from the same world SDF collision uses, clamped at 1.25.
- **Torque:** `Q = k_Q ω²`, **uncorrected** by inflow or ground. Don't read `T` and `Q` as a power budget.
- **Wrench:** `T·axis` in the world frame; `cross(r, T·axis) − spin·Q·axis` in the body frame.
- **Not modelled:** rotor inertia (no gyroscopic or spin-up reaction), BEMT, disc drag (that is `DragBody`'s job), and the wake. The wake is drawn, not simulated (`PHY-3`).

All of it is in `vehicles/rotor.hpp`, sections 1–7.

## Model types and templates (`engine D4`)

A `ModelType` is a body template, collision proxy, force elements, sensor mounts, visual refs and a parameter schema. Instances spawn from it into slot allocations. Nothing vehicle-specific sits below `vehicles/`. Under the engine model:

- the rotor and drag elements are catalog responders;
- `make_quadrotor` (plus layout, Y-up, nose +X, spins +−+−; the moment equations are in `vehicles/quadrotor.hpp` §2) becomes a **template** on the public API, owned by Interface.

## Not built

Buoyancy (it waits for a density field worth reading), `LiftSurface`, capsule and mesh proxies, rotor gyroscopics, and the engine-side translation lock the drone stand needs (`../backlog.md`; it is a constraint, Core and Physics).
