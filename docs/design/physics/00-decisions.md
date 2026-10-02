# Physics — decisions

**Owner:** Physics. **The only place a Physics decision's status lives.** One row per ID. Statuses: `live`; `live, paused` (in force, but its work waits on the restructure); `superseded by X`; `record` (a fact about the past); `proposed` (unsigned). Sources are in `../superseded/2026-09-consolidation/` unless noted. "02" means `02-engine.md`.

## Legacy rulings homed in Physics

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `engine D1` | Symplectic Euler, quaternion by exponential map, inertia in the body frame, accuracy bought with substeps rather than order; the integrator is a pure function of `(state, wrench, dt)` | approved 2026-08-08 | live | 02 §1, §6 |
| `engine D3` | Static world is an analytic SDF program; dynamic proxies are spheres and capsules; dynamic-dynamic uses a sorted grid; triangle-mesh colliders come later | approved 2026-08-08 | live. Capsule proxies are not built (`07-status.md`) | 02 §1, §6 |
| `engine D4` | A model-type layer one tier above meshes: body template, collision proxy, force elements, sensor mounts, visual refs, parameter schema. Nothing vehicle-specific below `vehicles/` | approved 2026-08-08 | live for the layer. "Quadrotor first" is superseded by the engine model: the quadrotor becomes a **template** (Interface), and its rotor and drag elements are catalog responders (`02-responders.md`) | 02 §1, §7 |
| `engine D5` | Rotor aero = thrust/torque curves + RPM lag + momentum-theory inflow + ground effect sampled from the world SDF. BEMT and flow-field coupling are roadmap, and the row schema reserves their fields | approved 2026-08-08 | live. BEMT is aero tier T2 (`02-responders.md`) | 02 §1, §7 |
| `engine D6` (provider half) | Per-world density and temperature, uniform wind and seeded Dryden turbulence are the medium's providers | approved 2026-08-08 | live. Temperature is not built. The interface half is Core's (`../core/00-decisions.md`) | 02 §1, §6 |
| `charter P7` | The effects layer is non-authoritative, with a roadmap to air flow; nothing becomes authoritative until the CPU twin can mirror it | approved 2026-08-06 | superseded by `L3`/`L4` and the engine model's fidelity tiers. Its promotion rule survives as `PHY-3` | `01-charter.md` §2 |
| `SL8` | SPH is implemented to v2 discipline, not ported: CPU and GPU passes, neighbour search on the existing sorted grid, a `Fluid` component, a measured band. If no acceptable band exists, it is declared CPU-only, never given a widened band | signed 2026-09-17 | live, paused (restructure §5). It returns as a field provider (`01-fields-and-media.md`). The transfer-register row stays open | `06-sandbox-and-v1.md` §2.2 |
| `D-S6-2` | A body's own `proxy_radius` overrides the world's default contact radius; `effective_proxy_radius()` is the one predicate both collision passes and the GPU kernels use | signed 2026-10-02 (user, via lead), as built (an in-program decision, S6 Task 2) | live | `07-status.md` §1; `physics/contacts.hpp` |

**Quoted here, homed elsewhere:** `CORE-3` (the gaussian-draw band every stochastic physics kernel inherits); `engine D2` (fp32 and pinned reduction order, which is why op order is the parity contract) and the `D6` interface half, both Core's; `engine A9` (gravity applied inside Integrate) is Core's; `SL14c` (the 50,000-body fluid scene as a standing capability target) is Interface's; the SPIR-V rules `P1`–`P5` and `E1`/`E2` are Test/Docs's.

## Physics rulings

| ID | Ruling | Signed | Status | Source |
|---|---|---|---|---|
| `PHY-1` | (moved) The gaussian draw's CPU↔GPU band at its source | — | moved to Core 2026-10-01 as `CORE-3` (`../core/00-decisions.md`), because it is a band on an RNG primitive. The Dryden, IMU and GNSS bands cite it | `01-charter.md` §4.1 |
| `PHY-2` | Grades of the physics modules, per backend: the table in `04-verification.md` | signed 2026-10-02 (user, via lead) | live | this library |
| `PHY-3` | A field or effect that stepping does not read may be added at best-effort grade, as a pure function of published state. It may not be read by stepping until it declares a grade that meets the world's requirement, and for reference grade that means a CPU implementation with a golden. This is `charter P7`'s promotion rule, in engine-model terms | signed 2026-10-02 (user, via lead) | live; first instance: `vehicles::rotor_wake_velocity` | `../plans/2026-10-01-drone-sim-box-design.md` |
