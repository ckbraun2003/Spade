# Physics — fields and media

**Owner:** Physics. **Normative.** This is the target design in the engine model's terms (`../01-engine-model.md`). Where the code is today is in `07-status.md`. Core owns the field registry, regions and the sampling interface (`../core/`). Physics owns the providers that answer through it.

## What a field provider must be

- **A pure function** of the world's parameters, position, time (tick) and its own registered state. No clock, no hidden generator (`L1`).
- **Seeded through `rng::Stream`** with its own domain tag, if it is stochastic. A draw count must not depend on parameter values. Dryden draws the same number of gaussians at turbulence level `none` as at `severe`, so the stream position stays a function of substep count alone, and changing one world's turbulence cannot move any other stochastic system.
- **Expressible as data** for a GPU kernel: parameters plus a kind tag, not behaviour hidden in host code. A provider with no GPU kernel is CPU-only, and placing it in a GPU chain is a refusal (`L6`).
- **Graded** (`L3`, `PHY-2`).

## The providers

| Field | Provider | Fidelity | Today |
|---|---|---|---|
| Gravity | per-world constant | constant | `WorldParams::gravity`, applied inside Integrate (`engine A9`, Core). It becomes a field when the scheduler lands |
| Air density | per-world constant | constant | `WorldParams` |
| Wind | per-world uniform wind | constant | `WorldParams::wind` |
| Gust | Dryden turbulence, MIL-F-8785C low-altitude form, seeded per world | procedural | `world/medium.*` (`DrydenMedium`). Position-independent: one gust per world, advanced once per substep in MediumUpdate. The filters are discretised exactly, not by Euler |
| Temperature | per-world constant | constant | **Not built** (named in `engine D6`) |
| Flow velocity, pressure, fluid density | SPH (`SL8`) | solved | **Not built**; paused (below) |
| Rotor wake (visualisation) | `vehicles::rotor_wake_velocity` | analytic, best-effort | `vehicles/rotor_wake.*`. Its one intended reader is the drone sim box's heatmap (drone plan Task 5); stepping never reads it (`PHY-3`) |

Today every responder reads these through `Medium::sample(world, pos) -> {density, wind}` (`engine D6`). The interface is the point: a provider can be replaced without touching its readers.

## SPH as a field provider (`SL8`, paused)

SPH is the v1 transfer register's one open row (`../../v1-transfer-register.md`), and v1 stays in the tree until it closes. Its design under the engine model:

- **What it provides:** fluid density, pressure and velocity fields over its region, sampled by responders (buoyancy, aero tier T3) and by cameras as channels.
- **How:** neighbour search on the existing sorted grid (no second spatial structure), then density and pressure-force passes, with a `Fluid` component carrying `rest_density`, `stiffness` and `viscosity`. `ComponentTypeId::fluid` (9) is already reserved.
- **Grade:** reference on the CPU with a golden. On the GPU it is banded with a measured band, or the GPU path is declared absent. **A band is never widened to admit it** (`SL8`).
- **Reference:** v1's `[SYSTEM]FluidDensity.comp` and `[SYSTEM]FluidForce.comp` (GPU-only, no determinism story), read for the physics, not ported.
- **Paused** until the module API and scheduler exist (restructure §5). Its plan is written then, in `plans/`.

## Effects that stepping does not read (`PHY-3`)

An analytic field that only a viewer reads may land at **best-effort** grade, CPU-only, as a pure function of published state. The rotor wake is the first: an actuator-disc slipstream built from the rotor element's own inflow, so it shows the induced velocity the thrust already implies (`vehicles/rotor_wake.hpp` holds the model and its limits). Stepping may read such a field only after it declares a grade that meets the world's requirement. For reference grade that means a CPU implementation with a golden. This is `charter P7`'s promotion rule.
