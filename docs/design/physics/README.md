# Physics library

Field providers (gravity, media, turbulence, and later solved flows including SPH), responders (integration, contact, aero tiers, propulsors), physical sensors (IMU, GNSS), their GPU kernels and their parity bands. Code:
- `engine/physics/`, `engine/vehicles/`, `engine/sensors/` and `engine/world/medium.*`;
- the physics kernels in `engine/shaders/kernels/` (`integrate`, `collision_*`, `grid_*`, `forces_drag`, `rotors`, `medium_update`, `sensor_*`) and `dryden.slang`;
- the matching `tests/test_*.cpp` suites.

## Read in this order

1. `01-fields-and-media.md`: what a field provider must be, the providers that exist, and SPH's design.
2. `02-responders.md`: integration, contact, aero tiers, the rotor element, model types.
3. `03-sensors.md`: IMU and GNSS models, and the rules every sensor follows.
4. `04-verification.md`: how a physics change is checked, the parity bands, and each module's grade.
5. `07-status.md`: what exists today and what is open.
6. `00-decisions.md`: the ruling behind an ID, and whether it is in force.

## If you're here to…

| …do this | read |
|---|---|
| change a force law or the integrator | `02-responders.md` (the rules), then that module's header. **Op order is the parity contract**: a reordering moves goldens |
| add an aero tier or a new responder | `02-responders.md` (tiers), `04-verification.md` (grades) |
| add or change a medium or field provider | `01-fields-and-media.md` |
| add a sensor | `03-sensors.md`; the arena and rings are Core's |
| start SPH | `01-fields-and-media.md` (SPH), `00-decisions.md` (`SL8`), `../../v1-transfer-register.md` |
| draw something physics implies but does not simulate | `01-fields-and-media.md` (effects), `PHY-3` |
| explain a moved golden or a parity failure | `04-verification.md`, then Test/Docs's golden governance |
| find out whether `engine D1`–`D6`, `P7`, `SL8` or `PHY-n` is in force | `00-decisions.md` |

## Series

- **Owned here:** `engine D1`, `D3`, `D4`, `D5`, the provider half of `D6`, `charter P7`, `SL8`, `D-S6-2`, and new rulings `PHY-n`.
- **Shared:** `engine D6` with Core (Core owns the sampling interface, Physics the providers), and `engine D4` with Interface (the layer here, the quadrotor template there).
- **Quoted, not owned:** the laws `L1`–`L8` (`../00-charter.md`); `engine D2`, `A1`, `A4`, `A9`, `SL6` (Core); `SL14c` (Interface); the SPIR-V rules and golden governance (Test/Docs).

Cite as `engine D5` or `PHY-2`, by document and section, never by line number. Status lives in `00-decisions.md` and `07-status.md` only.

## History

The source text is `../superseded/2026-09-consolidation/02-engine.md` (§1, §6–§8), with the physics rows of `07-status.md` and `08-lessons.md` beside it, and `06-sandbox-and-v1.md` §2.2 for SPH. The model derivations live in the code headers (`vehicles/rotor.hpp`, `world/medium.hpp`, `physics/contacts.hpp`, `sensors/imu.hpp`, `sensors/gnss.hpp`), which these specs point to rather than repeat. Active plans go in `plans/`, dated.
