# Physics plans

Active Physics plans, one file each, named `YYYY-MM-DD-<topic>.md`.

- `2026-10-02-module-api-requirements.md`: what SPH, the translation lock and Jacobi contact need from Core's module API. Input to Core's spec; Core decides.
- `2026-10-03-drone-builder-physics.md`: the drone builder's physics: motor and ESC, battery, propeller tier, composite inertia and a steady-state solver. Approved by the user 2026-10-03.
- `2026-10-03-drone-builder-functions-plan.md`: its pure functions and host utilities. Built, merged at `1d3112d`.
- `2026-10-03-propulsion-rows-plan.md`: the stateful motor and battery rows, after module-API stage 4.
- `2026-10-03-airframe-compile-plan.md`: part-built airframes to a `ModelType` (DBP-44), with a fitted momentum-variant rotor until the rows land. Built, merged at `58abbf4`.
- `2026-10-05-imu-contact-specific-force-plan.md`: the IMU's specific force includes the contact response (Kat's report), and the reported velocity at rest.

The drone sim box's Physics task (the rotor wake) was in the lead's plan, `../../plans/2026-10-01-drone-sim-box-plan.md`, Task 2. SPH gets its own plan here when it resumes (`PHY-4`, `../01-fields-and-media.md`).
