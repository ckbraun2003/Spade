# Physics plans

Active Physics plans, one file each, named `YYYY-MM-DD-<topic>.md`.

- `2026-10-02-module-api-requirements.md`: what SPH, the translation lock and Jacobi contact need from Core's module API. Input to Core's spec; Core decides.
- `2026-10-03-drone-builder-physics.md`: the drone builder's physics: motor and ESC, battery, propeller tier, composite inertia and a steady-state solver. Approved by the user 2026-10-03.
- `2026-10-03-drone-builder-functions-plan.md`: its pure functions and host utilities. Built, merged at `1d3112d`.
- `2026-10-03-propulsion-rows-plan.md`: the stateful motor and battery rows, after module-API stage 4.
- `2026-10-03-airframe-compile-plan.md`: part-built airframes to a `ModelType` (DBP-44), with a fitted momentum-variant rotor until the rows land. Built, merged at `58abbf4`.
- `2026-10-05-imu-contact-specific-force-plan.md`: the IMU's specific force includes the contact response (Kat's report), and the reported velocity at rest.
- `2026-10-05-airflow-design.md`: the joint spec, 3D airflow as grid-solver field providers (`PHY-9`). Every realm's answers folded in; Cameron's answers marked in §8; §11 the fidelity and validation section.
- `2026-10-12-airflow-chain-runner-design.md`: DRAFT for Core. Plan B's GPU chain runner: one record-once chain pattern shared with `StepRecorder` (a Core hunk), a spike target outside the engine, refusals by name on device limits and VRAM.
- `2026-10-12-airflow-drone-demonstration-plan.md`: approved by the user 2026-10-12. The user's goal, a 3D drone with its airflow, propeller flow, vortices and surface pressure, validated (`PHY-10`). Milestones D1 (one-way CPU, recorded) to D4 (the module, two-way); the work beyond plans A, B and C, per realm.
- `2026-10-05-airflow-solver-core-plan.md`: plan A, the CPU reference solver core and V1 (the cavity, the Taylor–Green report, the actuator disc's wake). **Built** (2026-10-12): V1 within its tolerances. Plans B (the GPU twin and the spike, with Core) and C (the `airflow` module, after stage 7) follow.
- `2026-10-13-airflow-solids-design.md`: approved by the lead 2026-10-13. P2, P2b and P3 on plan A's reference: solids read through `sample_parts()` (compiled and imported mesh parts alike), face apertures, sub-2-cell walls as labelled sheets, porous parts, the wall force and moment, and the drone box's boundaries.

The drone sim box's Physics task (the rotor wake) was in the lead's plan, `../../plans/2026-10-01-drone-sim-box-plan.md`, Task 2. SPH gets its own plan here when it resumes (`PHY-4`, `../01-fields-and-media.md`).
