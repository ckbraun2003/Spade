# IMU specific force includes the contact response — plan

**Owner:** Physics. **Status:** plan, 2026-10-05, for the lead's review. **From:** Kat's report (`kat_host_spade` at Kat `237c1ad5`), the lead's decision (1) yes, (2) no, (3) investigate.
**Closes:** the debt row in `07-status.md` ("A resting body's IMU reads ~0 specific force"), `03-sensors.md`'s caveat, `imu.hpp` §5, and `../backlog.md`'s IMU-contact item.

## The cause

Each substep runs `constraints` (static contact, then dynamic contact) before `integrate`, on CPU and GPU alike. Contact changes `vel` by impulses and `pos` by Baumgarte, and never touches `force_acc`. Integrate then overwrites `specific_force = conj(q) · force_acc / m`, so the contact's Δv never reaches the IMU. At rest, contact removes the `−g·h` the last Integrate added, and the IMU reads 0. A wrench-held body reads `+g` because the wrench is in `force_acc`.

## The change (decision 1)

Specific force over a substep is `a_ext + Δv_contact / h`, in body axes. At rest, `Δv_contact = +g·h`, so the IMU reads `+g` up. During an impact it reads the reaction.

- **Where Δv comes from:**
  - Static contact: one contact per body per substep, so Δv = vel after − vel before, local to the body (CPU `contacts.cpp`; GPU `collision_static.slang`, one thread per body).
  - Dynamic contact: a Gauss-Seidel sweep in which many pairs can touch one body, so the per-pair Δv is accumulated per body (CPU `grid.cpp` `resolve_pair`; GPU `collision_dynamic.slang`, one thread per world, so it's race-free).
- **Where it is kept (the one design choice, Core's layout and ABI):**
  - **A, recommended. A transient per-body `contact_dv` scratch (vec3), not registered state.** Contacts add to it in world axes. Integrate reads it, adds `contact_dv / h` to `accel_ext` for the specific force only (`vel` already has the impulse), and zeroes it.
    - It is zero at every substep boundary, so snapshots, restore and the state digest don't change, and no pass has to run for the invariant to hold.
    - CPU: a vector in the world's substep view. GPU: one scratch storage buffer, like the grid's, bound to the two contact kernels and to `integrate`.
    - Cost: one new binding in `bindings.slang` and a scratch access in the module declarations. Both are Core's.
  - B: reuse the `specific_force` row as the accumulator between `constraints` and `integrate`, with static contact writing (not adding) it for every active body. That needs no storage, but it is correct only while static contact runs for every active body every substep. A module set without it would read a stale body-axis value as a world contact. Not recommended.
- **Noise and bias:** unchanged. The contact term joins the true value before bias and white noise. The model has no range clamp, so an impact spike appears at full size: 1 m/s stopped in one 1 ms substep reads about 100 g. A range clamp is a possible later addition, not part of this change.
- **`−0.0`:** add the contact term only when `contact_dv` is non-zero. Adding a zero vector turns `−0.0` into `+0.0` and would move contact-free digests.

## Decision 3: the reported velocity at rest

It is not a reporting bug. The state velocity is `v_post-contact + (a+g)·h`, the next substep's pre-contact value. The position update uses it (`pos += v·h` sinks by `g·h²`, and Baumgarte lifts it back). Reporting 0 would contradict the motion the state records. No post-contact velocity is stored anywhere to report.

The real fix is the usual velocity-level order: integrate the velocity, resolve contacts, then integrate the position. Then the stored velocity at rest is 0 and nothing sinks. That splits `integrate` into two passes with `constraints` between them (Core's schedule and kernels) and moves every contact golden again. **Recommendation:** ship decision 1 now, and record decision 3 as its own plan with Core. If the lead wants both in one bump, the two changes share the goldens' regeneration.

## Tests (red first)

1. A body at rest on a plane, IMU every substep, zero noise: the mean reads `[0, +9.80665, 0]` within 1e-4. Red today (reads 0).
2. Free fall reads 0. Green today; a guard.
3. A ball dropped onto a plane: the impact substep reads about `(1+e)·v_impact / h` up, and the substeps before it read 0. Red today.
4. The GPU twins of 1 and 3, banded per Cameron's ruling.
5. Decision 3's "velocity at rest reads 0" belongs to the reorder plan; with decision 1 alone it would still read `−g·h`.

## What moves

- **Goldens** (`state_digest` folds `specific_force`):
  - `bounce`, `shower` and `two_world_isolation`, plus their `tests/golden/viewer/*.trajectory.txt`, move;
  - `ballistic`, `gnss_tumble` and `quad_hover` must not (the `−0.0` guard).
  - Regenerated under `TD-1`, and final only when the gcc leg reproduces them (`TD-12`).
- **Bands:** the contact scenarios' `kSpecificForce` rows are zero today (bounce, shower, shower_ladder, two_world_isolation, contact_pair, heterogeneous_geometry_set, restore_resume, gnss_receiver_body, gnss_tumble).
  - They are re-measured with the re-banding work.
  - Δv/h scales velocity differences by 1/h, so expect larger bands where the velocity band is non-zero.
- **Tests that pin "contacts write only pos and vel"** (`test_contacts.cpp:715`, `test_grid.cpp:1251`) still hold under A, since the scratch is not body state. Under B they would change.
- **Docs:** `imu.hpp` §5, `03-sensors.md`, `07-status.md`'s debt row, `../backlog.md`.

## Order of work

1. The lead and Core choose A or B, and whether decision 3 joins this bump.
2. Red tests (CPU), then the CPU change, then green.
3. The GPU change, then the GPU twins banded with the re-banding policy.
4. Golden regeneration (TD-1), docs, gcc check, then the leg (TD-12).
