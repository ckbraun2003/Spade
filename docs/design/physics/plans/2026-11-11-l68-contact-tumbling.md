# L68: contact tumbling (friction, restitution, contact torque, body shapes), DESIGN NOTE, for review

Asked by Cameron 2026-11-11 ("spade should integrate friction and tumbling"), for Kat (crashed drones bounce, roll and rest). Author: Core.
Reviewer: Physics (short). Approver: Spade. Nothing is built or queued. Kat's recon: `crash_physics_recon.md` (kat/look). Body-body contact is out of scope.

## What exists (and what it already does)

- `ContactParams {restitution_e, friction_mu, baumgarte_beta, slop, proxy_radius}` is per world and already drives a LINEAR response: normal impulse with
  restitution, Coulomb cap `mu * j_n`, Baumgarte push of `pos`. So friction and restitution for a sphere body (or an L9 contact-set vehicle) are one config
  edit in the host (it leaves both 0 today: Kat's probe). What is missing is the angular response, a manifold, and body shapes (`contacts.hpp` limits 1, 3).
- `BodyState` has `omega_body`, `mass`, `inv_inertia_diag` (body frame, principal axes) and the integrator already uses them: only the contact never writes them.
- Static contact is BANDED against the CPU on the device, not bit-equal (`test_gpu_contact_pass.cpp`: the SDF walk and normalize are the device's).

## Opt-in, so nothing moves

A new module `rigid_contact`, added to a module set by the host that wants it (`rigid_contact_modules()` = `standard_modules()` + it). Standard sets do not have it: no
array, no pass, no pipeline, no set-1 binding, identity, walk and SPIR-V unchanged (the batch D pattern: `points` exists only when asked for). Within a set that has
it, a body with no shape row (count 0) is skipped by one test, and the existing `static_contact.resolve` is not edited, so its kernel and SpirvPin rows do not move.
Proof: the pins at the standard set stay green (below) and a module-set run with no shaped body has the standard digest.

## The shape row

One `per_body` array `rigid_contact.shape`, row = 18 float4 (288 B): row 0 `{kind, count, radius, rest_speed}`, row 1 `{friction, restitution, 0, 0}`, then up to 16 body-frame
points `{x, y, z, 0}` (centre of mass at the origin, principal axes, as the contact sets). Registered arrays are walked, so the row is in the snapshot and in
`state_digest` for free; zero is "no shape". Every shape is compiled to this: a point list plus a radius.

| shape | compiled to | contact test per point |
|---|---|---|
| sphere r | 1 point at the origin, radius r | `phi < r` (today's sphere) |
| capsule (a, b, r) | points on segment a-b spaced <= r, ends included (max 16), radius r | `phi < r` |
| box (half extents) | its 8 corners, radius 0 | `phi < 0` |
| convex hull | the host's vertices (<= 16, else refused by name), radius 0 | `phi < 0` |

A point query is one `sample(world_sdf, pos + orient * p)` (distance and gradient from one walk, as today), so every shape works against the floor, walls, gate frames and
props, all of which are the world SDF. The set of shapes is closed (the four above); a mesh or an analytic capsule-vs-SDF distance is not planned.
Material: `friction`/`restitution` in the row, a NEGATIVE value means "this world's `ContactParams`". The SDF carries no material id, so one world has one surface
material (a per-surface table is out of scope). `rest_speed` (m/s, 0 = the default 0.1): approach speed below it gets e = 0, so a body settles without micro-bounces.

## The model, per substep, per shaped body (this order is the parity contract)

The pass sits after `static_contact.resolve`, before Integrate, on the velocity the previous Integrate left, as today. Bodies in slot order; points in row order; one sweep.
For each point k (p = pos + orient * local_k):

1. `s = sample(sdf, p)`; contact iff `s.distance < radius` (strict, NaN fails), `|grad| > 0`; `n = grad/|grad|`, `depth = radius - distance`; the contact point
   `c = p - n * radius`, lever `r = c - pos`.
2. Velocity at the point `v_c = vel + cross(orient * omega_body, r)`; `v_n = dot(v_c, n)`. Only an approaching point (`v_n < 0`) gets an impulse.
3. Normal: `e_eff = (-v_n > rest_speed) ? e : 0`; `a = conj(orient) * cross(r, n)`; `K_n = 1/m + dot(a, a * inv_inertia_diag)`; `j = -(1 + e_eff) * v_n / K_n`;
   `vel += (j/m) n`; `omega_body += inv_inertia_diag * (conj(orient) * cross(r, j n))`.
4. Friction, on the velocity after step 3: `v_t = v_c - dot(v_c, n) n`; if `|v_t| > 0`, `t = v_t/|v_t|`, `b = conj(orient) * cross(r, t)`, `K_t = 1/m + dot(b, b * inv_inertia_diag)`;
   `j_t = min(|v_t| / K_t, mu * j)` (an isotropic Coulomb cone: one tangent, the sliding direction; the cap makes it an exact stop and never a reversal); apply `-j_t t`
   at `c` the way step 3 applies `j n`.
5. After the sweep: the positional correction of today (`beta * max(depth - slop, 0)` along `n`) for the DEEPEST point only (strict `>`, the first wins a tie), on `pos`. The
   linear part of the velocity change is added to the contact scratch (PHY-7), so a bound IMU reads the hit.

Why this settles on a face: a corner hit has a lever arm, so `r x J` spins the body; it then meets the floor with another corner, and the sweep over all points,
re-run every substep at 1 kHz, removes the rotation at each contact until all points of one face rest. No gravity inside the pass; Integrate adds it after, as now.

**The substep rule** is the existing one: the pass runs once per substep, so its effect scales with the substep count the way the sphere's does. No CCD: the usable
envelope is `|v| h < radius` for radius shapes, and for radius-0 shapes `|v_point| h` must stay below the thinnest wall (the limit `contacts.hpp` states, now per point).

Known risks, each a test: (1) one sequential sweep per substep may jitter a resting box; if the settle test fails the fix is a per-shape iteration count (a lane in
row 0), not a new model. (2) A sphere on a flat floor rolls without end (there is no rolling resistance); a box or capsule stops. A small `spin_damping` (1/s, in the shape row) is
the standard answer; I recommend it OUT of the first cut unless Cameron wants spheres to stop (decision for Spade). (3) A radius-0 point deep in a thin wall pops out (as the sphere does).

## Existing world friction and restitution

For bodies WITHOUT the module's shape, nothing changes: the world's `ContactParams` act on the linear response as now, and Kat can set them today (plus a
radius). For shaped bodies the row's material wins, else the world's. No new world parameter and no change to `ContactParams` (its reserved lanes stay 0).

## API

- `rigid_contact_modules()`: the module set to pass to `Simulation::create`.
- `ShapeDesc` (kind, radius, a/b or half extents or points, friction, restitution, rest_speed) and `Result<void> Simulation::set_body_shape(BodyRef, const ShapeDesc&)`:
  validates by name, resolves before writing (as `set_world_wind`), writes the row, marks the mirror dirty; refuses by name a set without the module, a stale ref, `count > 16`, a
  non-finite or negative value, a degenerate capsule/box. `ShapeDesc{}` (kind none) clears it. Mass and inertia stay the caller's (`BodySpawn`), so a bare body supplies
  them; a helper `box_inv_inertia(mass, half_extents)` is the one convenience (decision: include, it is 6 lines).
- A vehicle model carries an optional `ModelType::shape`, written to its body's row at spawn by the existing vehicle-row hook (so a model-less bare body and a drone take
  one path). A model can also give its L9 contact points as a hull (`shape_from_contact_points`), the Phantom's 8 box corners per part being more than 16: those models name a
  smaller box or hull; stated, not hidden.

## GPU plan

One new kernel `collision_shape.slang`, one thread per body slot, mirroring the steps above op for op (NoContraction, the P1 to P4 profile, `parity`), after
`collision_static`; the shape array at a NEW set-1 binding (`contact_shapes`, the appendix pattern of `airflow_points`), so only a set with the module builds a second descriptor
set (pipelines 12 to 13). Standard sets: unchanged, SpirvPin standard rows unmoved, 3 new rows for the kernel. **Grade, for Physics and Spade to confirm:** the device is
within the existing static-contact band of the CPU (the SDF walk and normalize are the device's), bit-identical to itself run to run, and the CPU is bit-identical to
itself. "CPU and GPU bit-equal" is not what static contact is today and a body-level pass cannot beat its SDF; I will bound the difference in the tests instead (position and
orientation of a settled drop, and the number of bounces).

## REDs (each built, each failure line in READY)

Pins, green at red and for good: standard identity `0x3303cfc86821f502` and walk of 11 unchanged; no `rigid_contact.*` array, pass or pipeline in a standard set; the golden
corpus, `kHeldQuadDigest`, device `68eff17213201d8f`; a module set with no shaped body has the standard digest.
CPU, REDs: sphere drop bounces to e^2 of its height and rests (rest_speed); a level slide stops at `v^2/(2 mu g)` within 2 %; a box dropped on a corner gains angular
velocity at the first impact and settles on a face (up axis within 1 deg, `|v|` and `|omega|` under a threshold for the last half second, no jitter); a sphere on a
friction floor reaches `omega r = v`; a capsule lies down on both ends; a hull of the box's 8 corners is bit-equal to the box; a body thrown at a wall and at a gate-frame
SDF bounces off the normal; material row beats the world's and a negative takes the world's; refusals by name; two runs are bit-equal (memcmp); a snapshot mid-tumble restores to
the same digest after 2000 steps; the digest moves with the row; a vehicle model with a shape spawns with the row and a model without one is unchanged.
Device (Gpu*): pipeline count 12 to 13 only with the module; the drop and the corner tumble within the stated band of the CPU; two device runs bit-identical.

## Size

About 8 working days (6 to 10): shape row, module, validation, API and the vehicle hook 2; CPU pass and the REDs 2.5; device kernel, binding, SpirvPin and device tests 2;
tuning the settle, gates and review rounds 1.5. Physics' review is about half a day of theirs. The queue, not the work, sets the date.

## Decisions I need (Spade, Physics)

1. Opt-in module (recommended) over always-on. 2. The device grade above. 3. Spheres' rolling resistance: out or in. 4. One sweep first, iterations only if the settle test needs
them. 5. Rest speed default 0.1 m/s. 6. Convex hull capped at 16 points. 7. Include `box_inv_inertia`.
