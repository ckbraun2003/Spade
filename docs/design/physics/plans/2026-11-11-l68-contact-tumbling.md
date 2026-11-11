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

One `per_body` array `rigid_contact.shape`, row = 18 float4 (288 B): row 0 `{kind, count, radius, rest_speed}`, row 1 `{friction, restitution, iterations, 0}` (`iterations` is reserved from the start, 0 = 1 sweep; Physics' review), then up to 16 body-frame
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
material (a per-surface table is out of scope). `rest_speed` (m/s, 0 = the default 0.1): approach speed below `max(rest_speed, 2 |g| h)` gets e = 0, with the world's gravity and the substep `h` the pass is given, so a body
settles without micro-bounces and the default cannot be wrong at a coarse substep (at h = 10 ms, g h is 0.098 m/s; Physics' review).

## The model, per substep, per shaped body (this order is the parity contract)

The pass sits after `static_contact.resolve`, before Integrate, on the velocity the previous Integrate left, as today. Bodies in slot order; points in row order; one sweep.
For each point k (p = pos + orient * local_k):

1. `s = sample(sdf, p)`; contact iff `s.distance < radius` (strict, NaN fails), `|grad| > 0`; `n = grad/|grad|`, `depth = radius - distance`; the contact point
   `c = p - n * radius`, lever `r = c - pos`.
2. Velocity at the point `v_c = vel + cross(orient * omega_body, r)`; `v_n = dot(v_c, n)`. Only an approaching point (`v_n < 0`) gets an impulse.
3. Normal: `e_eff = (-v_n > max(rest_speed, 2 |g| h)) ? e : 0`; `a = conj(orient) * cross(r, n)`; `K_n = 1/m + dot(a, a * inv_inertia_diag)`; `j = -(1 + e_eff) * v_n / K_n`;
   `vel += (j/m) n` (the divides and the normalize of step 1 are IEEE on the CPU and fp32_math's `div32_ftz`/`sqrt32_ftz` on the device, so that gap is the SDF walk only); `omega_body += inv_inertia_diag * (conj(orient) * cross(r, j n))`.
4. Friction, on the velocity after step 3: `v_t = v_c - dot(v_c, n) n`; if `|v_t| > 0`, `t = v_t/|v_t|`, `b = conj(orient) * cross(r, t)`, `K_t = 1/m + dot(b, b * inv_inertia_diag)`;
   `j_t = min(|v_t| / K_t, mu * j)` (an isotropic Coulomb cone: one tangent, the sliding direction; the cap makes it an exact stop and never a reversal); apply `-j_t t`
   at `c` the way step 3 applies `j n`.
5. After the sweep: the positional correction of today (`beta * max(depth - slop, 0)` along `n`) for the DEEPEST point only (strict `>`, the first wins a tie), on `pos`. The
   linear part of the velocity change is added to the contact scratch (PHY-7), so a bound IMU reads the hit.

Why this settles on a face: a corner hit has a lever arm, so `r x J` spins the body; it then meets the floor with another corner, and the sweep over all points,
re-run every substep at 1 kHz, removes the rotation at each contact until all points of one face rest. No gravity inside the pass; Integrate adds it after, as now.

**The substep rule** is the existing one: the pass runs once per substep, so its effect scales with the substep count the way the sphere's does. No CCD: the usable
envelope is `|v| h < radius` for radius shapes, and for radius-0 shapes `|v_point| h` must stay below the thinnest wall (the limit `contacts.hpp` states, now per point).

Integrate already carries the gyroscopic term `omega x (I omega)` (`integrator.cpp` step 4b), so an asymmetric free tumble shows the intermediate-axis flip.
If the settle test needs more than one sweep, the per-shape `iterations` lane turns it on, the friction cap then uses the point's ACCUMULATED normal impulse (not the last `j`, or
the second sweep loses friction), and READY says so.

Known risks, each a test: (1) one sequential sweep per substep may jitter a resting box; if the settle test fails the fix is a per-shape iteration count (a lane in
row 0), not a new model. (2) A sphere on a flat floor rolls without end (there is no rolling resistance); a box or capsule stops. A small `spin_damping` (1/s, in the shape row) is
the standard answer; I recommend it OUT of the first cut unless Cameron wants spheres to stop (decision for Spade). (3) A radius-0 point deep in a thin wall pops out (as the sphere does), and eight corners at radius 0 miss a thin SDF feature that falls between them (a gate bar thinner than the box's
corner spacing, as well as a thin wall): stated for gate frames too. The settle test takes a tolerance in depth, not zero (a box on an edge resolves one corner at a time).

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
itself. "CPU and GPU bit-equal" is not what static contact is today and a body-level pass cannot beat its SDF; I will bound the difference in the tests instead, pinning OUTCOMES, not trajectories: the settled face (up-axis index) identical, settled position and orientation within the
static-contact band, bounce count equal within 1.

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

## Decisions (Spade, 2026-11-11: approved on all six, subject to Physics' review)

1. Opt-in module `rigid_contact`; READY proves the standard set unchanged with the standard digests and the SpirvPin rows.
2. Grade: CPU-CPU and GPU-GPU bit-identical; the GPU within a stated band of the CPU (settled pose, bounce count), the grade static contact has today.
   Test/Docs names the band before the first comparison.
3. No rolling resistance in the first cut. The API docs say a sphere rolls on.
4. One sweep per substep first. Iterations are added only if the settle test needs them, and READY says so.
5. A hull is capped at 16 points; a larger one is refused by name.
6. `box_inv_inertia(mass, half_extents)` is included.

If Physics' review changes anything material, it goes back to Spade before any ticket.

## As built (Core, 2026-11-11; where it differs from the plan above, this section wins)

- **Names.** `modules::rigid_contact_module()` (one module to add to `standard_modules()`, not a set); the array is `rigid_contact.contact_shapes` (its set-1 binding's name, binding 35); `Simulation::set_body_shape(BodyRef, const physics::ShapeDesc&)`, `body_shape_row(BodyRef)`, `ModelType::shape`, `physics::box_inv_inertia`.
- **A set without the module** refuses `set_body_shape` and a model that carries a shape with `Code::unavailable` naming `rigid_contact` (the lock module's precedent); a bad description is `invalid_argument` naming the cause (every message opens `set_body_shape:`; `iterations` above 16 too).
- **The solve is sequential impulses, not one sweep** (Spade accepted: the plan's "one sweep first" gave a 1-2.5 mm/s lateral creep on a settled box). The contacts are found once per substep (one SDF walk per point); then `iterations` sweeps (row lane; 0 = 4, at most 16) of normal then friction impulse per contact with ACCUMULATED impulses (normal >= 0, friction inside `mu j_n`), the restitution bias set from the pre-solve approach speed, then the deepest point's positional correction. Measured on a box slid to rest (10 s window): sweeps 1 / 2 / 4 / 8 / 16 -> max lateral speed 2.5e-3 / 1.4e-3 / 3.4e-6 / 2e-9 / 2e-10 m/s; drift 5.8e-3 / 1.6e-3 / 1.7e-5 / 1e-8 / 3e-11 m; whole step 2.1 / 2.6 / 3.1 / 4.4 / 7.2 us per shaped body (CPU, release). Pinned: < 1e-4 m/s and < 1 mm for the box and the hull.
- **A skin radius does not make a thin bar visible** (measured; replaces the plan's reading of risk 3). A shape is points: a feature thinner than the point spacing falls between them. A skin on the corners of a cube over a 0.10 m bar centred between them put the corner contacts BESIDE the bar (sideways normals) and the cube still fell through. Face centres (a 14-point hull) halve the spacing and held it on the 0.10 m bar when centred (borderline off-centre). Put points on the extremities that hit. The skin stays for rounded shapes (capsules) and thick bars.
- **Measured limits (not defects).** A 3 m/s slide stops 0.4% beyond v^2/(2 mu g); a step ends with about 2 g h of vertical speed (the pass runs before Integrate). A sphere rolls on (no rolling resistance).
- **Device.** `collision_shape.slang`, one thread per body slot, set-1 binding `contact_shapes` (35, read-only), `GpuRecipe::collision_shape`; pipelines 12 -> 13 only with the module; every divide and square root is fp32_math's `div32_ftz` / `sqrt32_ftz`. Band (Test/Docs' record `docs/design/test-docs/2026-11-11-l68-device-band.md`): at tilt 20 deg the CPU ensemble spreads are zero, so the bands are the floors (0.010 m, 2 deg, 1 bounce); measured device against CPU: lateral 0.00000 m, yaw 0.000 deg, impacts 0, height gap 4.8e-7 m, tilt 0.289 / 0.290 deg; both device runs memcmp-equal.
- **Standard sets unchanged:** identity `0x3303cfc86821f502`, walk of 11, `kHeldQuadDigest`, device digest `68eff17213201d8f`, every standard SpirvPin row; three SpirvPin rows are new (`collision_shape` x 32/64/128).
