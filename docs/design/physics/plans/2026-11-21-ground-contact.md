# Ground contact for take-off and landing: the X500's feet (O140 phase 1, item 4: design note, revised)

Physics, 2026-11-21. Branch `physics/ground-contact-note`; revised after Core's answer. Design, one opt-in print in `tests/test_x500_kit_inertia.cpp` (`[kit-gear]` lines, which measured the feet below), and the checks on branch `physics/ground-contact-checks` (`tests/test_x500_ground_contact.cpp`, 6/6 at the link's 500 us tick). **No engine change is needed:** the contact the X500 needs is the existing L68 `rigid_contact` module's hull shape, carried in the model file by L82's `collision:` block. This note's first version proposed an engine change; Core's answer replaced it, and the checks below confirm the existing machinery does the job with one setting (16 sweeps).

## 1. What is there

- A body's default contact is one sphere, `proxy_radius` (`D-S6-2`), centred on the body's origin, the centre of mass. The X500 kit model carries 0.377 m, so a drone on that sphere would stand 0.377 m high with no righting torque.
- **A shaped model replaces it.** With `rigid_contact_module()` in the module set, a model's `collision:` block (kind `hull`: 1 to 16 body-frame points and a skin radius, per-shape friction and restitution and `iterations`) gives the body a point list tested against the world's SDF with an impulse at each point; the model's proxy sphere is made inert at spawn. It runs on both backends.
- The kit model carries no landing gear as a solid (its notes file's `not_carried`), so the feet are a contact list, not part of the airflow solids.

## 2. What the real X500 stands on (measured, derived grade)

From the build table's frame meshes (`tools/flow_movie/data/x500_build/parts.yaml`, the `JIAO-EVA_1..4` objects, read from the local part store by the opt-in `X500KitInertia` test; design frame +X forward, +Y up, +Z right, origin between the plates; the centre of mass is at (-0.0045, -0.0254, 0.0009) m):

| Foot (rubber pad) | x (m) | y (m) | z (m) | size |
|---|---|---|---|---|
| EVA 1 front-right | +0.0210 .. +0.1140 | -0.2153 .. -0.1963 | +0.1105 .. +0.1295 | 93 x 19 x 19 mm |
| EVA 2 rear-right | -0.1140 .. -0.0210 | same | +0.1105 .. +0.1295 | same |
| EVA 3 rear-left | -0.1140 .. -0.0210 | same | -0.1295 .. -0.1105 | same |
| EVA 4 front-left | +0.0210 .. +0.1140 | same | -0.1295 .. -0.1105 | same |

- Lowest point of the drone: **y = -0.2153 m**, the pads' bottom faces, all four level. The pads hang from the skid tubes (`CARBON-FIBER-TUBE300_1/_2`, 250 mm long at z +-0.030) through the pylons (`PYLONS-X500_1/_2`) and the foot connectors (`JIAO-LIANJIE_1/_2`).
- **Standing height: the centre of mass 0.1899 m above the ground**; the discs 0.276 m. Footprint 0.135 m front to back between pad centres, 0.24 m side to side.
- **Tip-over, derived:** about 31 degrees in pitch and 34 in roll.

## 3. The shape (what the model file carries)

`collision: {kind: hull, points: [...], radius: 0.0095, friction: 0.6, restitution: 0.1, iterations: 16}`:

- **Twelve points, three to a pad** (both ends and the middle of each 93 mm bar, at y -0.2058, so the 9.5 mm skin touches at -0.2153): x +-0.0305, +-0.0675, +-0.1045, z +-0.12. Three, not two, so a pad cannot rest on a point pair (Core's suggestion; twelve of the sixteen allowed). They are written in the model's body frame: p_body = design_to_principal x (p_design - com_offset).
- **Sixteen contact sweeps (`iterations: 16`, the maximum), not the default 4.** The solve is sequential impulses; with twelve points the default leaves it unconverged, and a level drop at 3.1 m/s spins at 0.7 to 0.8 rad/s (0.70 with the centre of mass at the foot centroid, 0.79 with the X500's offset, 0.34 with no friction, 0.52 with no restitution). At 16 sweeps the spin is 0.017 rad/s. A converged solve zeroes every point's normal speed, so a level impact has no spin; my first estimate of 1 rad/s from the centre-of-mass offset was wrong for that reason.
- **The vehicle's spawn pose is the DESIGN frame's** (`VehicleSpawn`: the design origin and design axes, converted by `spawn()`), not the body's. The checks' first runs spawned the conjugate of `design_to_principal`, landing the drone 5 degrees yawed and tilted, which spun it; a vehicle placed level is orientation identity with the design origin 0.2153 m over the ground at rest.
- **The film's X500 model file is not changed.** The checks build the shape in memory on the loaded model; the link's model files (the PX4-match variant and the kit model) will carry the `collision:` block, as new files, so the posted film's pinned model stays as it is.

**Limit, stated: the points are the feet only.** A drone tipped past about 31 degrees has nothing else to hold it up, and sinks through the ground plane once its feet are above it (the checks' 40-degree pitch case turns 145 degrees in 1 s and has no contact left). Before any landing or crash scenario that can tip, add points on the lowest of the plates, the motors and the prop guards; for a first link bring-up that never tips, the feet are enough.

## 4. Values and their grades

| Value | Number | Grade |
|---|---|---|
| foot geometry | the table above | derived (mesh extents) |
| skin radius | 9.5 mm (half the pad's 19 mm section) | derived |
| friction | 0.6 | **assumed** (EVA foam on a hard floor, about 0.5 to 0.8; cite in phase 6) |
| restitution | 0.1 | **assumed** |
| sweeps | 16 | measured: 4 leaves 0.79 rad/s of spin on a level drop, 16 leaves 0.017 |

## 5. The checks (branch `physics/ground-contact-checks`, 6/6 at 500 us, CPU)

| Check | Result |
|---|---|
| rest | the centre of mass at 0.18887 m (expected 0.18987: the skin sinks about 1 mm under load); no walking; the IMU reads 9.8066 m/s^2 up with a spread of 1e-6 (no jitter); the rest velocity -4.9 mm/s, which is **PHY-8's -g dt** (the velocity is sampled before the contact) |
| drop from 0.5 m | rebound 3.8 mm (restitution 0.1 predicts about e^2 x 0.5 m = 5 mm), then rest at 0.18888 m |
| tip-over, pitch | 25 degrees rocks back to 0.00 degrees; 40 degrees tips (turns 145 degrees in 1 s) |
| tip-over, roll | 25 degrees rocks back to 0.00; 45 degrees tips (turns 147 degrees) |
| slope, 20 degrees | at friction 0.6 it holds, creeping 3.4 mm in 2 s: exactly **PHY-8's creep**, g sin(20) dt^2 a step (the first run at 1 ms moved 6.7 mm, twice as much: 4 times a step, half as many steps); at friction 0.3 it slides 0.59 m in 1 s (tan 20 = 0.364 against 0.3) |
| spin after a level drop | 0.017 rad/s at 16 sweeps (and 0.79 at 4, kept as a diagnostic) |

PHY-8 (the velocity-level substep, not built) is the one named limit: rest velocity -g dt and a creep of g sin(slope) dt^2 a step on a slope. Both shrink with the step (500 us is a quarter per step against 1 ms) and vanish when `PHY-8` lands.

No defect was found in `rigid_contact`: everything above is the existing machinery with the right input (the design-frame spawn, 16 sweeps).

## 6. Effort and what is next

Core: none for this item (no engine change); a default sweep count that scales with the point count would remove the trap above, a suggestion, not a need. Physics: the checks are done (about 2); the remaining work is the `collision:` block in the link's model files, which the PX4-match variant and kit-model generator will carry, and the extra non-foot points for tip scenarios when a scenario needs them (about 2).

OpenFrame-5F is outside this: its build table has no landing gear (a bumper and TPU boots at the arm ends), so it needs its own point list before it takes off from a surface.

## 7. Open questions

1. Core: should the default sweep count scale with the number of points (4 at the default is enough for a cube's eight corners but not twelve points at 3 m/s)? Not blocking; the model sets its own.
2. Does the lifted-plane fallback ever need to be used? The lead's decision: only for the first link bring-up, never for the F1 bars. With the hull working, it is not needed.
