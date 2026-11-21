# Ground contact for take-off and landing: the X500's feet (O140 phase 1, item 4: design note)

Physics, 2026-11-21. Branch `physics/ground-contact-note`. Design only, plus one opt-in print in `tests/test_x500_kit_inertia.cpp` (`[kit-gear]` lines) that measured what is below. **This item grows past Physics: it needs an engine change that is Core's** (more than one contact proxy on a body). That is the main finding.

## 1. What is there

- A body's contact is **one sphere**: `proxy_radius` (`ModelType`, overriding the world's default, `D-S6-2`), centred on the body's origin, which for a vehicle is its centre of mass. Contacts are resolved by velocity impulses (`PHY-7`), and a resting IMU reads +g.
- The X500 kit model carries `proxy_radius` 0.377 m (`x500.model.yaml`). A drone resting on that sphere stands with its centre of mass **0.377 m above the ground**, and nothing about the sphere resists tilt: its contact normal passes through the centre of mass, so a tilted drone feels no righting torque on a sphere and rolls freely. Neither is what a drone on legs does.
- The kit model carries **no landing gear** as a solid (the notes file's `not_carried`), and its `proxy_radius` is a conservative bound for the airflow grid, not a stand.

## 2. What the real X500 stands on (measured, derived grade)

From the build table's frame meshes (`tools/flow_movie/data/x500_build/parts.yaml`, the `JIAO-EVA_1..4` objects, read from the local part store by the opt-in `X500KitInertia` test, the design frame of the table: +X forward, +Y up, +Z right, origin between the plates; the centre of mass is at (-0.0045, -0.0254, 0.0009) m):

| Foot (rubber pad) | x (m) | y (m) | z (m) | size |
|---|---|---|---|---|
| EVA 1 front-right | +0.0210 .. +0.1140 | -0.2153 .. -0.1963 | +0.1105 .. +0.1295 | 93 x 19 x 19 mm |
| EVA 2 rear-right | -0.1140 .. -0.0210 | same | +0.1105 .. +0.1295 | same |
| EVA 3 rear-left | -0.1140 .. -0.0210 | same | -0.1295 .. -0.1105 | same |
| EVA 4 front-left | +0.0210 .. +0.1140 | same | -0.1295 .. -0.1105 | same |

(The table's file names number the pads 1 to 4 in the file's order, not the rotors'; by their corners they are front-right, rear-right, rear-left and front-left.)

- Lowest point of the drone: **y = -0.2153 m**, the pads' bottom faces, all four level. The pads' centres are at y -0.2058, so each pad is a 19 mm square section.
- The pads hang from the skid tubes (`CARBON-FIBER-TUBE300_1/_2`, 250 mm long at z +-0.030) through the pylons (`PYLONS-X500_1/_2`) and the foot connectors (`JIAO-LIANJIE_1/_2`, boxes x -0.020 .. +0.020, y -0.212 .. -0.180, z +-(0.103 .. 0.127)).
- **Standing height: the centre of mass is 0.1899 m above the ground** (y -0.0254 to -0.2153). The propeller discs, at 0.0607 m above the table's origin, are 0.276 m above the ground. Footprint: 0.135 m between pad centres front to back, 0.24 m side to side.
- **Tip-over, derived from the footprint and the centre-of-mass height:** about 31 degrees nose-down or nose-up (pad ends at x +0.114 and -0.114 against a centre of mass at x -0.0045 and 0.19 m high) and about 34 degrees in roll (0.1295 / 0.19). A test tolerance of 3 degrees.

## 3. The shape proposed

**Several contact spheres per body.** Each pad is a 93 x 19 x 19 mm bar: model it as **two spheres of radius 9.5 mm** at its two ends, centred at y -0.2058 (so the spheres touch at -0.2153) and at x +0.0305 and +0.1045 for the front pads, -0.0305 and -0.1045 for the rear pads, z +-0.12. That is **8 contact spheres** for the X500, in the model file as a list in the body frame (`contact_points`: local position and radius), authored by the model builder from the table and graded derived (the mesh extents), like the rotors' hubs.

What that needs from Core (the engine's contact pass is built for one proxy per body):
1. A contact point row (or an array on the body row): body slot, body-frame offset, radius; at most a few per body (8 for the X500; the cap is Core's to set).
2. The contact pass iterating each point, with the impulse applied at the point (so it makes a torque about the centre of mass) and the existing friction and restitution rules.
3. `model_type` carrying the list, `ModelType::validate` refusing a radius the world's grid cannot afford (the existing `D-S6-2` rule, per point), the model file's key, and `model_identity` folding it **only when present** so no existing model's digest moves.
4. The body's own `proxy_radius` stays as the broad-phase bound and the airflow grid's size; with points present it is no longer the stand.

CPU first (the lockstep link runs on the CPU); a Vulkan world with a contact list is refused until a consumer wants a GPU twin, as for the baro and the mag.

**Alternatives and why not.**
- *One smaller sphere (0.190 m radius).* Gives the right resting height and nothing else: no footprint, no righting torque, so no tip-over and no sensible touch-down on a slope. Not enough for a landing.
- *A plane under the feet only.* A "standing height" with a plane contact would stand the drone at the right height but is blind to slopes and obstacles. It is the cheapest fallback if the engine change must wait: the world's ground plane is lifted by 0.190 m for this model. Not recommended past a first flight.
- *Spring-damper foot forces in the force-element list.* Contact is impulse-based by decision (`PHY-7`); a force path would be a second contact model.

## 4. Values and their grades

| Value | Number | Grade |
|---|---|---|
| foot geometry | the table above | derived (mesh extents; the table's meshes are the kit file) |
| pad sphere radius | 9.5 mm (half the pad's 19 mm section) | derived |
| rubber on a hard floor: friction | 0.6 | **assumed** (EVA foam on concrete or wood is about 0.5 to 0.8; a figure to cite in phase 6) |
| restitution | 0.1 | **assumed** (a foam foot on a rigid floor) |
| landing speed in the tests | 0.5 to 1.5 m/s | the tests' own choice |

## 5. Tests (RED first, on the engine change)

- **Rest height:** the drone placed on a plane settles with its centre of mass 0.1899 m +-1 mm above it; the IMU reads +g up (`PHY-7`); the velocity at rest reads about -g x dt until `PHY-8` lands (named, bounded).
- **Drop:** from 0.5 m the drone settles within 1 s without lifting more than the restitution allows (0.1 means about 1% of the height).
- **Tip-over:** placed at 25 degrees of pitch it rocks back to level; at 40 degrees it tips (the two thresholds from section 2, 3 degrees tolerance).
- **Slope and slide:** on a 10 degree slope it stays (friction 0.6 holds above tan 10 = 0.18).
- **Take-off and landing with the autopilot,** phase 1's scripted flight: arm, take off to 5 m, hold 60 s, land; no bounce above 5 cm on the touch-down, PX4's landed state reached.
- **Determinism:** the same seed twice gives identical traces, contact included.

## 6. Effort and ownership

Core: the contact point row and the pass, about 3 to 4 (it is the engine's contact code). Physics: the model builder's `contact_points` from the table, the notes with grades, the tests above, about 2. **Total about 4 to 5 for the item, so the 4 I gave in the phase 0 record was low once the engine change is counted.**

OpenFrame-5F is outside this: its build table carries no landing gear (a bumper and TPU boots at the arm ends), so it would need its own list of contact points before it takes off from a surface.

## 7. Open questions

1. Core: can a body carry several contact points in the current contact pass, or is that the engine change above? (My reading of `D-S6-2` and `proxy_radius` is that it cannot.)
2. Does the world's contact grid cell size (0.5 m in the tests, 0.01 to 0.02 m in the airflow case) constrain 9.5 mm spheres?
3. Is there a better single-body primitive than spheres, such as a capsule, in the engine's contact code already?
