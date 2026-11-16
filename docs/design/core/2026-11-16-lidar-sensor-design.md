# LiDAR sensor (L78a-c)

**Owner:** Core. **Status:** DRAFT for the lead's review, then Spade Host and Kat. Design only; nothing is built, and the build follows L74 and L61 (L78a).
**Inputs:** KAT's ask (`kat/.agents/research/interception/spadehost-lidar-ask.md`); the lead's answers L78b and Kat's acceptance L78c (`.agents/decisions.md`); `sensors/{kinds,imu,gnss,rings}.hpp`, `physics/rigid_contact.hpp`, `render/raymarch.hpp`. Base: master `da7a37bd`.
**Settled (L78b, L78c), restated, not reopened:** LiDAR is a Spade primitive; noise is off (KAT's SDK adds it); the scan is a buffer, not a ring; the ask is the OS1-128 (128 x 1024, +-22.5 deg, 10 Hz, 0.3-100 m); hull planes are computed once on the CPU; same-backend byte equality across replay, reset and restore is required; CPU-GPU is a stated band; an origin inside something gives status `origin_inside`, range 0; each body may carry a ray shape separate from its contact shape.

## 1. The kind and its descriptor

`sensor_kind::lidar = 3` (`count` becomes 4). `kinds.hpp` asks for a row, a sample, a noise stream and a poll path before the tag: the LiDAR has the first, second and fourth, and no noise stream because it draws nothing (the `kinds.hpp` text gets one line saying so). Its API is `add_lidar_sensor(BodyRef, LidarSpawn)` and `poll_lidar(...)`, the IMU and GNSS shape: a distinct `LidarSensorRef` (a compile-time type, so `poll_imu(lidar_ref)` is refused), two-phase spawn (slot reserved at once, row written at the next step boundary, `kind == none` until then).

`LidarSpawn`, all validated finite, `!(x >= 0)` spelling so NaN refuses:

| Field | Meaning |
|---|---|
| `beams`, `columns` | rows and columns of the organized scan (128, 1024) |
| `v_fov_max`, `v_fov_min` (rad) | beam 0 is the highest; beam b is at `v_max - b (v_max - v_min) / (beams - 1)`; even spacing (KAT's inference; a per-beam table is a later, unasked addition) |
| `az_start`, `az_span` (rad) | column c is at `az_start + c az_span / columns`, so a 360 deg scan has no repeated seam column |
| `rate_divider` | one scan every N substeps, IMU convention; 10 Hz at h = 1 ms is 100. KAT converts its Hz |
| `min_range`, `max_range` (m) | 0.3 and 100 |
| `mount_pos`, `mount_orient` | body frame from the centre of mass, `mount -> body`, as the IMU's. Mount frame: x forward, y up, z right (glTF, the airframes'); positive azimuth turns about +y |
| `want_hit_ids` | allocate and fill the hit id array |

The beams and columns are a fixed upper bound per world, declared like `capacities.sensors`: `capacities.lidar_sensors` (default 0) and `capacities.lidar_scan_rays` (131,072 for the OS1-128 at 1024 columns; 2048 columns is 262,144). A world with the default 0 allocates nothing. The whole thing is an opt-in module (`lidar_module()`, as `rigid_contact_module()`): a set without it is untouched bit for bit, snapshot layout and hash included.

Every direction is `(cos t cos p, sin t, -cos t sin p)` from a per-beam and a per-column pair of floats. The tables are built on the host at the boundary with `fp32_math` `sin32`/`cos32` (the airflow blade tables' precedent; no libm) and stored in a registered array, so CPU and GPU read the same bytes. They hold at most 2 x (beams + columns) floats.

## 2. The sample and the scan buffer

Per ray, in two arrays, index `beam * columns + column`:
- `range`: float32, metres. Written for `hit` and `below_min` (the true distance); 0 for the other statuses. No NaN or infinity ever enters the buffer, so a memcmp and a CPU-GPU diff stay clean.
- `status`: uint8. `0 no_return` (nothing within `max_range`, or the march budget ended; zero fill reads as no return), `1 hit`, `2 below_min` (the first surface is nearer than `min_range`), `3 origin_inside`.
- `hit_id` (optional): uint32, the world-local body slot, `0xFFFFFFFE` for the static world, `0xFFFFFFFF` for none.

The buffer is not a small-sample ring (131,072 rays x 5 bytes is 640 KiB; 1.1 MiB with ids). Each sensor owns ONE scan slot plus a header: `scan_index` (uint64, from 1, `rings.hpp`'s rule), `tick` of the step that completed it, `generation` (the sensor ref's), the sensor's world position and quaternion at the scan (what Pliska's Eq. 11-12 needs), `beams`, `columns`, `min_range`, `max_range`, and the cursor. `poll_lidar(ref, since_index, out)` copies the latest scan into caller spans when `scan_index > since_index`, with `dropped = scan_index - since_index - 1` (the ring's rule: scans overwritten before the caller looked). Every byte of the slot is written each scan, the unused tail included (zero), so there is nothing indeterminate to memcmp.

When: the pass `lidar.scan` is in the Sensors phase, beside `imu.synthesize` and `gnss.synthesize`, and a sensor fires on the substep that completes each `rate_divider`. The scan is instantaneous: every ray sees the poses at the end of that substep (`set_body_state` acts only at step boundaries, so the target is fixed within a step). No per-ray time; motion distortion is a later addition.

## 3. Rays

**World.** One SDF evaluation of the world at the ray origin per scan: `d <= 0` makes the whole scan `origin_inside`. Otherwise each ray sphere-traces from t = 0 to `max_range`, with the shipped constants (`kRaymarchSurfaceEpsilon` 1e-4 and a step budget). A hit before `min_range` is `below_min`; one beyond `max_range`, or an exhausted budget, is `no_return`. `render::sphere_trace` cannot be called: `sensors/`, `physics/` and `world/` include no `render/` header today, and its start-inside rule (march past the solid, report no hit) is the opposite of the one wanted here. The LiDAR gets its own loop in `world/` over the same `eval()` and the same constants; it is the one extra copy of a 30-line loop, and a test compares it with `sphere_trace` on rays that start outside.

**Bodies.** `set_body_shape` (and the new `set_body_ray_shape`, below) computes, on the CPU, once: a bounding sphere (centre, radius) and the hull's face planes `n.x <= b`, at most 28 (2V - 4 for 16 points), merged when coplanar; a skin offsets every `b` by the skin (a slightly larger, sharp-edged polytope than a rounded hull; stated, conservative). They are stored in a registered per-body array, so they ride snapshots. L68's rows hold only points and skin, no faces and no convexity check; this adds the check (a non-convex or degenerate hull gets a named refusal at `set_body_ray_shape`, and at `set_body_shape` the body is simply not visible to the LiDAR, with a code the host can read back).

Per scan, a prep pass moves each candidate body's bounding sphere and planes into the world frame (one rotation per plane, once, not per ray). Per ray, per body: reject by the bounding sphere (about 10 flops); on a pass, clip the ray's parameter interval against every plane (the slab test: `denom = n.d`, `dist = b - n.o`, entering planes raise t_in, exiting planes lower t_out; `denom == 0` with `dist < 0` is a miss). A hit is `t_in >= 0`, `t_in <= t_out`. The origin in a body's hull (any body but its own) makes the scan `origin_inside` like the world case. Sphere ray shapes use the bounding-sphere quadratic directly; a capsule is open question 3.

**Closest wins.** Bodies first, then the world traced only to `min(max_range, best body t)`; the lower t wins, an exact tie goes to the lower body slot (so the pick cannot depend on loop order).

**Ray shape.** A per-body `RayShapeMode`: `inherit` (default: planes from the contact shape, recomputed whenever `set_body_shape` changes it), `own` (`set_body_ray_shape(ref, ShapeDesc)`, same descriptor and limits as the contact shape, for KAT's smaller airframe hull) or `off` (invisible). A body without the `rigid_contact` module has only `own` or `off`. Mesh ray shapes are later (L78b).

**Own body.** The sensor's `body_slot` is skipped in the body loop. Nothing else of the vehicle is a separate body today.

## 4. Determinism

- Each ray is independent, reads no other ray's result, and writes only its own slots. So CPU threads (if used) and GPU workgroup sizes {32, 64, 128} cannot change a byte (the T9 invariance holds by construction; it gets the usual test).
- Same backend, memcmp-equal: replay from the same state; `reset` (restore plus `reseed`; nothing here uses a stream, so `reseed` changes no scan); restore. The last holds because the scan slot and header are registered state: after a restore the buffer equals the one the snapshot held, and later scans follow from the restored phase and cursor. Snapshots grow by 0.64-1.1 MiB per sensor; open question 1.
- CPU-GPU, a band, not bits. The causes: ulp differences in the SDF evaluation moving a hit by up to the epsilon band (1e-4 m x a few); a ray tangent to a silhouette or hull edge flipping between a grazing hit and a miss; a hit within an ulp of `min_range` or `max_range`. Proposed form, to be pinned by measurement on the first build and not assumed: on rays whose status agrees, `|d range| <= 1 mm`; rays whose status differs are a fraction of the scan we measure, every one within a stated angle of a silhouette, an edge, or a grazing incidence. KAT does not need bit equality (L78c); the band goes in the record with its measurement.

## 5. Where it runs, and what it costs

CPU: the reference pass, in `sensors/`, deterministic and portable. GPU: two dispatches in the step's command buffer, one thread per body (prep, in world frame) and one per ray (trace), writing the device scan slot; the host gets it through the same device-to-host mirror a poll uses today. A `sphere_trace` port is new Slang over `sdf_eval.slang`, not a copy of the render one.

An estimate to be replaced by a measurement, not a claim. Bodies are cheap: 131,072 rays x K bodies x about 12 flops is on the order of 1 ms per body on one CPU thread, plus the slab test on the few rays that pass the sphere (a 0.64 m body at 25 m subtends about 18 rays). The world is the unknown: it is SDF evaluations per ray times the program's per-evaluation cost, and a ray nearly parallel to a floor at low height takes many steps (the render reference's own measurement was that 81.5% of its SDF work was on misses). The arithmetic that matters is the firing step: a scan costing X ms lengthens ONE step by X ms, every 100 steps. At 1 kHz the step budget is 1 ms, so a 20 ms scan is a 20 ms hitch every 100 ms.

To measure first (Test/Docs' bench harness, after the build): `BM_LidarScan/{cpu,gpu}` at 128 x 1024 in three worlds (flat floor; a room with a box; the sim box with one hull body), reporting microseconds per scan, microseconds per ray, SDF evaluations per ray (mean and 99th percentile) and the share of rays that exhaust the budget, against the 100 ms interval; plus the GPU readback of 640 KiB. Decision rules before building anything extra: a CPU scan over 20 ms in the room world earns multithreaded rays (free by section 4); a GPU scan under 2 ms is not worth tuning.

## 6. Open questions

1. **Snapshot the scan?** Recommended: yes, registered (byte-equal restore, simplest to test). Cost 0.64-1.1 MiB per sensor per snapshot. Alternative: not registered, the slot reads empty after a restore (`scan_index` kept), and the first scan after it is the first byte-equal one. KAT's ABI keeps it out of its own snapshot either way.
2. **Depth.** One scan slot plus `dropped` (recommended; a batch of 250 steps holds 2 scans and the first is overwritten, reported), or two for overlap.
3. **Capsule and sphere ray shapes.** Sphere is free. A capsule contact shape needs an analytic ray test (about 20 lines) or the body is silently invisible by default. Recommended: include it.
4. **Step budget.** A budget-exhausted ray is `no_return`; low grazing rays over a floor can hit the budget. Which budget (the render's 1024 is generous; 128 may do) comes from the measurement.
5. **Hit id** content (body slot as above, or slot plus generation) and the allocation when `want_hit_ids` is false (recommended: the array is allocated for every sensor only when the world declares it).
6. **Beam order and azimuth sign.** Row 0 is the highest beam as on the OS1; azimuth is counter-clockwise about +y. KAT to confirm it needs nothing else; column order does not matter to the VoFOD detector.
7. **`generation`.** The ask wants a "generation stamp". Here it is the sensor ref's generation (a recycled slot's scan cannot be mistaken); if KAT means its own bind generation, the host adds that.
8. **Many bodies.** The per-ray cost grows with the number of ray-visible bodies in the world; there is no broadphase. Fine for interception (one target); a per-scan angular cull is a later addition if a scene needs it.
9. **Hitch.** Is a 20-100 ms single step every 100 ms acceptable for KAT's host, which steps in at most 100 ms? If it paces against the wall clock, rays split across the interval is a different design (asynchronous scan, poses frozen at the firing step).
10. **Non-convex hulls.** `set_body_shape` takes 16 points without a convexity check. Recommended: the ray shape refuses a non-convex or degenerate set by name where the caller asked for it (`set_body_ray_shape`), and the inherited one makes the body invisible with a readable reason.

Later, by use case (KAT section 8): intensity and reflectivity, multi-echo, attenuation, mesh ray shapes, motion distortion, per-beam angle tables. None is designed here.
