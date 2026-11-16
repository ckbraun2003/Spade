# LiDAR sensor (L78a-c)

**Owner:** Core. **Status:** DRAFT, reviewed by the lead; KAT's answers recorded as decided (L78f, 2026-11-16). Design only; nothing is built, and the build follows L74 and L61 (L78a).
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

**Bodies.** `set_body_shape` (and the new `set_body_ray_shape`, below) computes, on the CPU, once: a bounding sphere (centre, radius) and the hull's face planes `n.x <= b`, at most 28 (2V - 4 for 16 points), merged when coplanar; a skin offsets every `b` by the skin (a slightly larger, sharp-edged polytope than a rounded hull; stated, conservative). They are stored in a registered per-body array, so they ride snapshots. L68's rows hold only points and skin, no faces. The ray shape is the convex hull of the points (quickhull on 16 points or fewer, on the CPU, deterministic), so convexity holds by construction. Degenerate sets follow L78e (section 6, question 10): a skin above 0 turns 1 point into a sphere, 2 points into a capsule and 3 or more coplanar points into a slab; only a hull with no volume and skin 0 is refused, by name.

Per scan, a prep pass moves each candidate body's bounding sphere and planes into the world frame (one rotation per plane, once, not per ray). Per ray, per body: reject by the bounding sphere (about 10 flops); on a pass, clip the ray's parameter interval against every plane (the slab test: `denom = n.d`, `dist = b - n.o`, entering planes raise t_in, exiting planes lower t_out; `denom == 0` with `dist < 0` is a miss). A hit is `t_in >= 0`, `t_in <= t_out`. The origin in a body's hull (any body but its own) makes the scan `origin_inside` like the world case. Sphere ray shapes use the bounding-sphere quadratic directly, and a capsule has its own analytic test (question 3, decided).

**Closest wins.** Bodies first, then the world traced only to `min(max_range, best body t)`; the lower t wins, an exact tie goes to the lower body slot (so the pick cannot depend on loop order).

**Ray shape.** A per-body `RayShapeMode`: `inherit` (default: planes from the contact shape, recomputed whenever `set_body_shape` changes it), `own` (`set_body_ray_shape(ref, ShapeDesc)`, same descriptor and limits as the contact shape, for KAT's smaller airframe hull) or `off` (invisible). A body without the `rigid_contact` module has only `own` or `off`. Mesh ray shapes are later (L78b).

**Own body.** The sensor's `body_slot` is skipped in the body loop. Nothing else of the vehicle is a separate body today.

## 4. Determinism

- Each ray is independent, reads no other ray's result, and writes only its own slots. So CPU threads (if used) and GPU workgroup sizes {32, 64, 128} cannot change a byte (the T9 invariance holds by construction; it gets the usual test).
- Same backend, memcmp-equal: replay from the same state; `reset` (restore plus `reseed`; nothing here uses a stream, so `reseed` changes no scan); restore. The last holds because the scan slot and header are registered state: after a restore the buffer equals the one the snapshot held, and later scans follow from the restored phase and cursor. Snapshots grow by 0.64-1.1 MiB per sensor; decided (question 1).
- CPU-GPU, a band, not bits. The causes: ulp differences in the SDF evaluation moving a hit by up to the epsilon band (1e-4 m x a few); a ray tangent to a silhouette or hull edge flipping between a grazing hit and a miss; a hit within an ulp of `min_range` or `max_range`. Proposed form, to be pinned by measurement on the first build and not assumed: on rays whose status agrees, `|d range| <= 1 mm`; rays whose status differs are a fraction of the scan we measure, every one within a stated angle of a silhouette, an edge, or a grazing incidence. KAT does not need bit equality (L78c); the band goes in the record with its measurement, and it is measured on the rays the target hull covers (3 columns on a 0.5 m target at 25 m; KAT's request, L78f), not only on the world's rays.

## 5. Where it runs, and what it costs

THE GPU SCAN IS THE PRODUCT; the CPU scan is the reference and test path (L78f, question 9). CPU: the reference pass, in `sensors/`, deterministic and portable. GPU: two dispatches in the step's command buffer, one thread per body (prep, in world frame) and one per ray (trace), writing the device scan slot; the host gets it through the same device-to-host mirror a poll uses today. A `sphere_trace` port is new Slang over `sdf_eval.slang`, not a copy of the render one.

An estimate to be replaced by a measurement, not a claim. Bodies are cheap: 131,072 rays x K bodies x about 12 flops is on the order of 1 ms per body on one CPU thread, plus the slab test on the few rays that pass the sphere (a 0.64 m body at 25 m subtends about 18 rays). The world is the unknown: it is SDF evaluations per ray times the program's per-evaluation cost, and a ray nearly parallel to a floor at low height takes many steps (the render reference's own measurement was that 81.5% of its SDF work was on misses). The arithmetic that matters is the firing step: a scan costing X ms lengthens ONE step by X ms, every 100 steps. At 1 kHz the step budget is 1 ms, so a 20 ms scan is a 20 ms hitch every 100 ms. KAT's stage 2 is about 150,000 s of simulated time per set, 1.5 M scans: at 20 ms a scan that is 8.3 h, at 2 ms 50 min (L78f).

To measure first (Test/Docs' bench harness, after the build): `BM_LidarScan/{cpu,gpu}` at 128 x 1024 in three worlds (flat floor; a room with a box; the sim box with one hull body), reporting microseconds per scan, microseconds per ray, SDF evaluations per ray (mean and 99th percentile) and the share of rays that exhaust the budget, against the 100 ms interval; plus the GPU readback of 640 KiB. THE GATE FOR BUILDING ON (L78f): a GPU scan well under 10 ms in the sim-box world, measured first; a scan that misses it is a finding for the lead before any further work, not a reason to add a design. There is no asynchronous design. Decision rules for extras (kept, KAT R1110rd): a CPU scan over 20 ms in the room world earns multithreaded rays (free by section 4); a GPU scan under 2 ms is not worth tuning.

## 6. Open questions

1. **Snapshot the scan?** DECIDED (KAT, L78f): yes, registered state in snapshots. KAT's host world snapshot must reproduce poll behaviour bit-identically (`kat_host_abi.h` B13), and 0.64-1.1 MiB per sensor is fine. The empty-after-restore alternative is withdrawn.
2. **Depth.** DECIDED (KAT, L78f): one scan slot plus `dropped`. KAT polls every 10 ms against a 100 ms scan interval. A batch of 250 steps holds 2 scans and the first is overwritten, reported.
3. **Capsule and sphere ray shapes.** DECIDED (lead, 2026-11-16): both are in. Sphere is free; a capsule has an analytic ray test (about 20 lines), so a capsule contact shape is not silently invisible by default.
4. **Step budget.** A budget-exhausted ray is `no_return`; low grazing rays over a floor can hit the budget. Which budget (the render's 1024 is generous; 128 may do) comes from the measurement.
5. **Hit id.** DECIDED (KAT, L78f): the world-local body slot (values as in section 2), allocated only when the world declares it (a `capacities` field beside `lidar_scan_rays`, default 0; a sensor's `want_hit_ids` fills it, and is refused where the world declared none).
6. **Beam order and azimuth sign.** DECIDED (KAT, L78f): row 0 is the highest beam; azimuth is positive about +y; column c is at `az_start + c * az_span / columns`, with no repeated seam column; the mount frame is x forward, y up, z right. KAT confirms the direction formula `(cos t cos p, sin t, -cos t sin p)`; the header needs no angle fields, because KAT's SDK keeps its own `LidarSpawn`.
7. **`generation`.** DECIDED (KAT, L78f): the sensor ref's generation stays in the header as drafted. KAT's own "generation" is a per-publish sequence starting at 1, which `scan_index` already is; KAT stamps tick plus generation from Spade's `tick` and `scan_index`.
8. **Many bodies.** The per-ray cost grows with the number of ray-visible bodies in the world; there is no broadphase. Fine for interception (one target); a per-scan angular cull is a later addition if a scene needs it.
9. **Hitch and backend.** DECIDED (KAT, L78f): this decides the backend. Stage 2 is about 150,000 s of simulated time per set, 1.5 M scans: at 20 ms a scan that is 8.3 h, at 2 ms 50 min. So the GPU scan is the product and the CPU scan is the reference and test path. The gate for building on is in section 5 (a GPU scan well under 10 ms in the sim-box world, measured first). No asynchronous design. Kat (R1110rd) finds the synchronous scan acceptable, since KAT steps without a wall clock.
10. **Degenerate hulls (L78e). DECIDED (lead, 2026-11-16).** The ray shape is the convex hull of the points (quickhull, 16 points or fewer, CPU, deterministic). L68 accepts 1 to 16 points plus a skin, so a 1-point and a 2-point hull are legal contact shapes today and stay so. With a skin above 0: 1 point is the sphere ray test, 2 points the capsule test, and 3 or more coplanar points a slab of thickness 2 x skin (the 2D hull's side planes plus two faces). With a skin of 0, only a hull with no volume (it cannot be hit) is refused, by name. KAT's use (L78f): it calls `set_body_ray_shape` (mode `own`) with the airframe's `contact_shape` hull, 16 points, convex. The refusal exists only in a set that holds the lidar module, so a set without it stays bit-identical; it holds for an inherited shape too, so no body is silently invisible. An earlier draft of this item (refuse a non-convex set, hide an inherited one) is withdrawn.

Later, by use case (KAT section 8): intensity and reflectivity, multi-echo, attenuation, mesh ray shapes, motion distortion, per-beam angle tables. None is designed here.
