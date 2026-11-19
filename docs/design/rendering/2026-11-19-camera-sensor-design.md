# Camera sensor: a batched ray-traced camera for KAT (design, for KAT's review before any code)

**Owner:** Rendering. **Status:** draft for review, 2026-11-19. Nothing here is built. **Decided by Cameron (KAT D189):** Spade builds KAT's mass camera renderer, modelled on HyperDrone/HyperLux, after the L106 windows and before LiDAR. **Not decided here:** anything marked OPEN; the spike (section 8) decides the rest before a line of the technique is written.

**Inputs:** KAT's answers to the six asks (section 1, quoted from the lead's relay); Eschmann and Loianno, *HyperDrone: Simulation-to-Reality Transfer at Millions of Frames per Second*, arXiv 2610.28950 v1 (CoRL 2026), read as the HTML version, and the repo `github.com/rl-tools/hyperdrone` (MIT), read only; `01-techniques-and-channels.md` (camera, channels, grades), `02-scene-and-appearance.md`, `03-verification.md`, `00-decisions.md` (`RS3`, `RS4`, `RND-3`, `RND-4`, `A2`, `A5`, `A6`), `../core/2026-11-16-lidar-sensor-design.md`, `engine/sensors/kinds.hpp`, `engine/compute/`.

**Reading rules for the sources.** The HTML text of the paper doubles many numbers; every figure below was de-duplicated, and the three training-cost figures (about 4 h, 6 GB, 15 h) are marked as read, not confirmed against the PDF. The `hyperdrone` repo holds the README, a notebook and a submodule pointer (`rl_tools`, not fetched); the renderer's code was not read, and no code was copied. Where the paper does not say, this document says "not stated", and the spike measures it for Spade's own scenes.

## 1. What KAT asked for (inputs, in KAT's priority)

1. **Operating points.** P1: 128 envs, one camera a drone, 30 Hz; two configs, 320x180 colour and 384x384 grey fisheye; target at least 12,000 renders/s per camera type on the RTX 3060 Ti 8 GB (128 envs x 30 Hz x about 15 s is about 58,000 renders an iteration, to stay under about 2x today's iteration of about 10 s). The spike measures exactly these two. P2: single native views for the editor window and offline datasets, 640x360 colour and 848x800 grey, throughput secondary. P3 (research): 256 to 1024 envs at 128x128 or less.
2. **Lenses.** AIGP: 640x360, fx = fy = 320, cx = 320, cy = 180, pinhole, no distortion, colour, about 30 Hz; the tilt is a per-camera mount value, open. T265: 848x800 grey, global shutter, about 163 deg diagonal FOV, 30 Hz, Kannala-Brandt 4 (KB4) with k1..k4 as inputs; KAT forwards calibrated values later.
3. **Layout.** Images NCHW uint8 (C = 1 grey, 3 colour); instance id uint16 a pixel; corners float32, fixed size a camera: G slots x 4 x (u, v, visible), plus an id a slot. Depth later (LiDAR shares it). No normals, no flow.
4. **Labels.** KAT's gates are CSG prefabs (outer box minus inner box, material "gate"); gate id is the track element's index; the corners are the four inner aperture corners in gate-local order (+x,+y), (+x,-y), (-x,-y), (-x,+y) from half_width and half_height. Spade stays general: the API takes labelled keypoints per instance (any object, any count) and KAT supplies the gate corners as keypoints. No KAT names in Spade. Colour uses the existing material for now; jitter and lighting variety are later knobs; no textures yet (D179 approved them 'once Spade can'; item 7 is what needs them).
5. **KAT owns the CUDA import and the DLPack side.** Spade stays CUDA-free.
6. **Timing.** A camera-rate divider like the IMU's; each frame is stamped with its capture tick; blur is sampled across the shutter.
7. **Image realism: "wide randomisation" (Cameron, KAT D190, confirmed).** The owner's words: "textures, backgrounds, heavy lighting changes and distractors", chosen per episode. KAT's detail: (a) textures on gate frames and the ground from a pool (D179 already approved textures, "once Spade can"; D190 needs them); (b) background variety; (c) lighting ranges (sun direction and intensity, ambient, exposure); (d) distractors placed per episode (gate-like orange and grey rectangles, poles). Why beyond detection: the owner's fallback, if IMU plus gate corners cannot meet the bar, is image-based visual odometry from these renders, which needs textured, feature-rich scenes (section 6a, "The visual-odometry fallback"). Kept general in Spade's terms: seeded scene randomisation of materials, textures, lights and extra objects that any project can use. Determinism (the same seed gives the same pixels, the CPU oracle matching) is Spade's own law (`L3`, `L4`), applied here, not part of D190's scope.

## 2. What HyperDrone does that matters, and why

| Fact (source) | Why it matters here |
|---|---|
| Simulation and rendering both run on the GPU in one process, so no CPU-GPU transfer sits in the loop (section 3.1, Appendix C: "Simulation, rendering, teacher and student run in one process on one GPU") | The whole point. The learner must read pixels the renderer wrote without a host copy. Sections 6 and 7 are Spade's version of it |
| Frames are rendered "from scratch" every step on a hardware ray-tracing backend (OptiX; also Vulkan, Metal, and a software WebGPU path 9x to 27x slower on the same card, as read) (3.1, Table 2) | A ray query per pixel has no per-frame raster state to rebuild, so a moving scene costs only an acceleration-structure update. The Vulkan backend is the proof it works without CUDA |
| Batching is by scenes and instances: a scene is a world, an instance is one set of drones in it; training uses 64 instances in each of two scenes, 128 environments (Appendix C); the example is 3 worlds x 4 agents at 1024x1024 (Appendix E) | Same shape as KAT's 128 envs. Many envs share one scene description; that sharing is where memory is saved |
| Scenes are content-addressed (hash) and fetched once into a shared cache (Appendix E) | Dedupe static geometry by content hash across envs |
| Throughput at 64x64, 4096 envs (Table 2, as read): RTX 4090, ProcTHOR RGB, 1.75 M frames/s at the Low fidelity, 1.30 M Medium, 0.57 M High; Madrona 0.28 M; VisFly (4096) 2,780. RTX 5090 Vulkan: 6.56 M Low, 6.79 M Medium, 4.96 M High. Physics adds little (569,534 render against 564,210 with physics, as read) | The scale to aim at, and proof the render is not slowed by the step. Fidelity matters: Low is baked colours, High is full PBR with reflections. Spade's look today (Lambert + ambient + sky) is nearer Low; with section 6a's textures it is nearer Medium (textures, one reflection on metal in HyperDrone), without the reflection |
| Output API: `read()` returns `(instances, views, H, W[, C])` and accepts caller `out=` buffers, "CUDA arrays on OptiX"; probe readback "zero-copy, like the image outputs" (E, D) | Caller-owned output memory, as `PA-1` already says. The learner passes its own buffer, or takes a pool slot |
| Cheap augmentation inside the renderer: 2x2 rays a pixel (anti-aliasing), N rays across the shutter (motion blur, N = 2 for training, 16 in the example), brightness scale U(0.5, 1.5), exposure mismatch U(0.75, 1.25), FOV +-5 deg, mount +-1 cm and +-5 deg, global shutter only, no rolling shutter (4.1, Table 5, section 5) | The right list of knobs. Blur and AA are extra rays, so they are a cost multiplier the budget must name |
| Outputs: RGB, depth, semantic and instance segmentation, normals, flow (4.1, E) | KAT needs only image, instance id and corners now; the rest is a channel list, not a redesign |
| Deterministic: the same seed on the same hardware and backend gives the same results; cross-hardware and cross-backend determinism not stated (4.3) | Spade's rule is the same (`L3`, `L4`): same backend, byte equal; across backends a band |
| Not stated: how the acceleration structure is organised across instances, per-scene memory, VRAM of the renderer alone, fisheye or any lens distortion, keypoints, VK_KHR_ray_query by name, a frame-time breakdown | Every one of these is a spike question (section 8). HyperDrone cannot be copied for them, and has no lens model Spade needs |

What it does not give us, so the design adds it: a KB4 lens, projected labelled keypoints with visibility, a stamped camera rate, CSG shapes, and an external-memory export (HyperDrone's zero copy is CUDA-native; ours must cross Vulkan to CUDA, section 7).

## 3. The technique and where it sits

A camera is a technique plus channels (`01-techniques-and-channels.md`). This one is **ray-trace, on Vulkan compute with `VK_KHR_ray_query`**, the one cell the techniques table has as "not designed yet", and the first user of `RND-4`'s Vulkan resumption that is a sensor rather than a viewport. Findings from the tree:

- The Vulkan context enables no ray-tracing and no external-memory extension today (only `VK_EXT_debug_utils` and `VK_EXT_memory_budget`). The raster in `render/vulkan/` is background only and in no build graph. So the camera is a new technique on the existing compute backend (Slang kernels, `compute/vulkan/`), not a resumption of that raster. Whether Slang's `RayQuery` compiles to SPIR-V here is checked in the spike.
- `kinds.hpp` says a sensor kind is added only with its row, sample, noise stream and poll path, and records that `CameraComponent` (`objects/component.hpp`, id 7) exists with no sample type. The camera kind follows that rule; this document does not add a tag.
- The hardware, an RTX 3060 Ti (Ampere, second-generation ray-tracing cores) on driver 572.83, exposes ray queries. The spike checks the feature bits (`rayQuery`, `accelerationStructure`) and refuses with the reason where absent; the fallback is announced (`L6`), and is the CPU reference, not a silent change of picture.
- **Grade (`RND-3`):** the CPU reference ray tracer is `reference`; the Vulkan camera is `banded` against it, with a colour oracle before it is selectable (`03-verification.md`). A GPU frame is never a golden (`L4`).

## 4. Scene representation (shared with the future LiDAR)

The render scene is already split into draw items, one per union leaf or whole CSG root (`SR-9`), each with bounds, a material and a body or static owner; meshes (glTF props, body meshes) are separate items (`RS2`). The camera consumes that, in three kinds of geometry:

| Geometry | Device representation | Hit test |
|---|---|---|
| Triangle meshes (drone bodies, props) | One bottom-level structure (BLAS) a distinct mesh, built once, shared by every instance | Hardware triangle test |
| SDF shapes and CSG subtrees (gates, boxes, spheres, any union leaf) | One axis-aligned box a draw item (a BLAS of procedural boxes), carrying the item's SDF sub-program (`compute/sdf_program.hpp` already holds the device image of a world's program) | The ray query hands the shader the box candidate; the shader sphere-traces the item's SDF inside the box (the `RS3` rule: CSG is marched per pixel within its bounds) with `kRaymarchSurfaceEpsilon` 1e-4, and commits the hit. A gate (outer box minus inner box) is one box and a short march, exact to the epsilon |
| The analytic infinite ground (`SR-17`) | None; a closed-form plane test in the shader before the query result is read | `t = -o.y / d.y` against the nearest query hit |

**Instances.** The top level (TLAS) holds one instance per body or placed static: a transform, a BLAS reference, an 8-bit mask, and a 24-bit custom index that points at the instance's label, material and sub-program. A static gate placed at a different pose in each env is the same BLAS with a different transform, so per-env memory is the instance records (64 bytes each in the structure), not geometry.

**How the 128 envs share the structure (OPEN, the spike decides):**
- **L, a lattice TLAS.** One TLAS holds every env's instances, each env translated to its own cell of a virtual lattice (rays carry the same offset); one build and one trace dispatch a frame, no descriptor array. Exact if the cells are farther apart than the ray's far limit, and the lattice stays small enough for fp32 (1,024 envs on a 32 x 32 grid of 100 m cells is about 3.2 km, and a hardware transform error of about 0.2 mm at that range; the spike measures it against the 1e-4 epsilon).
- **E, a TLAS an env.** An array of structures, a ray selects its own. Exact with no precision cost, but needs a descriptor array of acceleration structures and 128 small builds in one batched call.
- Statics change only at reset, so their BLAS and the static part of the TLAS are built then; only moving instances are updated each frame. The lean is **L**, if the precision holds.

**Shared with the LiDAR.** The scene description and its device image are the same object; a camera and a LiDAR are two ray generators and two result writers over it. The LiDAR design (`../core/2026-11-16-lidar-sensor-design.md`) today marches the SDF with its own 30-line loop inside `world/`, because `sensors/` includes no `render/` header. OPEN, for Core and Rendering to settle with the lead: where the shared ray scene lives so dependencies keep pointing downward (this document proposes a small library under `engine/` that links `world` and `render` and that `sensors` may call through its device passes, not a `render/` include from `sensors/`). The LiDAR is not designed here; it is a reason to keep the scene image free of any camera type.

## 5. Lens maths, rays, and the pipeline of one pixel

**Convention.** Pixel (i, j) has its centre at (i + 0.5, j + 0.5), origin at the top-left corner; so AIGP's cx = 320, cy = 180 on 640x360 is the exact image centre. Camera frame: x right, y down, z forward (the mount from the body is a per-camera rigid transform and a per-camera tilt; Spade's own world frame is untouched). OPEN, KAT: confirm the sign of the tilt and the mount frame, as L78f did for the LiDAR.

**Rays from a table.** For every camera type the host computes, once, in fp64, a table of unit ray directions a pixel (320x180 x 12 bytes is 0.7 MB; 384x384 is 1.8 MB), shared by every env that uses that type. The trace kernel never evaluates a lens; pinhole and fisheye take the same path, and a new lens is a new table.
- **Pinhole:** d = normalize((x - cx) / fx, (y - cy) / fy, 1).
- **KB4:** with r = sqrt(a^2 + b^2) and theta = atan2(r, 1) for a normalised point (a, b), the image radius is theta_d = theta (1 + k1 theta^2 + k2 theta^4 + k3 theta^6 + k4 theta^8), and the pixel is (fx a theta_d / r + cx, fy b theta_d / r + cy). Unprojection solves theta_d for theta by Newton's method with a fixed iteration count and a fixed start, so it is deterministic. 163 deg diagonal means a half angle of about 81.5 deg, so every ray is in front of the image plane (theta < 90 deg); a lens beyond 180 deg is refused by name.
- The table is per camera type, not per env; per-camera FOV and mount jitter (KAT's later knobs) change the mount transform, not the table. A per-env intrinsics change is OPEN (it would need a table an env).

**Forward projection (corners).** The same maths forward, used for keypoints (section 6). It needs an arctangent. The engine rule is no libm in engine source (`SR-14`); the fp32 kernels in `spade::math` are used if they have an `atan`, and a bounded polynomial added with its own pinned error if not. OPEN, to confirm in code before Phase 1. The CPU and GPU forms are pinned against the fp64 reference to a stated pixel error (proposal: 0.01 px).

**One pixel.** `ray (origin from the camera pose at the sample time, direction from the table, rotated by the camera orientation) -> query (the TLAS, ray-flags opaque, the camera's mask) -> candidate boxes march their SDFs, triangles test in hardware -> nearest hit and ground -> shade (material, sun, ambient, sky, atmosphere: `shade_vertex_color`'s maths ported, `RND-1`'s sun convention) -> exposure -> noise -> sensor mode -> quantise -> write NCHW`.
- **Shading.** Lambert + ambient + sky + the atmospheric term (`SR-17a`), the one definition of a lit pixel ported to the shader (`01-techniques-and-channels.md` rule). **Shadows are off in v1:** a sun-visibility ray is one extra ray a hit (about double the cost at one sample), and it differs from the orthographic map the raster uses (`RS6a`). A flag, default off; whether KAT's training wants shadows is an ask (section 10). Textures: from section 6a's pool when the appearance set has one (D179 approved them; `RND-13` below); a surface with none shades as above.
- **Sensor mode.** `colour` (3 channels) or `grey` (1 channel, luminance Y = 0.2126 R + 0.7152 G + 0.0722 B of the shaded linear colour, then the same exposure and quantise). The colour path before quantise is float32 in linear light; the write is uint8, rounded to nearest, with no gamma curve unless KAT asks (OPEN, a per-camera flag).
- **Exposure.** A per-camera linear gain applied before clamp, so a later "exposure jitter" is a seeded gain a frame (the HyperDrone brightness scale), not a different pipeline.
- **Noise.** Off by default (the LiDAR's rule, L78b: the SDK adds noise where the learner wants it). When on, a signal-dependent Gaussian (variance a * I + b, I the exposed value) from a **counter-based generator keyed by (seed, camera, capture tick, pixel, channel)** through `core/rng.hpp`'s stream rules, so the noise is independent of dispatch order, thread count and batch size, and a replay or restore repeats it. The CPU reference computes the same integer hash, so the noise matches exactly wherever the shaded value does. The Gaussian transform uses no `log` or `cos`; a rational approximation or a sum of uniforms, with its error stated.
- **Motion blur and the shutter.** Global shutter only (KAT's T265; no rolling shutter, as in HyperDrone). A frame averages N rays across the exposure window, N a per-camera integer (1 off; HyperDrone trains at 2). Sample k uses the camera pose at time t_cap - shutter_fraction x frame_period x (1 - k/(N-1)), interpolated between the published poses (position linear, orientation slerp done with the engine's fp32 kernels). **v1 blurs by camera motion only**: the scene is posed at the capture tick, so a moving body other than the camera's own is not blurred (a TLAS update a sample is the later price). OPEN, KAT: does P1 have other moving bodies in view?
- **Anti-aliasing.** 1 ray a pixel by default; 2x2 is a knob. Rays a pixel = AA x blur samples, and every cost number below says which.
- **Self-visibility.** A per-camera mask removes the carrying body (and chosen others) from the camera's view, through the instance mask bit, so a T265 with an arm in view and an FPV with none are the same code.

## 6. Labels: instance ids and keypoints

- **Instance id, uint16 a pixel.** 0 is "no labelled object" (sky, ground, unlabelled geometry); a labelled body, prop or static carries a host-set label (uint16, nonzero), stored in the instance's custom index. KAT sets a gate's label to its track index plus one; Spade attaches no meaning to the number. Two instances with one label are one object (a gate built of several shapes keeps one id). The id is the id of the **nearest hit** of the primary ray; with N blur samples, the id of the middle sample (OPEN: or the majority; the spike checks they differ rarely).
- **Keypoints.** An object carries a list of labelled keypoints in its own frame: (slot, local point). The API is general: any object, any count up to a capacity. Per camera and frame the output is, per slot, `(u, v, visible)` from the forward projection of the posed point, plus the slot's object label. A slot with no object, or whose object is behind the camera or outside the image, is `visible = 0` with u and v zeroed (every byte of the output is written, as the LiDAR's rule is).
- **Visibility is one occlusion ray**, camera to the point, any-hit: visible if the point is in front of the image plane and inside the frame and no surface is nearer than the point by more than `occlusion_epsilon` (default 2 mm, plus 1e-4 times the range). The epsilon is not optional for KAT's use: an aperture corner lies **on** the gate's own surface (the inner box's corner is on the outer box's cut), and a ray to it must not report its own gate as an occluder. The march epsilon (1e-4 m) is below it by design.
- **KAT's gates:** KAT supplies four keypoints a gate, in the order of ask (4), from half_width and half_height, as data. Nothing about gates, tracks or "AIGP" is in Spade. Fixed size: G slots x 4 is KAT's layout; Spade's capacity is "keypoints a camera", and the buffer shape is `[N, G, 4, 3]` float32 plus `[N, G]` labels (uint16; widened to int32 if KAT wants -1 for empty, OPEN).
- Depth, normals, flow: not in v1. The channel list is open for them; depth is the one the LiDAR shares, so the hit record keeps the distance (float32) even when no depth output is requested (a depth channel is then a write, not a redesign).

## 6a. Seeded scene randomisation (D190: textures, sky, light, distractors)

**What it is, in Spade's terms.** An **appearance set** is a render-only description of what may vary: a material pool (colour ranges, shading flag), a texture pool, a sky pool, ranges for the sun, the ambient term and the exposure, and distractor templates. An **episode draw** is a pure function of (set, world seed, env index, episode counter) that picks one concrete **appearance record** for each env: which texture each surface class wears, the sky, the light numbers, and the distractor instances. It happens on the host at env reset, in integer and fp64 arithmetic with the engine's counter-based generator, and the record is written into the env's scene image. So the CPU oracle and the GPU see **exactly the same scene**: the draw cannot disagree, and what is banded is the shading of that scene, not what was chosen. The record is also readable by the caller (chosen indices and numbers), so KAT can log an episode and replay it from the seed. It is render-only (the `RS5` split: Core owns the keys, Rendering the meaning); nothing in it touches stepping (`L5`), and a world without it is untouched bit for bit.

**Streams.** Each aspect (textures, sky, light, distractor count, distractor placement, per-frame exposure and noise) has its own rng domain tag (`kinds.hpp`'s rule: a tag is pinned once a recorded digest contains it), so adding a distractor class never re-seeds the textures. Per-frame variation (exposure jitter, noise) is keyed by the capture tick, as in section 5.

**(a) Textures.** D179 already approved textures; Spade's own rulings still say no. `RS6` ("no textures, PBR or post-processing") and `RS15` (textures out of scope) are amended for the camera sensor by a new ruling, proposed `RND-13`, which says so in one line each; the editor viewport and the CPU and GL rasters keep today's look until someone asks.
- **Where the texture coordinates come from.** Triangle meshes use glTF `TEXCOORD_0`. SDF shapes and CSG subtrees have no UVs, so they are mapped **triplanar in object space** (the dominant axis of the hit normal picks the projection, no blend in v1) at a stated texels-per-metre, which is exact on boxes (gates) and deterministic. The ground is planar in world x, z with a per-episode offset, rotation (quarter-turns plus a seeded angle) and scale.
- **Storage and memory.** The pool is one storage buffer of RGBA8 texels with a mip chain a texture, plus a table (offset, width, height, mip count). 128 textures at 512x512 with mips is about 170 MB; 64 at 1024x1024 about 340 MB; the proposal is at most 256 MB for the pool at 128 envs, OPEN with KAT because the learner shares the card. Block compression (BCn) is not used: decoding it bit for bit on the CPU is an extra oracle to maintain, and the pool is small enough raw. A grey camera reads a luminance-only copy of the pool (a quarter the bytes, built at load), so section 8's grey fisheye does not pay for colour it discards.
- **Filtering, and how the band is kept.** **No hardware sampler.** The shader reads texels with exact integer loads from the buffer and filters them itself: bilinear within a mip and a linear blend between two mips (trilinear), with the fractional weights quantised to 8 bits so the arithmetic on both sides is integer after the one place a float enters, the conversion of (u, v) to a texel position. The CPU reference runs the same code path in fp32 for that conversion, so a texel fetch differs only where a coordinate sits within one fp32 rounding of a texel boundary; the filter test samples a dense grid of (u, v) on both and pins the result (equal except at most 1 level of 255, counted). The mip is chosen from the pixel's footprint (the lens table's pixel solid angle times the hit distance over the cosine of the angle to the surface, converted to texels), with its log2 taken from the float's exponent bits (no libm), so no ray differentials are traced. sRGB texels are converted to linear by a 256-entry table shared by both paths.
- **Sources.** Textures arrive as decoded pixel arrays through the API (KAT decodes with its own tools; Spade gains no image codec), or are **generated by Spade** from the seed: checker, stripes, value noise and a few feature-rich procedural patterns, from integer hashes. The procedural ones need no assets, have no licence question, and are the first pool.
- **Cost.** A bilinear fetch is four loads, a trilinear eight, and 128 envs wearing 128 different textures is a cache-unfriendly pattern. The spike's textured variant measures it against the untextured one; if it misses the target the lead and KAT see the number (`A6`).

**(b) Backgrounds.** Variety comes from two things in v1: the **sky gradient** drawn from ranges (today's `sky_gradient_color`, keyed on ray elevation: zenith and horizon colours, so the atmospheric term `SR-17a` uses the drawn colour), and **far geometry** (a horizon wall, a skyline of boxes) as a distractor class (d), textured by (a). Cube-map skies (a pool of panoramas, six faces chosen by the dominant axis of the ray, so no arctangent; about 25 MB for 16 at 256x256) are **not** in 6c: they are open item 14 for KAT, built only if KAT's own design asks.

**(c) Lighting.** Sun azimuth and elevation, intensity and tint, the ambient term, and the exposure gain are drawn from ranges; the sun vector is built on the host with the engine's fp32 sine and cosine (`SR-14`), then it is one number set in the record, and `RND-1`'s convention holds (`sun_direction` points toward the sun). Shadows stay an optional shadow ray, default off (section 5); with a randomised sun the shadows matter more to realism, so the question to KAT (open item 4) is sharper.

**(d) Distractors.** A **template** is a prefab (a set of SDF shapes, or a mesh) with a count range, a placement region (a box or a ring in the env's frame), size, colour and texture ranges, a label (default 0 unlabelled; a separate label per class if the learner wants hard negatives marked), and keep-clear volumes the caller supplies (the gate apertures, the flight line) that no instance may enter. Placement is **rejection sampling with a fixed iteration budget** from the template's stream, so the result is deterministic and a failed draw degrades to fewer instances, announced in the record, never to an unbounded loop. Distractors are **render-only** (collision-free props, `RS5`); a physical pole is a world-file object and Physics' matter, and KAT says if it wants any. Each is an instance in the TLAS (a pole or a rectangle is one procedural box with a short march, so the cost is the instance records); capacity is `capacities.camera_instances` an env (proposal 64), and the lattice cell (section 4) already sets the region.

**(e) Determinism (Spade's `L3`, `L4`; not a D190 term).** The draw is a host function shared by both paths; the per-pixel shading is banded as in section 9; the textured and randomised frames get their own CPU goldens for fixed seeds (CPU only, `L4`), and a test draws the same (set, seed, env, episode) twice, in two processes, and after a reset, and compares the record byte for byte.

### The visual-odometry fallback (D190)

If IMU plus gate corners cannot meet the bar, KAT falls back to image-based visual odometry from these renders. Whether the frames are good enough is KAT's judgement, not something this document can assert. What VO needs from the camera, and where each is met:
- **Textures fixed to their surfaces for the whole episode.** The draw is at reset only (6a); the only per-frame variation is exposure and noise, so a feature seen at frame k is at the same surface point at k + 1. The triplanar and planar mappings are functions of the object-space or world point, so they do not slide as the camera moves.
- **Enough texture contrast.** The procedural pool entries are built with a stated contrast and multi-scale content (value noise at several octaves, not only flat checkers), and the pool load reports per-texture RMS contrast and gradient energy as advice. The real test is KAT's own detector on the sample frames below; the contrast numbers are a way to find a bad texture, not a pass mark.
- **A stable rate and stamp.** The camera divider (section 1, item 6), the capture tick in every frame's header, and a `dropped` count as the sensor rings keep it, so VO never has to infer a frame interval or a gap.
- **Intrinsics exposed.** The record and the stamp header carry the camera's true intrinsics (pinhole or KB4 numbers), its mount pose, and any per-episode jitter actually drawn, so VO can run with the calibrated values or with the true ones in a test.
- **Global shutter, blur that follows camera motion, and a known exposure** (the gain is in the record), which a tracker needs to be honest about brightness constancy.

**Sample frames for KAT's tracker.** The spike's textured variant (section 8) writes short frame sequences: a fixed fly-through path over the ground and a few gate textures from the procedural pool, at the P1 colour and grey-fisheye configs, with the intrinsics and the per-frame tick and pose beside them, as PNGs under the Footage folder. KAT runs a feature tracker on them before Phase 6c is sized for real. This comes after KAT's review and the lead's go for the spike, not before.

## 7. Export: zero copy to KAT's CUDA

Spade stays CUDA-free; the export is Vulkan's, and the CUDA side is KAT's (ask 5).

- **A flat buffer, not an image.** The pixels are written by the compute shader straight into a **linear `VkBuffer` in NCHW** (images `[N, C, H, W]` uint8, ids `[N, H, W]` uint16, corners and labels as above), allocated with external-memory export. CUDA maps a buffer as a plain device pointer, so DLPack on KAT's side is a view, with no tiling, swizzle or format change to undo. (A `VkImage` would need an optimal-tiling-to-linear copy first.)
- **Handles.** `VK_KHR_external_memory` + `VK_KHR_external_memory_win32` (and `_fd` on Linux), plus `VK_KHR_external_semaphore` + its win32 or fd form for ordering. On Windows the handle is an opaque Win32 handle (CUDA's `cudaExternalMemoryHandleTypeOpaqueWin32`); on Linux an opaque fd. The semaphore is a **timeline semaphore** where the CUDA version imports one (a binary pair otherwise): the renderer signals value = capture tick when a slot is complete; KAT waits on it, and signals a second one when it has copied or consumed the slot. Spade exports handles and values through the C ABI; it does not call CUDA.
- **Same GPU.** The import works only on the same physical device. The export reports `deviceUUID` and `deviceLUID`; KAT checks its CUDA device matches and refuses by name if not. A machine with two GPUs needs an explicit choice (OPEN: Physics' one-GPU scheduling, section 9).
- **A frame pool of K slots** (the engine's missing frame pool, `A2`): a slot stays valid until released; at least two outstanding; the camera pass takes the next free slot and never recycles one in use. K = 3 by default (render, consume, spare). Memory (arithmetic, not measured): P1 colour is 128 x 3 x 180 x 320 = 22.1 MB of image and 128 x 57,600 x 2 = 14.7 MB of ids, 37 MB a slot, 111 MB for three; P1 fisheye is 128 x 147,456 = 18.9 MB of grey and 37.7 MB of ids, 57 MB a slot, 170 MB for three. P3 (1,024 x 128x128) is 16.8 MB of grey and 33.6 MB of ids. All small against 8 GB; the budget that matters is the acceleration structures and the learner's own memory (section 8).
- **Fallbacks.** If external memory is not offered, the camera still renders into a Spade-owned buffer and a host readback path (a copy, announced, `L6`) serves the editor window and datasets (P2). The readback is the pipelining item already on the backlog (a ring of pixel buffers); it is not the training path.
- **What the timing stamp is.** Each slot carries (capture tick, camera index, a generation, the camera pose at capture), written by Spade beside the buffer in a small header KAT reads, so the learner knows which physics tick a frame belongs to without asking.

## 8. Measure first: the spike, and the budget

**The spike is a gate (`A6`: a camera sensor is benched against its rate envelope when it is built, and a miss triggers design review).** It is a standalone program in the bench tree, not wired into Core, and it answers these before the technique's design is committed. All on the RTX 3060 Ti 8 GB through the queue, one `gpu` ticket a run, `--vram-gb 4`.

| Question | Measure |
|---|---|
| Does `VK_KHR_ray_query` work from our Slang kernels, with a procedural-box candidate and a triangle BLAS in one TLAS? | A correct image of a gate (box minus box) and a mesh, compared by eye and then by the CPU oracle |
| Renders a second at the two P1 configs, 128 envs, 1 ray a pixel, N = 1? | 320x180 colour and 384x384 grey fisheye; the target is 12,000 a second each. Report separately: kernel time, TLAS build and update, the pose upload, the export-buffer write. For the quantity per ray: rays a second and a ray cost |
| What do AA 2x2 and blur N = 2 and 4 cost? | The same configs with the multipliers, so the budget says what a knob buys |
| Lattice (L) or per-env TLAS (E)? | Both, at 128 and 1,024 envs: build time, VRAM, trace time, and for L the hit error at the far corner against the CPU oracle |
| Memory? | VRAM by `VK_EXT_memory_budget`: structures, tables, three slots, scratch; at 128 and 1,024 envs; with and without the learner's CUDA context resident (KAT measures that side) |
| Does the export cost anything? | Time a frame with the buffer exported against a plain device buffer; and a CUDA-side import test is KAT's, on a buffer Spade hands over |
| Is it deterministic? | The same scene and tick rendered twice, in two processes, and after a reset and a restore: `memcmp` of the whole output. If acceleration-structure builds are not reproducible on this driver, this is where it shows, and the answer is a band, not a promise |
| What do 6a's extras cost? | A textured variant (a procedural pool of 128 at 512x512, 128 envs on different textures, triplanar on the gate boxes) and a 64-distractors-an-env variant against the plain one: ray cost, texture bandwidth, TLAS size; and sample frames written for KAT |
| Is a pixel right? | Against a small CPU oracle: id agreement and depth on the silhouette, and a colour check on a known palette |

**GPU time the spike needs (estimates, in GPU-queue minutes a job occupies the card, startup included; not measurements).** The spike runs only after KAT's review and the lead's go; anything over 30 GPU minutes goes to the lead first, and the total below is over it, so this is that notice. Every job is a `gpu` ticket at `--vram-gb 4`, never `--gpu-alone`.

| Variant | What runs | GPU min |
|---|---|---|
| V1 P1 colour, plain | 320x180, 128 envs, 1 ray a pixel, three timed repeats | 3 |
| V2 P1 fisheye, plain | 384x384 grey KB4, 128 envs, three repeats | 3 |
| V3 knobs | AA 2x2, blur N = 2 and 4, on both configs (six runs of about a minute) | 5 |
| V4 structure | lattice against per-env TLAS, at 128 and 1,024 envs (four runs), with the memory reading taken in the same runs | 6 |
| V5 export | exported buffer against a plain device buffer, the timeline semaphore round trip | 3 |
| V6 determinism | two processes, a reset and a restore, `memcmp` | 3 |
| V7 oracle | the pixel check against the CPU oracle (the CPU part is not GPU time) | 2 |
| **Core spike, V1 to V7** | | **25** |
| V8 textured variant | procedural pool of 128 at 512x512, 128 envs on different textures, triplanar gates; the VO sample sequences | 6 |
| V9 distractors | 64 an env, the TLAS size and the trace cost | 4 |
| **6a variants, V8 and V9** | | **10** |
| Reruns after a fault | an allowance, not a plan | 10 |
| **Total** | | **about 45** |

The core spike alone is under the 30. The proposal is two approvals: the core spike first (about 25), then the 6a variants (about 10) after its result, so no single approval passes 30 and the textured variant is only run if the core numbers leave room for it. The CPU oracle's time and KAT's CUDA-side import test are not on this card's account.

**Scale check, from the paper (arithmetic, not a prediction).** HyperDrone's 4090 numbers at 64x64 are about 2.3 G pixels a second at High and 7.2 G at Low (569,534 and 1,749,660 frames/s x 4,096 px, as read). KAT's P1 needs 12,000 x 57,600 = 0.69 G pixels a second (colour) and 12,000 x 147,456 = 1.77 G (fisheye) at one ray a pixel. A 3060 Ti has 38 SMs against the 4090's 128, so scaling by SM count alone gives about 0.7 G (High) to 2.1 G (Low) for this card. The targets sit inside that range at one ray a pixel and **outside it with 2x2 AA or N = 2 blur on the fisheye** (3.5 G to 7 G). The spike's job is to turn this into measured numbers. If the 12,000 a second is missed, the decision comes back to the lead and KAT with the measured cost of each knob (`A6`'s design review), not a quieter design.

**Budget rule (proposed).** The camera pass at P1 gets a stated share of the iteration (the target in ask 1 gives about 4.8 s of render per camera type at 12,000/s against an iteration of about 10 s today, a ceiling of about 20 s). A camera budget of at most 1 GB of VRAM at 128 envs is the proposal, OPEN with KAT because the learner shares the card.

## 9. Determinism, bands and tests

- **CPU reference (`reference`):** a CPU ray tracer over the same scene description, in fp64: the existing `sphere_trace` for SDF shapes (`render/raymarch`) plus a triangle BVH on the CPU. It is the oracle and the source of goldens (CPU only, two-platform promotion, `L4`). It shares the lens table, the shading functions and the noise hash with the GPU path, so what differs is traversal, not definitions.
- **GPU (`banded`):** per pixel, the instance id must agree except on silhouette pixels, and the number of disagreeing pixels is a measured, versioned, pinned band (`SR-30`, `SR-31`: every band proves its detection surface, bands are versioned); depth agrees within a relative band; colour agrees within a measured tolerance on a known-palette scene (the colour oracle `03-verification.md` requires before a GPU technique is selectable); keypoint (u, v) within a stated pixel error; visibility flips only within `occlusion_epsilon` of a surface.
- **Same-backend equality:** the same inputs give the same bytes on the same GPU, across replay, reset and restore, **if the spike shows it holds**; the LiDAR's rule (L78) is the model, and a stated band if it does not.
- **Cadence invariance (`L5`):** rendering never changes stepping. The camera pass reads published state only; a test renders zero, one and every tick and compares the world's hash (the test `03-verification.md` says is missing in this repo, added here).
- **The camera is derived state, not snapshot state:** the camera's configuration is a row (snapshot, hash), but the pixels and pool slots are not. A world with no camera module is untouched bit for bit, as the LiDAR's opt-in rule says.
- **A frame is a pure function of (scene, poses, camera, tick, seed)** with no clock and no static mutable state (the rule for every technique).

## 10. Phases and sizes

Sizes are my estimates in realm sessions (S = one, M = two to four, L = five or more), not commitments.

| Phase | What | Owner | Size | Gate |
|---|---|---|---|---|
| 0. Spike | Section 8 (with the textured and distractor variants), standalone in the bench tree | Rendering | M, plus S for the 6a variants | The numbers go to the lead and KAT before Phase 2 is designed in detail. Miss of 12,000 a second: design review (`A6`) |
| 1. Camera model and CPU reference | Lens table (pinhole, KB4), rays, the CPU ray tracer, image, id and corner channels, goldens, the `atan` question | Rendering | M | CPU goldens on two platforms; lens tests against fp64 |
| 2. The GPU technique | Device scene image, BLAS and TLAS (L or E), batched dispatch, shading port, bands, colour oracle, the announced fallback | Rendering | L | Banded agreement; the P1 throughput from the spike, held |
| 3. The sensor kind | Row, sample, rate divider, capture-tick stamp, capacities, pool, C ABI | Core, with Rendering | M | Needs Phase 1's channel list; `kinds.hpp`'s rule (row before tag) |
| 4. Export | Linear buffer with external memory, semaphores, the pool, UUID and LUID, a C ABI surface; KAT's CUDA import test | Rendering and KAT | M | KAT reads a frame in CUDA with no host copy, bytes equal |
| 5. Labels | Instance labels, keypoints, visibility | Rendering | S | The gate corners as data from KAT agree with a hand-computed projection |
| 6. Sensor realism | Exposure, noise, motion blur, mode | Rendering | M | Seeded replay equality; noise statistics test |
| 6a. Seeded appearance, no textures | The appearance set and the episode draw (host, counter-based streams), the appearance record, material and colour variety, sky gradients, lighting ranges, the exposure draw | Rendering, Core (keys) | M | Same (set, seed, env, episode) gives the same record in two processes and after reset; CPU goldens for fixed seeds |
| 6b. Distractors | Templates, placement by bounded rejection sampling, keep-clear volumes, per-class labels, instance capacity | Rendering | M | Deterministic placement; the TLAS instance cost measured in the spike's variant |
| 6c. Textures | The texture pool buffer with mips, the luminance copy, glTF UVs, triplanar for SDF shapes, the planar ground, procedural patterns, the explicit bilinear and trilinear filter, `RND-13` | Rendering | L | The filter grid test (CPU against GPU, at most 1 level in 255, counted); the textured band; sample frames to KAT for a feature-detector check |
| 7. Native views | P2: 640x360 colour and 848x800 grey, the editor window and offline dataset path, host readback | Rendering, Interface | S | Pictures match the batched path |
| 8. LiDAR on the shared scene | Out of scope here; a ray generator and writer on the same device image | Core, Rendering | L | After 2 |

Phases 1 and 3 can run in parallel after Phase 0. The editor window (Interface) can use the CPU reference from Phase 1 for single views before the GPU lands.

## 11. What Rendering needs from Core

1. **A `camera` sensor kind**: a row (mount pose, lens id, resolution, rate divider and phase, shutter fraction, blur and AA samples, mode, exposure gain, mask bits), a sample (the stamp header), an rng domain tag for the noise stream, and the poll or export path. Added with its row, not ahead of it. The `CameraComponent` (id 7) gets its sample type.
2. **Capacities** declared like `capacities.lidar_scan_rays`: `capacities.cameras` (default 0), `capacities.camera_pixels` (the output budget), `capacities.keypoints`, and a world with the defaults allocates nothing.
3. **A C ABI** for: adding a camera to a body with a spawn struct; setting a lens (the table or the intrinsics and k1..k4); setting labels and keypoints on an object; acquiring, reading the stamp of, and releasing a pool slot; exporting the buffer and semaphore handles with the device ids. The same two-phase spawn and distinct typed refs as the IMU and the LiDAR (`poll_imu(camera_ref)` refused at compile time).
4. **The rate divider and the capture tick** from the same scheduler the IMU uses, and a frame stamped with the tick it is rendered from.
5. **A published-state seam:** the camera reads body poses from a published (read-only) view of a tick, with the previous tick's pose for blur interpolation. See the Physics ask for the device side.
6. **The labels and materials**: a per-object label (uint16) in the world file or the API, and material resolution as the render scene does it today; Core owns the world-file keys, Rendering the meaning (`RS5`).
7. **Seed plumbing and the appearance set (D190):** the world seed and a per-env episode counter reaching the draw at reset; a render-only appearance set in the world file or the API (material pool, texture pool entries as decoded arrays or procedural recipes, sky pool, light ranges, distractor templates, keep-clear volumes); the appearance record readable by the caller; `capacities` for texture bytes, sky entries, and `camera_instances` an env. Render-only, so no change to a world that does not use it.

## 12. What Rendering needs from Physics (one-GPU scheduling)

The card is shared by the physics step (and the flow solver), the camera, and KAT's CUDA learner. Vulkan and CUDA work on one device are time-sliced by the driver; they do not run concurrently by default, so the order is a design input.
1. **Poses without a readback.** When the world set steps on the Vulkan backend, the body poses already live in the device arenas (`state_mirror`). The camera pass should read them there (or from a copy the step writes at the end of the tick, inside the same queue submission) rather than through a host copy. Needs a stated device-side, read-only pose view and the tick's boundary.
2. **Order and ownership in one iteration:** step, then camera, then (KAT) policy and learner. Who signals whom: the step's completion semaphore gates the camera; the camera's timeline semaphore gates KAT's CUDA read. A single submission chain, no host stall between.
3. **A VRAM budget** for the camera (section 8 proposes at most 1 GB at 128 envs) beside the world set's and the flow solver's; one place that sums them on the 8 GB card.
4. **Which GPU,** where a machine has two, and that the camera and KAT's CUDA use the same one.
5. **Priority:** flow solves for the airflow films are long and heavy; a training run and a flow run are never both on the card (the queue's `gpu` class already enforces it for tickets; the in-process question is for Physics).

## 13. Risks, and what is deliberately not here

- **Throughput is the risk, not memory.** Section 8's scale check says P1 is plausible at one ray a pixel and not with the multipliers on the fisheye. The knobs are priced before anyone relies on them.
- **Ray-query support and Slang.** If `RayQuery` does not compile through our toolchain, the spike says so in the first hour; the alternatives (a plain compute march over a BVH, or a GLSL side-path) are slower and are a lead decision, not a quiet switch.
- **Acceleration-structure determinism** is a driver property; the spike measures it, and the band is the fallback.
- **Dynamic-body blur** is not in v1; if KAT's scenes put moving bodies in view, Phase 6 grows by a TLAS update a sample.
- **Texture and distractor cost.** 6a is the first thing that can push P1 over its budget after blur and AA (section 8): the spike prices it before 6b and 6c are sized, and the procedural pool is the cheap first step.
- **Not here:** textures on the editor viewport and the rasters (`RND-13` covers the camera sensor only), shadows by default, rolling shutter, depth, normals, flow, lens-flare or any post-processing (`RS15`'s exclusion stands), the LiDAR beyond sharing the scene, and any CUDA code in Spade.

## 14. Open items, for the lead and KAT

| # | Item | Who |
|---|---|---|
| 1 | Tilt and mount frame signs, per camera | KAT |
| 2 | Gamma: linear uint8, or an sRGB curve, per camera | KAT |
| 3 | Moving bodies in view at P1 (blur needs them or not) | KAT |
| 4 | Shadows wanted in training frames | KAT |
| 5 | Empty keypoint label: 0 or -1; label width uint16 or int32 | KAT |
| 6 | Per-env intrinsics (FOV jitter) as a table an env, or mount-only | KAT |
| 7 | Windows only for training, or Linux too (handle type, the Docker leg) | KAT, lead |
| 8 | Shared ray scene: where it lives and who owns it | Core, Rendering, lead |
| 9 | `atan` in `spade::math` or a new pinned kernel | Rendering, Core |
| 10 | VRAM share and same-GPU rule | Physics, lead |
| 11 | Texture pool budget (proposal 256 MB at 128 envs) and the sources: decoded arrays from KAT, procedural, or both | KAT |
| 12 | Distractors render-only (proposal), or any that must collide (poles) | KAT, Physics |
| 13 | `RND-13`: amend `RS6` and `RS15` for the camera sensor (textures; D179 and D190 are the authority) | lead |
| 14 | Cube-map skies as a separate pool: wanted or not (out of 6c unless KAT's own design asks) | KAT |
