# camera_spike

The core spike for the camera sensor design (`docs/design/rendering/2026-11-19-camera-sensor-design.md`, section 8, V1 to V11). A standalone Vulkan ray-query benchmark. It links no engine library, changes no engine API and is not installed. Build: `scripts/build.ps1 -Preset release -Target camera_spike` (behind `SPADE_VULKAN`). It needs `VK_KHR_ray_query` and `VK_KHR_acceleration_structure`, and refuses with the reason where the device lacks them.

```
camera_spike bench  --config colour|fisheye --envs N --aa A --blur B --structure L|E [--dynamic] [--export] [--frames F] [--repeats R]
camera_spike oracle --config ... --envs N --structure L|E      GPU frame against an fp64 CPU ray tracer, env 0 and the last env
camera_spike det    --config ... [--out FILE]                   same scene twice, and after a full rebuild; FILE holds the output
camera_spike cmp    FILE_A FILE_B                               across two processes
camera_spike export                                             win32 external memory and timeline semaphore: handles, import back

camera_spike genpool  --tex-cache FILE                          V8: write the 128-texture pool (CPU only, under a second)
camera_spike bench    ... --gates G --distractors D             V8/V9: a denser scene (the default is 4 gates, none: the oracle's scene)
camera_spike bench    ... --textured --tex-cache FILE [--tex-seed S]   V8: texture the gates, distractors and ground; prints texture traffic
camera_spike bench    ... --mix [--mix-blur-max M] [--mix-layout grouped]   V10: per-env AA and blur inside one dispatch
camera_spike texcheck --tex-cache FILE                          V11: the texture filter, GPU against the CPU reference, on a dense (u, v) grid
camera_spike samples  --tex-cache FILE --out DIR [--configs colour,fisheye] [--env-list 0,1,2]   V11: PNG fly-throughs and a text file of poses
```

- **Scene:** per env four gates (outer box minus inner box, ONE procedural box candidate with an in-shader CSG sphere trace, epsilon 1e-4), a 4,096-triangle torus (hardware triangles), an analytic ground. Lambert + ambient + sky, sRGB from a 1,024-entry table, grey is luminance.
- **Lenses:** `colour` is a 320x180 pinhole (fx = fy = 160, cx = 160, cy = 90); `fisheye` is a 384x384 KB4 (fx = fy = 193, cx = cy = 192, k1..k4 = -0.008, 0.002, -0.0005, 0.0001, monotonic to 82 degrees). Rays come from a per-type direction table.
- **Structures:** `L` is one TLAS holding every env on a 100 m lattice; `E` is one TLAS an env (an array of TLAS indexed by env, compiled for 128 or 1,024 envs).
- **Camera path:** 2.5 s long at 30 Hz, so 75 frames is one period; blur samples at t - 0.5 x period x (1 - k/(N-1)).
- **Dense scene (V8, V9):** `--gates G --distractors D` puts G gates down the track (z = 2 + 3g) and D unlabelled distractors (half flat boxes, half 3 m poles, each ONE procedural box with a short march) over the 32 m the camera sees; the mesh is last. Base is 10 gates + 20 distractors, dense is 20 + 64. Each shape kind has its own one-AABB BLAS; the instance custom index is (kind << 20) | (variant << 16) | label.
- **Textures (V8):** 128 procedural textures (checker, stripes, six-octave value noise, a feature-rich noise-checker-dots-lines pattern), 512x512 RGBA8 with a full mip chain, 170.7 MiB in ONE storage buffer plus a (base word, size, mips, kind) table (`texpool.hpp`, `tex.slang`). No hardware sampler: integer texel loads, an own bilinear inside a mip and a linear blend between two mips, both fractional weights quantised to 8 bits, the mip from the pixel footprint (angle x range / cos, scaled to mip-0 texels) with log2 taken from the float's exponent bits (no log2, no pow). Gates and distractors use box mapping in object space (the dominant axis of the object-space normal, not a three-way blend), the ground is planar (x, z) with a per-env offset; all are fixed to the surface. Each env gets a gate, a ground and a distractor texture, 128 envs on 128 different gate textures (`(37 e + 11 seed) mod 128`). The mesh is not textured.
- **Per-env counts (V10):** with `--mix` the kernel reads (aa, blur) per env from a buffer: even envs AA 2x2 with one shutter sample, odd envs one ray with blur 2, 3, 4 in turn. Envs are dispatched env-major (one z slice an env), so a warp never straddles two envs. AA is 1 or 2 only (the ray tables are 1x and 2x).
- **Shader:** `camera_spike.slang`, compiled by its own slangc rule without `-fp-mode precise` (the camera is banded, not parity).

## Measured on 2026-11-19, RTX 3060 Ti 8 GB, driver 572.83, commit a03e3688

Mean over whole-path repeats, 128 envs, one ray a pixel unless stated. Kernel time is a GPU timestamp pair; wall time is host pose computation plus submit to wait. The target is 12,000 renders a second a camera type.

| Run | kernel ms | renders/s (kernel) | renders/s (wall) |
|---|---|---|---|
| V1 colour 320x180 | 1.36 | 94,000 | 82,000 |
| V2 fisheye 384x384 grey | 2.72 | 47,000 | 40,000 |
| colour, AA 2x2 | 5.05 | 25,000 | 24,000 |
| colour, blur 2 | 2.51 | 51,000 | 38,000 |
| colour, blur 4 | 5.04 | 25,000 | 24,000 |
| colour, AA 2x2 + blur 2 | 10.0 | 12,800 | 12,100 |
| fisheye, AA 2x2 | 10.7 | 11,900 | 11,600 |
| fisheye, blur 2 | 5.30 | 24,000 | 23,000 |
| fisheye, blur 4 | 10.8 | 11,900 | 11,400 |
| fisheye, AA 2x2 + blur 2 | 21.4 | 6,000 | 5,800 |

The scene is light (about 5 instances an env, mostly sky and ground), so these are a ceiling for the structure, not a prediction for a dense track.

## Second round, 2026-11-19 (V8 to V11), same card, head of `rendering/camera-spike`

E structure, 128 envs unless stated, kernel time is a GPU timestamp pair (mean over whole-path repeats, 75 frames each), wall time adds the host pose computation and the submit to wait. Texture traffic is counted by an atomic over one whole camera period in a separate, untimed pass.

**V8, textured against untextured, one ray a pixel** (base = 10 gates + 20 distractors, dense = 20 + 64):

| Config, scene | untextured ms | textured ms | texture cost | textured renders/s (kernel / wall) | texture samples a pixel | requested GB/s |
|---|---|---|---|---|---|---|
| colour, base | 1.391 | 1.402 | +0.8% | 91,300 / 76,100 | 0.64 | 108 |
| colour, dense | 1.468 | 1.564 | +6.5% | 81,800 / 72,000 | 0.69 | 103 |
| fisheye, base | 2.766 | 2.952 | +6.7% | 43,400 / 27,900 (one slow wall run) | 0.59 | 122 |
| fisheye, dense | 3.019 | 3.227 | +6.9% | 39,700 / 36,600 | 0.62 | 116 |
| colour AA 2x2 + blur 2, dense | 11.47 | 11.84 | +3.2% | 10,800 / 9,950 | 5.49 | 109 |

Reading: a filtered sample reads 2 mips x 4 texels x 4 bytes = 32 bytes, so the textured scenes ask the pool for 100 to 130 GB/s before the caches (the card's DRAM is 448 GB/s) and cost 1 to 7% of the kernel: the texture traffic is not the bottleneck, the traversal is. A second episode (`--tex-seed 5`, fisheye base) reads 3.014 ms (+2% over seed 0). The pool costs 170.7 MiB of VRAM (the 128-env textured run's whole delta is 248 to 269 MiB). The colour default (AA 2x2 + blur 2) on the dense scene is 11,159 renders/s untextured and 10,811 textured, below the 12,000 target at 128 envs (it was 12,800 on the light V3 scene).

**V9, 75 to 85 instances an env** (10 gates + 64 distractors, colour):

| Structure, envs | trace kernel ms (renders/s) | TLAS size | TLAS build GPU ms | TLAS update GPU ms (the mesh moves) |
|---|---|---|---|---|
| E, 128 | 1.546 (82,800) | 3.0 MiB (3.5 updatable) | 0.28 | 0.117 |
| L, 128 | 1.577 (81,200) | 2.8 MiB | 0.28 | 0.104 |
| E, 1,024 | 11.73 (87,300) | 24.0 MiB (27.8 updatable) | 2.21 | 0.769 |
| L, 1,024 | 13.42 (76,300) | 22.1 MiB | 1.33 | 0.690 |
| E, 1,024, 20 gates + 64 distractors, textured | 12.65 (81,000) | 27.4 MiB | 3.29 | n/a |

Each per-env TLAS is about 23 KB (312 bytes an instance) with 10 KB of build scratch and 4 KB of update scratch, so 1,024 of them are 24 MiB of structure and the scratch is a fixed cost, not a per-frame one. Going from 5 to 75 instances an env costs the E kernel 31% at 128 envs (1.18 to 1.55 ms) and 23% at 1,024 (9.5 to 11.7 ms). E stays ahead of L at 1,024 envs.

**V10, per-env AA and blur inside one batch** (fisheye, base scene, textured; 5 separate processes each, mean kernel ms):

| Batch | kernel ms | renders/s (kernel) |
|---|---|---|
| uniform AA 2x2 | 11.26 | 11,400 |
| uniform blur 3 | 8.56 | 15,000 |
| mean of the two uniform batches | 9.91 | |
| mixed (half AA 2x2, half blur 2..4) | 9.82 | 13,000 |
| mixed against the mean | -0.9% | |
| mixed, first half AA, second half blur (one run) | 9.70 against a 9.68 mean (+0.3%) | |

The blur half averages 3 samples, so the mixed batch traces 3.5 rays a pixel on average, the same as the mean of the two uniform batches. The colour config (5 processes): uniform AA 2x2 5.354 ms, uniform blur 3 3.928 ms, mean 4.641 ms, mixed 4.641 ms (0.0%). The colour default AA 2x2 + blur 2 uniform is 10.44 ms (12,260 renders/s). One earlier colour mixed run read 5.50 ms (+19%) in all three of its repeats; nine later runs in other processes read 4.46 to 4.71 ms, and that one was not reproduced (the cause is not known; the card may have been shared that minute). Per-batch AA and blur are therefore not needed on cost grounds: the dispatch is env-major, so a warp runs one count and the mixed batch costs what its rays cost.

**V11, sample frames and the filter check**
- `texcheck`: 9,437,184 samples (4 textures, one of each kind, 7 footprints from 0.5 to 700 texels, a 1536 x 1536 grid across the repeat edge at about 1,024 points a repeat) through the GPU filter and the CPU reference: all 28,311,552 channel values equal, none off by one. The weights are 8-bit integers and the arithmetic is the same float sequence on both sides, so equal is the expected result; it is a check of the shader, the buffer layout and the mip offsets.
- `samples`: 60 frames of the P1 colour (320x180 RGB) and the grey fisheye (384x384) from envs 0, 1 and 2 (gate textures checker, stripes, value noise), one PNG a frame (stored, uncompressed, 57 MB in all) and `samples.txt` with the intrinsics, k1..k4 and each frame's tick and capture-time pose. Written to `Vantage/Videos/Footage/Spade/2026-11-19/camera-spike-samples`.

What was not done: the fp64 oracle (V7) was not extended to textured frames (the filter alone is checked against the CPU, the traversal against the oracle on the untextured scene); distractor boxes and poles are not in the oracle either.
