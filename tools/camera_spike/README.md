# camera_spike

The core spike for the camera sensor design (`docs/design/rendering/2026-11-19-camera-sensor-design.md`, section 8, V1 to V7). A standalone Vulkan ray-query benchmark. It links no engine library, changes no engine API and is not installed. Build: `scripts/build.ps1 -Preset release -Target camera_spike` (behind `SPADE_VULKAN`). It needs `VK_KHR_ray_query` and `VK_KHR_acceleration_structure`, and refuses with the reason where the device lacks them.

```
camera_spike bench  --config colour|fisheye --envs N --aa A --blur B --structure L|E [--dynamic] [--export] [--frames F] [--repeats R]
camera_spike oracle --config ... --envs N --structure L|E      GPU frame against an fp64 CPU ray tracer, env 0 and the last env
camera_spike det    --config ... [--out FILE]                   same scene twice, and after a full rebuild; FILE holds the output
camera_spike cmp    FILE_A FILE_B                               across two processes
camera_spike export                                             win32 external memory and timeline semaphore: handles, import back
```

- **Scene:** per env four gates (outer box minus inner box, ONE procedural box candidate with an in-shader CSG sphere trace, epsilon 1e-4), a 4,096-triangle torus (hardware triangles), an analytic ground. Lambert + ambient + sky, sRGB from a 1,024-entry table, grey is luminance.
- **Lenses:** `colour` is a 320x180 pinhole (fx = fy = 160, cx = 160, cy = 90); `fisheye` is a 384x384 KB4 (fx = fy = 193, cx = cy = 192, k1..k4 = -0.008, 0.002, -0.0005, 0.0001, monotonic to 82 degrees). Rays come from a per-type direction table.
- **Structures:** `L` is one TLAS holding every env on a 100 m lattice; `E` is one TLAS an env (an array of TLAS indexed by env, compiled for 128 or 1,024 envs).
- **Camera path:** 2.5 s long at 30 Hz, so 75 frames is one period; blur samples at t - 0.5 x period x (1 - k/(N-1)).
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
