# Drone sim box — design

**Status:** approved in conversation 2026-10-01. **Purpose:** the sandbox's new default scene. It shows end to end, in the GUI, that what Spade claims to exist actually works — quadrotor, rotor model, integrator, Dryden turbulence, CPU stepping, and both render paths — and it makes any gap visible. User-directed exception to the restructure's feature pause (`2026-10-01-spade-restructure-design.md` §5).

## What the user sees

- **An empty world** (no ground) with a quadrotor at the origin. The drone never translates; the air moves around it, like a model in a wind tunnel.
- **Attitude keys:** arrows for pitch and roll, `Z`/`X` for yaw. Each key nudges a target that holds when released. Pitch and roll are limited to ±60°; `R` returns to level.
- **Camera:** always faces the drone. `A`/`D` move around it, `Q`/`E` move up and down, `W`/`S` move nearer or further, clamped to a 0.5–6 m view sphere. Mouse drag still orbits.
- **Two views,** toggled with `V`:
  - **Standard:** the shaded drone.
  - **Air-velocity heatmap:** a colour-mapped slice of air speed, with a legend.
- **Physics panel:** wind speed and heading, turbulence level (none, light, moderate or severe), air density, gravity, throttle as a percentage of hover, and backend (CPU or Vulkan).
- **Readouts:**
  - attitude and body rates;
  - each rotor's speed and thrust;
  - net moments;
  - hover induced velocity;
  - IMU accelerometer and gyro;
  - tick count;
  - physics and render milliseconds per frame;
  - the active backend and render path.

## How it works

**Simulation.**
- `WorldBuilder` builds a world with no geometry. That is valid: SDF distance is `FLT_MAX` and ground effect is 1.
- The world holds one instance, with `turbulence = dryden_params(level)`.
- The airframe comes from `make_quadrotor` with mass **1.0 kg**, registered and spawned at the origin, then `flush_structural`.
- Stepping uses a fixed-dt accumulator: `dt` = 2 ms, 2 substeps, at most 100 steps per frame.
- The sandbox links `spade::sim`. This is the first time the sandbox steps a simulation.

**Holding the drone in place — two sandbox behaviors, CPU, no engine change.**
- Kinematic slot: set `pos = 0` and `vel = 0`.
- Force slot: set `force_acc = −mass·gravity`, and leave `torque_acc` alone.
- Integrate then adds exactly `+gravity` back, so `vel` stays bitwise 0 for a power-of-two mass.
- Side effect: the IMU reads +g, as a stand-mounted sensor would.
- **On Vulkan behaviors do not run today** (restructure defect 1). So selecting Vulkan in this scene is **refused, with a visible message**, until the engine has a translation-lock constraint on both backends. That constraint is future Core work in the Constraints phase.

**Attitude control (scene code — this is template material, not engine).**
- Targets for yaw, pitch and roll come from the keys.
- A PD controller on the attitude error, with damping on body rates, produces the desired moments.
- A mixer converts those moments to four rotor speeds around the throttle trim (`hover_command(params, g)` × throttle). It uses the engine's layout: plus frame, nose along +X, rotors 0–3 at +X, +Z, −X, −Z, spin +1, −1, +1, −1, roll `L(T3−T1)`, pitch `L(T0−T2)`, yaw `−(Q0−Q1+Q2−Q3)`.
- The rotor speeds are sent with `set_rotor_commands`. The engine's rotor model (lag, inflow, torque) and integrator then do the rest.

**Changing a physics option** rebuilds the Simulation, debounced, carrying the attitude, body rates and rotor speeds across. Every one of these values sits in `config_hash`, so they are fixed per Simulation by design.

**Air field (analytic, CPU, read only for visualisation — stepping never reads it).** Velocity at a point is:

    v(p) = medium(p) + Σ_rotors wake_i(p)

- `medium(p)` is wind plus the current Dryden gust. It comes from a new public read accessor, **`Simulation::sample_medium(world, pos) -> Result<MediumSample>`** (Core).
- `wake_i(p)` is an actuator-disc slipstream, given as a pure function in `engine/vehicles/` (Physics):
  - **Inputs:** rotor hub position, thrust axis, radius `R`, `k_T`, `ω`, density, and the freestream.
  - **Disc velocity:** `v_i = v_h·λ`, where `v_h = rotor_hover_induced_velocity(k_T ω², ρ, R)` and `λ = rotor_inflow_factor(...)`.
  - **Axial profile:** `v(s) = v_i (1 + s/√(s²+R²))`, where `s` is distance downstream along the wake axis. That gives `v_i` at the disc, `2v_i` far downstream and 0 far upstream.
  - **Tube:** contracts by continuity, with a smooth edge.
  - **Wake axis:** skewed along `−axis·2v_i + freestream`.
- There is no obstruction by the frame. This is honest for what the engine models; a solved flow field can replace it later as a higher-fidelity field provider.

**Rendering (GL path and CPU fallback, no engine change).**
- **Drone parts:** a body box, four arm boxes (the +X nose arm coloured differently) and four rotor discs (thin cylinders). Each part's `DrawItem` is `pose × part_offset`, built from `quadrotor_arm_offset`.
- **Heatmap slice:**
  - **Placement:** the body plane through one pair of opposite rotors, containing the thrust axis and that arm. Of the two arm pairs, it uses the one whose plane faces the camera more squarely. It is 2 m wide and 2.5 m tall, extending further below the drone along the thrust axis for the downwash. *(Amended 2026-10-02 by the lead after Physics' review: the original "vertical plane facing the camera" missed both plumes whenever the camera was 28–62° from an arm, so the downwash vanished every 90° of orbit. With the body plane, the plumes are always in-plane, at any attitude, and the plane is never more than 45° off facing the camera.)*
  - **Cells:** 64 × 64 instances of one double-sided quad.
  - **Colour:** each cell takes `material_override = palette_base + bin(|v|)`, using 32 unlit viridis-style palette materials appended once after the builder materials.
  - **Speed range:** auto, from 0 to the maximum, with a manual override.
  - **Drone:** drawn depth-tested through the slice.
  - **Shadows:** off in the heatmap view.
- The CPU helper `render_frame` gains an options parameter.

**Lighting defect folded in (restructure defect 4).** GL lights with `−sun_direction` while the CPU path uses `+sun_direction`, so this scene would light the drone differently on each path. Rendering settles the sun-direction convention, including whether the default direction is below the horizon, and fixes it before this scene is called done.

**Flags.** In window mode, `--scene drone|builder` with `drone` the default. `--headless` gains `--scene` and `--view standard|heatmap`. `--smoke` stays on the builder scene.

## Tests (display-free, `tests/test_sandbox_drone.cpp` plus engine unit tests)

- **Mixer:** each moment axis drives the correct rotors with the correct sign.
- **Controller:** in a CPU simulation, a pitch, roll or yaw target is reached within tolerance.
- **Pin:** `pos` and `vel` stay bitwise 0 across 1,000 steps, with the controller active.
- **Wake** (engine unit tests): `v_i` at the disc, `→2v_i` far downstream, `→0` far upstream, zero with `ω = 0`, and the expected axis skew under crosswind.
- **Field:** with the rotors stopped, `v(p)` equals the medium sample.
- **Heatmap:** a known slice pixel equals `to_byte(palette[bin(|v(p)|)])` exactly on the CPU path. Standard and heatmap renders differ.
- **Rebuild:** a physics-option change carries attitude, body rates and rotor speeds across.
- **Vulkan in this scene:** refused with a message, and never silently stepped without the pin.

## Ownership

| Realm | Work |
|---|---|
| Core | `Simulation::sample_medium`; defect 1 (refuse attached behaviors on Vulkan) |
| Physics | the rotor wake function and its tests; reviews the controller and mixer |
| Rendering | heatmap palette and cells; defect 4 (sun convention) |
| Interface | the scene, input, camera, panel, flags, accumulator, rebuild, and sandbox tests |
| Lead | sequences builds; reviews across realms |
